"""chassis_bridge —— 底盘串口桥，mower-host 独占这个串口。

收发全包：订阅语义话题 → 组 11 字节帧 → 写串口；读串口 → 解 24 字节帧 →
发 /odom /imu/data_raw /PowerVoltage。

三件不能省的事：
  * 帧同步用滚动缓冲 + 帧尾/BCC 双校验。固件吐的字节流里每个 0x7B 前面是
    0xFC，任何要求"前一字节是帧尾"的判据都会一帧都解不出来
    （见 protocol.UplinkParser 的说明）。
  * **带 cmd_vel 超时看门狗**。固件侧没有可用的指令丢失保护：一旦停发速度
    指令，底盘会继续按最后一次速度跑。这个兜底只能放在桥里——所有速度指令
    都要经过这里写串口。
  * 不写 TF。odom_combined → base_footprint 由 ekf_node 出（publish_tf: true）。
  * **急停锁存 + 命令确认都在这里做**。固件的 emergency_stop 在 SelfTest_Run()
    里立刻清零、不锁存，CMD=8 也没有对应分支；上行帧里也没有 ACK 位。
    所以锁存由桥维护（锁存期间丢弃 /cmd_vel 并补零速），确认则靠上行帧里
    编码器实测的速度去"看效果"，结果发到 /cmd_ack。
"""
import json
import math
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int16, Empty, String, Bool

from .protocol import (
    Mahony, UplinkParser, parse_uplink,
    build_vel_frame, build_servo_frame, build_selftest_frame,
    build_emergency_frame, build_release_frame,
)

# 两套协方差：静止时更信编码器，运动时更信 IMU。
# 位置 xy 的方差静止时给 1e-9、动起来给 1e-3；姿态在 2D 下不可信，一律 1e6。
_COV_POSE_STILL = [1e-9, 0, 0, 0, 0, 0,
                   0, 1e-3, 1e-9, 0, 0, 0,
                   0, 0, 1e6, 0, 0, 0,
                   0, 0, 0, 1e6, 0, 0,
                   0, 0, 0, 0, 1e6, 0,
                   0, 0, 0, 0, 0, 1e-9]
_COV_POSE_MOVING = [1e-3, 0, 0, 0, 0, 0,
                    0, 1e-3, 0, 0, 0, 0,
                    0, 0, 1e3, 0, 0, 0,
                    0, 0, 0, 1e6, 0, 0,
                    0, 0, 0, 0, 1e6, 0,
                    0, 0, 0, 0, 0, 1e3]
_COV_TWIST_STILL = [1e-9, 0, 0, 0, 0, 0,
                    0, 1e-3, 1e-9, 0, 0, 0,
                    0, 0, 1e6, 0, 0, 0,
                    0, 0, 0, 1e6, 0, 0,
                    0, 0, 0, 0, 1e6, 0,
                    0, 0, 0, 0, 0, 1e-9]
_COV_TWIST_MOVING = [1e-3, 0, 0, 0, 0, 0,
                     0, 1e-3, 0, 0, 0, 0,
                     0, 0, 1e6, 0, 0, 0,
                     0, 0, 0, 1e6, 0, 0,
                     0, 0, 0, 0, 1e6, 0,
                     0, 0, 0, 0, 0, 1e3]

# 固件上电后 uartx_callback.c:186 会丢掉 CONTROL_DELAY 之内的所有命令，
# Time_count 在 100 Hz 的 balance_task 里自增，CONTROL_DELAY=1000 → 10 秒。
BOOT_SILENCE_SEC = 10.0


class ChassisBridge(Node):

    def __init__(self):
        super().__init__('chassis_bridge')

        p = self.declare_parameter
        self.port = p('port', '/dev/mower_chassis').value
        self.baud = p('baud', 115200).value
        self.cmd_vel_timeout = p('cmd_vel_timeout', 0.5).value
        self.odom_frame_id = p('odom_frame_id', 'odom_combined').value
        self.robot_frame_id = p('robot_frame_id', 'base_footprint').value
        self.gyro_frame_id = p('gyro_frame_id', 'gyro_link').value
        # 里程计修正系数。当前全 1.0；若走 1 m 有偏差，按比例改这几个值。
        self.odom_x_scale = p('odom_x_scale', 1.0).value
        self.odom_y_scale = p('odom_y_scale', 1.0).value
        self.odom_z_scale_positive = p('odom_z_scale_positive', 1.0).value
        self.odom_z_scale_negative = p('odom_z_scale_negative', 1.0).value

        # 命令确认参数：上位机只能"观察效果"，不是帧级回执（固件没有序号/ACK 位）
        # 桥启动即进入急停锁存：断电重启/桥崩溃重启后，车不会因为
        # 一帧残留指令就动，必须由人显式「解除急停」。要放开设成 false。
        self.estop_on_start = bool(p('estop_on_start', True).value)
        # 软停保持时间：收到 /soft_stop 后这段时间内所有 /cmd_vel 强制按零速发，
        # 防止另一路发布者（Nav2 / 跟随）把零速盖掉
        self.soft_stop_hold = float(p('soft_stop_hold', 1.0).value)
        self.ack_timeout_vel = p('ack_timeout_vel', 1.5).value
        self.ack_timeout_estop = p('ack_timeout_estop', 0.6).value
        self.ack_tol = p('ack_tol', 0.05).value
        self.ack_ratio = p('ack_ratio', 0.25).value
        self.ack_frames = int(p('ack_frames', 3).value)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.volt_pub = self.create_publisher(Float32, '/PowerVoltage', 10)
        self.ack_pub = self.create_publisher(String, '/cmd_ack', 10)
        # 锁存状态用 latched 发布：界面上半路重启时也能立刻知道"现在是急停中"
        self.estop_pub = self.create_publisher(
            Bool, '/estop_state',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        self.create_subscription(Empty, '/estop_release', self._on_release, 10)
        self.create_subscription(Empty, '/emergency_stop', self._on_emergency, 10)
        self.create_subscription(Empty, '/selftest_cmd', self._on_selftest, 10)
        self.create_subscription(Int16, '/servo_cmd', self._on_servo, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Empty, '/soft_stop', self._on_soft_stop, 10)

        self._parser = UplinkParser()
        self._q = queue.Queue(maxsize=200)
        self._ahrs = Mahony()
        self._lock = threading.Lock()
        self._x = self._y = self._yaw = 0.0
        self._last_rx = None
        self._last_cmd = None
        self._stopped = False
        self._volt_count = 0
        self._estop = False          # 桥自己的急停锁存（固件那份不锁存）
        self._pending = None         # 待确认命令
        self._last_ack = 0.0
        self._last_zero = 0.0
        self._acked_cmd = None       # 已确认过的速度目标，避免持续流重复确认
        self._last_meas = (0.0, 0.0) # 最近一帧实测速度
        self._hold_until = 0.0       # 软停保持截止时刻
        self._selftest_watch = None  # 自检运动观测窗口

        try:
            import serial
        except ImportError:
            raise SystemExit('缺 pyserial：pip install pyserial')
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            # 开不了口就明确死掉。静默跑着什么都不做比崩了更危险 ——
            # 用户会以为桥在工作。
            raise SystemExit('打开串口 %s 失败: %s' % (self.port, e))

        self._running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
        # 帧的解析与所有 publish 都在这个定时器里（executor 线程）做，
        # 串口线程只负责读字节入队 —— 避免跨线程 publish
        self.create_timer(0.01, self._drain)
        self.create_timer(0.1, self._watchdog)
        # rclpy 没有一次性定时器 —— 不销毁它每 10 秒会再报一次"静默期结束"。
        self._boot_timer = self.create_timer(BOOT_SILENCE_SEC, self._boot_done)
        self._set_estop(self.estop_on_start)   # latched：后启动的界面也能拿到当前状态

        self.get_logger().info(
            'chassis_bridge 已起：%s@%d。固件上电后 %.0f 秒内不收命令，'
            '先别发指令。' % (self.port, self.baud, BOOT_SILENCE_SEC))

    # ---- 串口 ----

    def _read_loop(self):
        while self._running:
            try:
                chunk = self._ser.read(64)
            except Exception as e:
                self.get_logger().error('串口读失败，桥退出: %s' % e)
                self._running = False
                break
            if not chunk:
                continue
            for frame in self._parser.feed(chunk):
                try:
                    self._q.put_nowait(frame)
                except queue.Full:
                    pass        # 队列满就丢帧，宁可少一帧也不拖住串口线程

    def _write(self, frame):
        with self._lock:
            try:
                self._ser.write(frame)
            except Exception as e:
                self.get_logger().error('串口写失败: %s' % e)

    # ---- 收帧 → 话题 ----

    def _drain(self):
        while True:
            try:
                frame = self._q.get_nowait()
            except queue.Empty:
                return
            self._on_frame(frame)

    def _on_frame(self, frame):
        d = parse_uplink(frame)
        now = self.get_clock().now()
        # dt 用实测帧间隔，不用写死的 1/20 —— 帧率一偏，写死的那个就发散。
        dt = 0.0 if self._last_rx is None else (now - self._last_rx).nanoseconds * 1e-9
        self._last_rx = now
        dt = min(max(dt, 1e-4), 0.5)
        self._last_meas = (d['vx'], d['vz'])
        self._publish_odom(d, dt, now)
        self._publish_imu(d, dt, now)
        self._publish_voltage(d)
        self._check_ack(d)
        self._check_selftest()

    def _publish_odom(self, d, dt, now):
        vx = d['vx'] * self.odom_x_scale
        vy = d['vy'] * self.odom_y_scale
        vz = d['vz'] * (self.odom_z_scale_positive if d['vz'] >= 0
                        else self.odom_z_scale_negative)
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        self._x += (vx * c - vy * s) * dt
        self._y += (vx * s + vy * c) * dt
        self._yaw += vz * dt

        m = Odometry()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.odom_frame_id
        m.child_frame_id = self.robot_frame_id
        m.pose.pose.position.x = self._x
        m.pose.pose.position.y = self._y
        # 偏航只进四元数，不进 position.z —— 把 yaw 塞进 z 会让 EKF 拿到假高度。
        m.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        m.twist.twist.linear.x = vx
        m.twist.twist.linear.y = vy
        m.twist.twist.angular.z = vz
        still = not (d['vx'] or d['vy'] or d['vz'])
        m.pose.covariance = _COV_POSE_STILL if still else _COV_POSE_MOVING
        m.twist.covariance = _COV_TWIST_STILL if still else _COV_TWIST_MOVING
        self.odom_pub.publish(m)

    def _publish_imu(self, d, dt, now):
        q0, q1, q2, q3 = self._ahrs.update(d['gyro'][0], d['gyro'][1], d['gyro'][2],
                                           d['accel'][0], d['accel'][1], d['accel'][2], dt)
        m = Imu()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.gyro_frame_id
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q1, q2, q3, q0
        # 只信 yaw：单 IMU 的 roll/pitch 靠加速度计，车体振动下不可信。
        # ekf.yaml 的 imu0_config 也只取了 yaw 和 yaw_vel。
        m.orientation_covariance[0] = 1e6
        m.orientation_covariance[4] = 1e6
        m.orientation_covariance[8] = 1e-6
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = d['gyro']
        m.angular_velocity_covariance[0] = 1e6
        m.angular_velocity_covariance[4] = 1e6
        m.angular_velocity_covariance[8] = 1e-6
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = d['accel']
        self.imu_pub.publish(m)

    def _publish_voltage(self, d):
        # 电压每 11 帧发一次就够 —— 20 Hz 下来是 1.8 Hz，电池不会那么快变。
        self._volt_count += 1
        if self._volt_count > 10:
            self._volt_count = 0
            m = Float32()
            m.data = d['voltage']
            self.volt_pub.publish(m)

    # ---- 话题 → 发帧 ----

    def _on_soft_stop(self, msg):
        """软停：保持一段时间强制零速，不锁存、不需要解除。"""
        self._hold_until = time.monotonic() + self.soft_stop_hold
        self._pending = None        # 之前的待确认作废，避免停完再报"未跟上"
        self._acked_cmd = None
        self._write(build_vel_frame(0.0, 0.0, 0.0))
        self._ack('soft_stop', True, 0, 'hold_%.1fs' % self.soft_stop_hold)
        self.get_logger().info('软停：%.1f 秒内强制零速' % self.soft_stop_hold)

    def _on_cmd_vel(self, msg):
        if self._estop:
            # 锁存期间任何速度指令都当零速处理，并持续补零帧压住固件侧的最后速度
            self._write(build_vel_frame(0.0, 0.0, 0.0))
            self._ack('vel', False, None, 'estop_latched')
            return
        if time.monotonic() < self._hold_until:
            # 软停保持期：不管谁在发，一律零速；另一路发布者盖不掉
            self._write(build_vel_frame(0.0, 0.0, 0.0))
            self._ack('vel', False, None, 'soft_stop_hold')
            return
        self._last_cmd = self.get_clock().now()
        self._stopped = False
        self._write(build_vel_frame(msg.linear.x, msg.linear.y, msg.angular.z))
        vx, vz = msg.linear.x, msg.angular.z
        if not vx and not vz:
            # 停车指令不用确认，清掉待确认项
            self._pending = None
            self._acked_cmd = None
            return
        a = self._acked_cmd
        if a is not None and abs(a[0] - vx) <= self.ack_tol and abs(a[1] - vz) <= self.ack_tol:
            return          # 这个目标已经确认过，Nav2 20 Hz 持续流不再重复发确认
        m0 = self._last_meas
        if abs(m0[0] - vx) <= self.ack_tol and abs(m0[1] - vz) <= self.ack_tol:
            # 车本来就在这个速度上（例如滑行），无从判断指令是否生效，如实报 no_change
            self._ack('vel', True, 0, 'no_change')
            self._pending = None
            self._acked_cmd = (vx, vz)
            return
        p = self._pending
        if (p is None or p.get('cmd') != 'vel'
                or abs(p['vx'] - vx) > self.ack_tol or abs(p['vz'] - vz) > self.ack_tol):
            # 目标变了才重新计时；同一目标的连续指令不打断计数
            self._pending = {'cmd': 'vel', 'vx': vx, 'vz': vz,
                             't0': time.monotonic(), 'hits': 0}
            self._acked_cmd = None

    def _on_servo(self, msg):
        self._write(build_servo_frame(msg.data))
        if time.monotonic() - self._last_ack > 2.0:
            self._ack('servo', None, None, 'no_feedback')   # 上行帧没有舵机反馈

    def _on_selftest(self, msg):
        self._write(build_selftest_frame())
        # 固件自检第一拍会设 Vx=0.3（SelfTest_Run），所以"动不动"可以观测
        self._selftest_watch = {'t0': time.monotonic()}

    def _check_selftest(self):
        w = self._selftest_watch
        if w is None:
            return
        elapsed = int((time.monotonic() - w['t0']) * 1000)
        if abs(self._last_meas[0]) > 0.05:
            self._ack('selftest', True, elapsed, 'motion_seen')
            self._selftest_watch = None
        elif elapsed > 3000:
            self._ack('selftest', False, elapsed, 'no_motion')
            self._selftest_watch = None

    def _set_estop(self, active):
        self._estop = active
        m = Bool(); m.data = active
        self.estop_pub.publish(m)

    def _on_emergency(self, msg):
        self._set_estop(True)
        self._stopped = True
        self._write(build_emergency_frame())
        self._write(build_vel_frame(0.0, 0.0, 0.0))
        self._pending = {'cmd': 'estop', 't0': time.monotonic(), 'hits': 0}
        self.get_logger().warning('急停：桥已锁存，丢弃 /cmd_vel 直到 /estop_release')

    def _on_release(self, msg):
        self._set_estop(False)
        self._write(build_release_frame())
        self._pending = None
        # 固件没有 CMD=8 分支，这里解除的是"桥的锁存"；帧发出去只是留痕
        self._ack('release', True, 0, 'latch_cleared')

    # ---- 命令确认（观察效果，不是帧级回执） ----

    def _ack(self, cmd, ok, ms=None, why=''):
        now = time.monotonic()
        if why == 'estop_latched' and now - self._last_ack < 1.0:
            return          # 锁存期间每个 /cmd_vel 都会触发，限流 1 秒一次
        self._last_ack = now
        m = String()
        m.data = json.dumps({'cmd': cmd, 'ok': ok,
                             'ms': ms if ms is not None else 0,
                             'why': why}, ensure_ascii=False)
        self.ack_pub.publish(m)

    def _check_ack(self, d):
        p = self._pending
        if p is None:
            return
        t = time.monotonic()
        elapsed = int((t - p['t0']) * 1000)
        if p['cmd'] == 'vel':
            tol_vx = max(self.ack_tol, self.ack_ratio * abs(p['vx']))
            tol_vz = max(self.ack_tol, self.ack_ratio * abs(p['vz']))
            if abs(d['vx'] - p['vx']) <= tol_vx and abs(d['vz'] - p['vz']) <= tol_vz:
                p['hits'] += 1
                if p['hits'] >= self.ack_frames:
                    self._ack('vel', True, elapsed, 'speed_follows')
                    self._acked_cmd = (p['vx'], p['vz'])
                    self._pending = None
            else:
                p['hits'] = 0
                if t - p['t0'] > self.ack_timeout_vel:
                    self._ack('vel', False, elapsed, 'no_effect')
                    self._pending = None
        elif p['cmd'] == 'estop':
            if abs(d['vx']) <= self.ack_tol and abs(d['vz']) <= self.ack_tol:
                p['hits'] += 1
                if p['hits'] >= self.ack_frames:
                    self._ack('estop', True, elapsed, 'stopped')
                    self._pending = None
            else:
                p['hits'] = 0
                if t - p['t0'] > self.ack_timeout_estop:
                    self._ack('estop', False, elapsed, 'still_moving')
                    self._pending = None

    # ---- 看门狗 ----

    def _watchdog(self):
        if self._estop or time.monotonic() < self._hold_until:
            # 锁存/软停期间即使没有人发 /cmd_vel，也每 0.5 秒补一帧零速
            if time.monotonic() - self._last_zero > 0.5:
                self._write(build_vel_frame(0.0, 0.0, 0.0))
                self._last_zero = time.monotonic()
            return
        if self._last_cmd is None or self._stopped:
            return
        idle = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if idle <= self.cmd_vel_timeout:
            return
        self.get_logger().warning(
            '/cmd_vel 停了 %.1fs（> %.2fs），发零速度停车'
            % (idle, self.cmd_vel_timeout))
        self._write(build_vel_frame(0.0, 0.0, 0.0))
        self._stopped = True

    def _boot_done(self):
        self._boot_timer.cancel()
        self.destroy_timer(self._boot_timer)
        self.get_logger().info('固件静默期结束，现在可以发指令了')

    # ---- 收尾 ----

    def shutdown(self):
        self._running = False
        if self._last_cmd is not None:
            # 退出前把速度归零，别让车带着最后一条指令跑。
            self._write(build_vel_frame(0.0, 0.0, 0.0))
            time.sleep(0.05)
        try:
            self._ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ChassisBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
