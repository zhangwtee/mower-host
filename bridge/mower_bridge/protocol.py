"""串口协议。发 11 字节,收 24 字节。

字段位置是照着 mower-chassis 的源码一个个核的:
app/protocol.c 管收,app/telemetry.c 管回。

core/protocol.py 只管组发帧、不解析,算是这里的子集。
"""
import math

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D

# 发帧（上位机 → STM32）命令号。对照 mower-chassis/app/protocol.c 的分支，没有的别加。
CMD_VEL = 0         # 三轴速度，mm/s，16 位有符号大端
CMD_RECHARGE = 1    # 自动回充，同速度字段，走 charger.Up_Move*
CMD_DOCK = 3        # 红外对接，同速度字段，走 charger.Dock_Move*
CMD_SERVO = 4       # [3:4] = 脉宽 us，固件夹到 500~2500
CMD_SELFTEST = 5    # 无字段
CMD_EMERGENCY = 7   # 急停：固件锁存进 ESTOP，必须收到 CMD=8 才退出
CMD_RELEASE = 8     # 解除急停：固件靠它退出 ESTOP，还要求使能开关在位

TX_LEN = 11
RX_LEN = 24
DIST_LEN = 19       # 超声波帧，固件每拍跟在底盘帧后；解析在 DistanceParser

SERVO_MIN_US = 500
SERVO_MAX_US = 2500

# IMU 原始值 → 物理单位。量程在固件里写死了(±500°/s、±2g),
# 哪天 bsp/imu.c 改了这两个数,这里也得跟着改。
# 姿态已经由固件转到上位机坐标,这边只做单位换算。
GYROSCOPE_RATIO = 0.00026644    # ±500 °/s
ACCEL_RATIO = 1671.84           # ±2 g


def bcc(data, n):
    """前 n 字节逐字节异或。同固件 app/protocol.c 的 protocol_bcc。"""
    v = 0
    for i in range(n):
        v ^= data[i]
    return v


def _i16(d, i):
    v = (d[i] << 8) | d[i + 1]
    return v - 0x10000 if v & 0x8000 else v


def _u16(d, i):
    return (d[i] << 8) | d[i + 1]


def _s16_bytes(v):
    """16 位有符号 → 大端两字节。负数靠 & 0xFF 落成补码，
    与固件 XYZ_Target_Speed_transition 的读法一致。"""
    v = max(-32768, min(32767, int(round(v))))
    return [(v >> 8) & 0xFF, v & 0xFF]


def build_frame(cmd, data=None):
    f = bytearray(TX_LEN)
    f[0] = FRAME_HEADER
    f[1] = cmd
    f[2] = 0
    for i in range(6):
        f[3 + i] = data[i] if data and i < len(data) else 0
    f[9] = bcc(f, 9)
    f[10] = FRAME_TAIL
    return bytes(f)


def build_vel_frame(vx, vy, vz):
    b = _s16_bytes(vx * 1000) + _s16_bytes(vy * 1000) + _s16_bytes(vz * 1000)
    return build_frame(CMD_VEL, b)


def build_servo_frame(pulse_us):
    pulse_us = max(SERVO_MIN_US, min(SERVO_MAX_US, int(pulse_us)))
    return build_frame(CMD_SERVO, _s16_bytes(pulse_us))


def build_selftest_frame():
    return build_frame(CMD_SELFTEST)


def build_emergency_frame():
    return build_frame(CMD_EMERGENCY)


def build_release_frame():
    return build_frame(CMD_RELEASE)


class UplinkParser:
    """24 字节底盘帧的滚动缓冲解析器。

    别用"上一字节是帧尾"那种判据。固件每帧底盘数据后面都紧跟一帧 19 字节超声波
    (app/telemetry.h，头 0xFA 尾 0xFC)，所以每个 0x7B 前面都是 0xFC，按那条判据
    count 根本起不来,一帧都解不出来。当初照着老代码抄就是这个坑。

    做法是扫 0x7B 取 24 字节，帧尾和 BCC 都对上才认，不对就丢 1 字节重扫。
    假阳性大概 1/65536，失步了也能自己回来。
    """

    def __init__(self):
        self._buf = bytearray()
        self.dropped = 0     # 丢了几个假帧头，观测串口质量用

    def feed(self, chunk):
        self._buf += chunk
        out = []
        while True:
            i = self._buf.find(FRAME_HEADER)
            if i < 0:
                self._buf.clear()
                break
            if i:
                del self._buf[:i]
            if len(self._buf) < RX_LEN:
                break
            if self._buf[RX_LEN - 1] == FRAME_TAIL and bcc(self._buf, RX_LEN - 2) == self._buf[RX_LEN - 2]:
                out.append(bytes(self._buf[:RX_LEN]))
                del self._buf[:RX_LEN]
            else:
                self.dropped += 1
                del self._buf[:1]
        return out


def parse_uplink(frame):
    """24 字节底盘帧 → dict。布局见 mower-chassis/app/telemetry.c。

    [1] 是 robot_control.FlagStop —— 源码里那句 //set aside //预留位 是错的，
    app/telemetry.c 明确给它赋了值（固件里就是 flag_stop）。
    """
    return {
        'flag_stop': frame[1],
        'vx': _i16(frame, 2) / 1000.0,
        'vy': _i16(frame, 4) / 1000.0,
        'vz': _i16(frame, 6) / 1000.0,
        'accel': tuple(_i16(frame, 8 + 2 * i) / ACCEL_RATIO for i in range(3)),
        'gyro': tuple(_i16(frame, 14 + 2 * i) * GYROSCOPE_RATIO for i in range(3)),
        'voltage': _u16(frame, 20) / 1000.0,
    }


# ---- 19 字节超声波帧 ----
#
# 固件每拍在 24 字节底盘帧后面紧跟一帧：头 0xFA、8 路 int16 大端（mm）、
# BCC、尾 0xFC。0 表示这一路没有读数（没装模块，或者 CAN 上没收到）。

DIST_HEADER = 0xFA
DIST_TAIL = 0xFC
DIST_SLOTS = 8
DIST_BCC_LEN = 17


class DistanceParser:
    """19 字节超声波帧的滚动缓冲解析器。

    和 UplinkParser 同一个套路，只是头尾不同。两帧在字节流里交错出现，
    各扫各的，谁也不影响谁。
    """

    def __init__(self):
        self._buf = bytearray()
        self.dropped = 0

    def feed(self, chunk):
        self._buf += chunk
        out = []
        while True:
            i = self._buf.find(DIST_HEADER)
            if i < 0:
                self._buf.clear()
                break
            if i:
                del self._buf[:i]
            if len(self._buf) < DIST_LEN:
                break
            if (self._buf[DIST_LEN - 1] == DIST_TAIL
                    and bcc(self._buf, DIST_BCC_LEN) == self._buf[DIST_LEN - 2]):
                out.append(bytes(self._buf[:DIST_LEN]))
                del self._buf[:DIST_LEN]
            else:
                self.dropped += 1
                del self._buf[:1]
        return out


def parse_distance(frame):
    """19 字节超声波帧 → 8 个距离，单位 mm。0 = 这一路没有读数。"""
    return [_i16(frame, 1 + 2 * i) for i in range(DIST_SLOTS)]



class Mahony:
    """Mahony 互补滤波：用加速度计修陀螺零偏，出四元数。公开算法。

    kp=1.0 / ki=0 是这类滤波的常用起点。更新的步长用**实测 dt** 而不是写死
    1/20 —— 帧率一偏，写死的那个会发散，实测的只是滤波强弱变化。
    """

    def __init__(self, kp=1.0, ki=0.0):
        self.kp = kp
        self.ki = ki
        self.q = [1.0, 0.0, 0.0, 0.0]
        self._fb = [0.0, 0.0, 0.0]

    def update(self, gx, gy, gz, ax, ay, az, dt):
        q0, q1, q2, q3 = self.q
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            n = math.sqrt(ax * ax + ay * ay + az * az)
            ax, ay, az = ax / n, ay / n, az / n
            halfvx = q1 * q3 - q0 * q2
            halfvy = q0 * q1 + q2 * q3
            halfvz = q0 * q0 - 0.5 + q3 * q3
            ex = ay * halfvz - az * halfvy
            ey = az * halfvx - ax * halfvz
            ez = ax * halfvy - ay * halfvx
            if self.ki > 0.0:
                self._fb = [self._fb[i] + self.ki * e * dt for i, e in enumerate((ex, ey, ez))]
            else:
                self._fb = [0.0, 0.0, 0.0]
            gx += self.kp * ex + self._fb[0]
            gy += self.kp * ey + self._fb[1]
            gz += self.kp * ez + self._fb[2]
        h = 0.5 * dt
        gx *= h
        gy *= h
        gz *= h
        self.q = [
            q0 + (-q1 * gx - q2 * gy - q3 * gz),
            q1 + (q0 * gx + q2 * gz - q3 * gy),
            q2 + (q0 * gy - q1 * gz + q3 * gx),
            q3 + (q0 * gz + q1 * gy - q2 * gx),
        ]
        n = math.sqrt(sum(c * c for c in self.q))
        self.q = [c / n for c in self.q]
        return self.q
