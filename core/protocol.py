"""11 字节发帧（上位机 → STM32）。

这里只组帧、不解析 —— Ros2Client 把帧再解回语义话题，所以这份帧不会真的过串口。
真正过串口的是桥 `bridge/mower_bridge/protocol.py`，**那份是权威实现**（含收帧解析），
改协议时两边要一起改。

字段对照固件 mower-chassis/app/protocol.c 的分支，没有的别加。
"""


class Protocol:
    FRAME_HEADER = 0x7B
    FRAME_TAIL   = 0x7D

    CMD_SERVO     = 4
    CMD_SELFTEST  = 5
    CMD_EMERGENCY = 7   # 急停：固件清三轴速度、打断自检；锁存由桥维护
    CMD_RELEASE   = 8   # 解除：上位机用它清桥的锁存，帧发出仅留痕

    SERVO_MIN_US = 500
    SERVO_MAX_US = 2500

    def __init__(self):
        self.frame_len = 11

    def _frame(self, cmd, data=None):
        frame = bytearray(self.frame_len)
        frame[0] = self.FRAME_HEADER
        frame[1] = cmd
        frame[2] = 0
        for i in range(3, 9):
            frame[i] = data[i - 3] if data and i - 3 < len(data) else 0
        frame[9] = 0
        for i in range(9):
            frame[9] ^= frame[i]
        frame[10] = self.FRAME_TAIL
        return bytes(frame)

    @staticmethod
    def _s16(v):
        """16 位有符号 → 大端两字节。负数靠 & 0xFF 落成补码；
        原来的 `vx_mm >> 8` 对负数得负数，塞进 bytearray 会直接抛 ValueError。"""
        v = max(-32768, min(32767, int(round(v))))
        return [(v >> 8) & 0xFF, v & 0xFF]

    def build_servo_frame(self, pulse_us):
        pulse_us = max(self.SERVO_MIN_US, min(self.SERVO_MAX_US, int(pulse_us)))
        return self._frame(self.CMD_SERVO, self._s16(pulse_us))

    def build_selftest_frame(self):
        return self._frame(self.CMD_SELFTEST)

    def build_emergency_frame(self):
        return self._frame(self.CMD_EMERGENCY)

    def build_release_frame(self):
        return self._frame(self.CMD_RELEASE)


if __name__ == '__main__':
    p = Protocol()
    assert p.build_servo_frame(100)[3:5] == bytes([0x01, 0xF4])     # 夹到 500
    assert p.build_servo_frame(9999)[3:5] == bytes([0x09, 0xC4])    # 夹到 2500
    assert p.build_emergency_frame()[1] == 7
    assert p.build_release_frame()[1] == 8
    for f in (p.build_servo_frame(1500),
              p.build_selftest_frame(), p.build_emergency_frame(),
              p.build_release_frame()):
        assert len(f) == 11 and f[0] == 0x7B and f[10] == 0x7D
        chk = 0
        for b in f[:9]:
            chk ^= b
        assert f[9] == chk
    print('protocol.py 自检通过')
