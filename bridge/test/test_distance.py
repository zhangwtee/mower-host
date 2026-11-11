"""距离帧：19 字节超声波帧的解析和重同步。"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mower_bridge.protocol import (  # noqa: E402
    DIST_HEADER, DIST_TAIL, DIST_LEN, DIST_SLOTS,
    DistanceParser, bcc, parse_distance,
)


def dist_frame(*vals):
    """照 mower-chassis/app/telemetry.c 组一个 19 字节超声波帧。

    固件只填前 6 路（A~F 来自 CAN 的 0x101/0x102），后两路恒 0。
    """
    b = bytearray(DIST_LEN)
    b[0] = DIST_HEADER
    v = list(vals) + [0] * (DIST_SLOTS - len(vals))
    for i in range(DIST_SLOTS):
        b[1 + 2 * i] = (v[i] >> 8) & 0xFF
        b[2 + 2 * i] = v[i] & 0xFF
    b[DIST_LEN - 2] = bcc(b, DIST_LEN - 2)
    b[DIST_LEN - 1] = DIST_TAIL
    return bytes(b)


def test_distance_frame_shape():
    """头尾和 BCC 的位置要和固件一致。"""
    f = dist_frame(123, 456, 789, 1000, 1500, 2000)
    assert f[0] == 0xFA
    assert f[DIST_LEN - 1] == 0xFC
    assert f[DIST_LEN - 2] == bcc(f, DIST_LEN - 2)


def test_distance_values_parse():
    """8 路都要按大端 int16 解出来。"""
    v = parse_distance(dist_frame(123, 456, 789, 1000, 1500, 2000))
    assert v == [123, 456, 789, 1000, 1500, 2000, 0, 0], v


def test_distance_negative():
    """距离不该是负的，但解析不能崩——固件那边越界值就是负的。"""
    v = parse_distance(dist_frame(-1, -32768, 32767))
    assert v[0] == -1 and v[1] == -32768 and v[2] == 32767, v


def test_distance_parser_mixed_stream():
    """底盘帧和超声波帧交错，两个解析器各扫各的，互不干扰。"""
    chassis = bytearray(24)
    chassis[0] = 0x7B
    chassis[23] = 0x7D
    chassis[22] = bcc(chassis, 22)

    dp = DistanceParser()
    stream = (bytes(chassis) + dist_frame(10, 20, 30)) * 3
    got = []
    for i in range(0, len(stream), 5):      # 故意切碎
        got += dp.feed(stream[i:i + 5])
    assert len(got) == 3, got
    assert parse_distance(got[0])[0] == 10


def test_distance_parser_rejects_bad_bcc():
    """BCC 坏了要丢掉并继续扫，不能卡死。"""
    bad = bytearray(dist_frame(10, 20))
    bad[DIST_LEN - 2] ^= 0xFF
    dp = DistanceParser()
    got = dp.feed(bytes(bad) + dist_frame(77, 88))
    assert len(got) == 1, got
    assert parse_distance(got[0])[0] == 77
    assert dp.dropped >= 1


def test_distance_parser_garbage():
    """纯垃圾不能产出帧。"""
    dp = DistanceParser()
    junk = bytes((i * 7 + 1) & 0xFF for i in range(200))
    junk = junk.replace(b'\xfa', b'\x00')
    assert dp.feed(junk) == []

if __name__ == '__main__':
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print('ok   %s' % name)
            except AssertionError as e:
                fails += 1
                print('FAIL %s: %s' % (name, e))
    print('\n%s' % ('全过' if not fails else '%d 个失败' % fails))
    sys.exit(1 if fails else 0)