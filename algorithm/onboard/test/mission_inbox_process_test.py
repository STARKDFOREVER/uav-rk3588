#!/usr/bin/env python3
# mission_inbox_process_test.py —— 真实进程级 KML 延迟投递测试（CTest 集成自测）
# 测试目的：在无飞机的情况下启动真实可执行文件，验证：
#   1) 缺失 KML 时进程保持 WAIT_KML，且不打开、不写入 UART；半成品 KML 直接终态失败；
#   2) 合法 KML 投递后只装载一次任务（[KML READY] 仅出现一次），随后才走到串口打开阶段；
#   3) 等待期间收到 SIGTERM 可干净退出（退出码 0）。
"""Exercise delayed KML arrival in the real executable, without any aircraft."""
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time

# 三个位置参数：被测可执行文件、配置文件、样例 KML。
binary, config, sample = sys.argv[1:]
# 实机控制路径必需参数；雷达参数只为通过参数校验，进程会先因串口失败退出。
CONTROL_ARGS = ['--enable-control', '--livox-config', 'unused.json', '--lidar-ip', '192.168.1.173']


def wait_for(log, marker, process, timeout=5):
    # 轮询日志直到 marker 出现：先查文本再查进程存活，避免进程已崩溃却盲等超时。
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        log.seek(0)
        # 每轮从头重读临时文件：子进程持续追加，这样无需维护读取偏移。
        text = log.read().decode()
        if marker in text:
            return text
        # 进程提前退出说明已经失败：立即抛出当前日志，比等到超时更易定位。
        if process.poll() is not None:
            raise AssertionError(text)
        time.sleep(.02)
    # 超时视为失败（默认 5s 仅为防止 CI 挂死，不是性能断言）。
    raise AssertionError('Timed out waiting for ' + marker)


with tempfile.TemporaryDirectory() as temp:
    # 临时目录隔离收件箱与 .tmp，测试结束自动清理。
    inbox = Path(temp, 'mission.kml')
    delivery = Path(temp, 'mission.kml.tmp')
    # 程序错误统一写到该目录；控制台不再出现 [FATAL]。
    error_dir = Path(temp, 'errors')
    error_log_file = error_dir / 'onboard_error.log'

    def error_text():
        return error_log_file.read_text() if error_log_file.exists() else ''
    # 伪终端模拟 FCU 串口：slave 路径交给进程，master 用于检测是否真的写了数据。
    master, slave = pty.openpty()
    # stdout 写入临时文件边运行边轮询；错误由程序自身写入 --error-dir 下的日志。
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen([binary, '--config', config, '--kml', str(inbox),
                                    '--serial', os.ttyname(slave), '--error-dir', str(error_dir)]
                                   + CONTROL_ARGS,
                                   stdout=log, stderr=log)
        try:
            # 启动后应先进入 WAIT_KML，此时 KML 尚不存在；稍等确认进程存活。
            wait_for(log, '[STATE] WAIT_KML', process)
            time.sleep(.2)
            assert process.poll() is None
            # select 零超时轮询主端：无数据即证明进程没有打开/写入 UART。
            assert not select.select([master], [], [], 0)[0], 'UART wrote before KML arrival'
            # 半成品 XML：解析失败是终态，原因只写入错误日志，不碰 UART。
            inbox.write_text('<kml>incomplete')
            assert process.wait(timeout=5) == 2
            assert not select.select([master], [], [], 0)[0], 'Invalid KML activated the UART'
            log.seek(0)
            failed = log.read().decode()
            assert '[FATAL]' not in failed, failed
            assert 'Expected valid KML XML' in error_text(), error_text()
            print('Partial KML fails terminally before any device is opened')
        finally:
            # 兜底清理：断言失败时也必须确保子进程不残留。
            if process.poll() is None:
                process.kill()
                process.wait()

    # 关闭 PTY 两端，避免 fd 泄漏影响后续用例。
    os.close(master)
    os.close(slave)

    # 第二段：合法任务投递后只装载一次，随后进入串口打开阶段并失败。
    inbox.unlink()
    # 清掉上一段的错误日志，确保本段断言的是本次运行的 FATAL。
    error_log_file.unlink(missing_ok=True)
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen([binary, '--config', config, '--kml', str(inbox),
                                    '--serial', '/nonexistent-fcu', '--error-dir', str(error_dir)]
                                   + CONTROL_ARGS,
                                   stdout=log, stderr=log)
        try:
            wait_for(log, '[STATE] WAIT_KML', process)
            # 先写 .tmp 再 replace 到正式路径，复刻生产端原子投递。
            delivery.write_bytes(Path(sample).read_bytes())
            delivery.replace(inbox)
            # 串口打开失败应返回 2；控制台只保留 [KML READY]，[FATAL] 写入错误日志。
            assert process.wait(timeout=5) == 2
            log.seek(0)
            output = log.read().decode()
            assert '[KML READY]' in output and '[FATAL]' not in output, output
            assert output.count('[KML READY]') == 1, output
            assert '[FATAL]' in error_text(), error_text()
            print('Delayed control start latches one task, then reaches the serial open stage')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()

    # 第三段：等待期间收到 SIGTERM 应干净退出且不打开设备。
    inbox.unlink()
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen([binary, '--config', config, '--kml', str(inbox),
                                    '--serial', '/nonexistent-fcu',
                                    '--error-dir', str(error_dir)] + CONTROL_ARGS,
                                   stdout=log, stderr=log)
        try:
            wait_for(log, '[STATE] WAIT_KML', process)
            # 发送 SIGTERM 并期望退出码 0
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=2) == 0
            print('SIGTERM cleanly cancels KML waiting without opening a device')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
