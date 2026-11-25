#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# mission_inbox_process_test.py —— 真实进程级 KML 延迟投递测试（CTest 集成自测）
# 测试目的：在无飞机的情况下启动真实可执行文件，验证：
#   1) 缺失 KML 时进程保持 WAIT_KML，且不打开、不写入 UART；半成品 KML 直接终态失败；
#   2) 合法 KML 投递后只装载一次任务（[KML READY] 仅出现一次），
#      后续非法替换不影响已接受任务；
#   3) --enable-control 时任务交付后才走到串口打开阶段（打开失败退出码 2）；
#   4) 等待期间收到 SIGTERM 可干净退出（退出码 0）；
#   5) --prepare-only 严格模式缺文件时退出码 2，原因写入错误日志而不打印。
# 运行参数：argv[1]=被测可执行文件，argv[2]=配置路径，argv[3]=样例 KML 路径。
# 通过判据：所有 assert 通过；任一失败抛 AssertionError → 非 0 退出。
# 说明：用 pty.openpty() 伪造串口，select 零超时轮询主端可读性来证明“未写 UART”；
#   程序错误统一写入 --error-dir 指定的 onboard_error.log，控制台只保留非错误状态。
#   wait_for 的 5s 超时仅用于防止 CI 挂死，不是性能断言。
# ---------------------------------------------------------------------------
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
        # duration=1.5 让进程自行结束，便于验证“只装载一次任务”后的稳定行为。
        process = subprocess.Popen([binary, '--config', config, '--kml', str(inbox),
                                    '--serial', os.ttyname(slave), '--duration', '1.5',
                                    '--error-dir', str(error_dir)],
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
            assert 'Expected valid KML XML' not in failed and '[FATAL]' not in failed, failed
            assert 'Expected valid KML XML' in error_text(), error_text()
            print('Partial KML fails terminally before any device is opened')
        finally:
            # 兜底清理：断言失败时也必须确保子进程不残留。
            if process.poll() is None:
                process.kill()
                process.wait()

    # 第二段：合法任务延迟投递；接受后替换内容不得改写已锁存任务。
    inbox.unlink()
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen([binary, '--config', config, '--kml', str(inbox),
                                    '--serial', os.ttyname(slave), '--duration', '1.5',
                                    '--error-dir', str(error_dir)],
                                   stdout=log, stderr=log)
        try:
            wait_for(log, '[STATE] WAIT_KML', process)
            # 先写 .tmp 再 replace 到正式路径，复刻生产端原子投递。
            delivery.write_bytes(Path(sample).read_bytes())
            delivery.replace(inbox)
            wait_for(log, '[KML READY]', process)
            # 接受后再覆盖为非法内容：已锁存任务不应被改写，也不应重复装载。
            inbox.write_text('invalid replacement after acceptance')
            # 正常结束退出码 0；随后断言 [KML READY] 只出现一次。
            assert process.wait(timeout=6) == 0
            log.seek(0)
            output = log.read().decode()
            # 只允许一次装载，防止目录轮询导致重复任务。
            assert output.count('[KML READY]') == 1, output
            # 未启用控制时只能进入观测模式。
            assert '[OBSERVE]' in output and '[CONTROL]' not in output, output
            print('Real process starts on valid delivery and latches one task')
        finally:
            # 兜底清理：断言失败时也必须确保子进程不残留。
            if process.poll() is None:
                process.kill()
                process.wait()
            # 关闭 PTY 两端，避免 fd 泄漏影响后续用例。
            os.close(master)
            os.close(slave)

    # 第三段：--enable-control，但串口路径不存在，必须在任务接受后才失败。
    inbox.unlink()
    with tempfile.TemporaryFile() as log:
        # livox 相关参数仅为满足参数解析，不启动真实雷达（会因串口失败先退出）。
        process = subprocess.Popen([binary, '--config', config, '--kml', str(inbox),
                                    '--serial', '/nonexistent-fcu', '--enable-control',
                                    '--livox-config', 'unused.json', '--lidar-ip', '192.168.1.173',
                                    '--error-dir', str(error_dir)],
                                   stdout=log, stderr=log)
        try:
            wait_for(log, '[STATE] WAIT_KML', process)
            # 仍先等待缺失 KML；投递合法任务后才进入串口打开阶段。
            delivery.write_bytes(Path(sample).read_bytes())
            delivery.replace(inbox)
            # 串口打开失败应返回 2；控制台只保留 [KML READY]，[FATAL] 写入错误日志。
            assert process.wait(timeout=5) == 2
            log.seek(0)
            output = log.read().decode()
            assert '[KML READY]' in output and '[FATAL]' not in output, output
            assert '[FATAL]' in error_text(), error_text()
            print('Delayed control start reaches the serial open stage after task acceptance')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()

    # 第四段：等待期间收到 SIGTERM 应干净退出且不打开设备。
    inbox.unlink()
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen([binary, '--config', config, '--kml', str(inbox),
                                    '--serial', '/nonexistent-fcu',
                                    '--error-dir', str(error_dir)], stdout=log, stderr=log)
        try:
            wait_for(log, '[STATE] WAIT_KML', process)
            # 发送 SIGTERM 并期望退出码 0（优雅取消，而非被杀）。
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=2) == 0
            print('SIGTERM cleanly cancels KML waiting without opening a device')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
    # 最后：--prepare-only 严格模式下缺文件立即返回 2，原因只写入错误日志。
    strict = subprocess.run([binary, '--config', config, '--kml', str(inbox), '--prepare-only',
                             '--error-dir', str(error_dir)],
                            capture_output=True, text=True, timeout=3)
    assert strict.returncode == 2, strict
    assert 'Cannot open local KML' in error_text() and 'Cannot open local KML' not in strict.stderr, strict
