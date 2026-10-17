"""Exercise the actual gimbal_test executable with unsolicited RX and no TX replies."""
import csv
import math
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty


def run_case(executable, directory, name, receive, read_only=False, startup=False, export_csv=True,
             uart=False, pose=False):
    master, slave = os.openpty()
    tty.setraw(slave)
    config = directory / f"{name}.yaml"
    output = directory / f"{name}.csv"
    config_text = f"use_can: {'false' if uart else 'true'}\ncom_port: {os.ttyname(slave)}\nimu_timeout_ms: 150\n"
    if uart:
        config_text += (f"serial_imu:\n  port: {os.ttyname(slave)}\n  protocol: uart44\n"
                        "  angle_unit: degrees\n  velocity_unit: degrees\n  tx:\n"
                        "    protocol: sp44\n    angle_unit: degrees\n")
    config.write_text(config_text)
    args = [executable, str(config)]
    if startup or pose:
        args += ["--read-only"]
    else:
        args += ["--latency", "--duration=0.5", "--interval-ms=10"]
        if export_csv:
            args += [f"--csv={output}"]
        if read_only:
            args += ["--read-only"]
    # Continuous telemetry has no relationship to the commands read from the PTY.
    packet = struct.pack("<2sB9fHHB", b"SP", 1, 1, 0, 0, 0,
                         0.25, -0.5, -0.125, 0.75, 22, 11, 0, 0x0D)
    if uart:
        packet = b"SP\x00" + bytes(16) + struct.pack("<5fHHB", -15, 25, 75, 30, -60, 0, 0, 0x0D)
    command_size = 44
    expected = b"SP\x00" + bytes(40) + b"\x0d"
    valid_receive = receive is True
    if receive == "malformed":
        packet = b"SP" + bytes(24) + b"\x0d"  # 当前旧固件的27字节格式。
    received = bytearray()
    stopped = threading.Event()
    pump_errors = []

    def pump():
        next_frame = time.monotonic()
        try:
            while not stopped.is_set():
                now = time.monotonic()
                if receive and now >= next_frame:
                    os.write(master, packet[:7])
                    os.write(master, packet[7:])
                    next_frame = now + (0.001 if receive == "malformed" else 0.005)
                if select.select([master], [], [], 0.001)[0]:
                    received.extend(os.read(master, 4096))
            while select.select([master], [], [], 0)[0]:
                received.extend(os.read(master, 4096))
        except Exception as error:
            pump_errors.append(error)

    process = None
    worker = threading.Thread(target=pump)
    try:
        process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        worker.start()
        if pose:
            time.sleep(0.3)
            process.send_signal(signal.SIGINT)
        log, _ = process.communicate(timeout=5)
        stopped.set()
        worker.join(timeout=1)
        assert not worker.is_alive() and not pump_errors, pump_errors
        expected_code = 1 if startup or not valid_receive else 0
        assert process.returncode == expected_code, log
        if pose:
            assert "yaw 75.00, pitch -15.00, roll 25.00 degree" in log, log
            assert not received
            print("gimbal_test UART read-only: actual executable decodes Euler44 pose correctly")
            return
        if startup:
            assert "Timed out waiting for a valid 44-byte SP frame" in log, log
            assert not received
            print("gimbal_test: startup timeout returns an error without hanging")
            return
        if not export_csv:
            assert "[Gimbal latency final]" in log, log
            assert len(received) >= command_size * 10 and len(received) % command_size == 0, log
            assert all(received[i:i + command_size] == expected
                       for i in range(0, len(received), command_size)), log
            print("gimbal_test: latency mode works without optional CSV")
            return
        with output.open() as stream:
            rows = list(csv.DictReader(stream))
        assert rows, log
        stats = rows[-1]
        assert all(math.isfinite(float(value)) for value in stats.values()), stats
        assert 0.5 <= float(stats["elapsed_s"]) < 2, stats
        assert int(stats["tx_errors"]) == 0, stats
        assert len(received) % command_size == 0, received.hex()
        commands = len(received) // command_size
        assert int(stats["tx_frames"]) == commands, (stats, commands)
        assert int(stats["tx_attempts"]) == commands, stats
        assert all(received[i:i + command_size] == expected
                   for i in range(0, len(received), command_size)), received.hex()
        if read_only:
            assert commands == 0, commands
        else:
            assert commands >= 10, stats
            assert 0 <= float(stats["tx_write_mean_ms"]) <= float(stats["tx_write_max_ms"]), stats
        if valid_receive:
            assert int(stats["rx_frames"]) >= 10, stats
            assert int(stats["rx_bytes"]) >= int(stats["rx_frames"]) * 44, stats
            assert 0 < float(stats["rx_interval_mean_ms"]) <= float(stats["rx_interval_max_ms"]), stats
            assert float(stats["rx_age_ms"]) >= 0, stats
        else:
            assert int(stats["rx_frames"]) == 0 and float(stats["rx_age_ms"]) == -1, stats
            assert "No valid 44-byte SP RX frames" in log, log
            if receive == "malformed":
                assert int(stats["rx_bytes"]) > 5000 and int(stats["rx_bad_tails"]) > 0, stats
                assert "Reconnecting" not in log, log
        print(f"gimbal_test {name}: RX {stats['rx_frames']}, TX {commands}; no reply needed")
    finally:
        stopped.set()
        if process is not None and process.poll() is None:
            process.kill()
            process.wait()
        if worker.ident is not None:
            worker.join(timeout=1)
        os.close(master)
        os.close(slave)


def main():
    executable = str(Path(sys.argv[1]).resolve())
    with tempfile.TemporaryDirectory(prefix="gimbal_latency_") as temp:
        directory = Path(temp)
        run_case(executable, directory, "full_duplex_no_reply", True)
        run_case(executable, directory, "default_without_csv", True, export_csv=False)
        run_case(executable, directory, "read_only", True, read_only=True)
        run_case(executable, directory, "tx_without_rx", False)
        run_case(executable, directory, "tx_with_old_27_byte_rx", "malformed")
        run_case(executable, directory, "startup_without_rx", False, startup=True)
        run_case(executable, directory, "uart44_bidirectional_no_reply", True, uart=True)
        run_case(executable, directory, "uart44_tx_without_rx", False, uart=True)
        run_case(executable, directory, "uart44_read_pose", True, uart=True, pose=True)


if __name__ == "__main__":
    main()
