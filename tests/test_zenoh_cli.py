#!/usr/bin/env python3
"""Check CLI validation and SIGINT/SIGTERM cleanup of the real C++ executable."""
import signal
import socket
import subprocess
import sys

binary = sys.argv[1]
help_result = subprocess.run([binary, "--help"], capture_output=True, text=True, timeout=5)
assert help_result.returncode == 0
assert "quaternion XYZW" in help_result.stdout
for args in [
    ["--iterations", "0"], ["--iterations", "-1"], ["--iterations", "1x"],
    ["--max-points", "0"], ["--max-points", "999999999999999999999999"],
    ["--rotation-tolerance", "nan"], ["--translation-tolerance", "-1"],
    ["--rotation-tolerance", "0.1junk"], ["--time-limit-ms", "-1"],
    ["--time-limit-ms", "18446744073709551615"], ["--unknown", "x"],
    ["--iterations"], ["--iterations", "1", "--iterations", "2"],
    ["--output-key", "invalid/*"], ["--input-key", ""],
]:
    result = subprocess.run([binary, *args], capture_output=True, text=True, timeout=5)
    assert result.returncode == 1, (args, result.stderr)
    assert "icp_zenoh:" in result.stderr

for stop_signal in [signal.SIGINT, signal.SIGTERM]:
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    process = subprocess.Popen(
        [binary, "--listen", f"tcp/127.0.0.1:{port}"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        assert process.stderr.readline().startswith("icp_zenoh:"), "node did not start"
        process.send_signal(stop_signal)
        assert process.wait(timeout=10) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
print("CLI validation, SIGINT, and SIGTERM passed")
