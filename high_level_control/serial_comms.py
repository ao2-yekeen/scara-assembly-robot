# serial_comms.py
# ─────────────────────────────────────────────────────────────────────────────
# Serial communication layer — real hardware and dry-run simulation.
# Both classes share the same interface: send(cmd) → response_str.

import time
from display import YELLOW, RESET


class DryRunSerial:
    """
    Simulates Arduino serial responses without hardware.
    Use for testing the full Python pipeline offline.
    """

    # Maps command prefix → expected response
    _RESPONSES = {
        "MOVE:":    "OK:MOVE",
        "GRIP":     "OK:GRIP",
        "RELEASE":  "OK:RELEASE",
        "HOME":     "OK:HOME",
        "REHOME_Z": "OK:REHOME_Z",
    }

    def send(self, cmd: str) -> str:
        time.sleep(0.02)  # simulate serial latency
        response = self._resolve(cmd)
        print(f"    {YELLOW}[DRY]{RESET} {cmd:<60} → {response}")
        return response

    def _resolve(self, cmd: str) -> str:
        for prefix, response in self._RESPONSES.items():
            if cmd.startswith(prefix):
                return response
        return "NACK:UNKNOWN"


class RealSerial:
    """
    Wraps the comms module for real Arduino communication.
    Defers import of pyserial until actually needed.
    """

    def __init__(self, port: str, baud: int, timeout: float = 10.0):
        from comms import connect
        self._ser = connect(port, baud, wait_ready=True)
        self._timeout = timeout

    def send(self, cmd: str, timeout: float | None = None) -> str:
        from comms import send_command
        return send_command(self._ser, cmd, timeout=timeout or self._timeout)

    def close(self) -> None:
        from comms import disconnect
        disconnect(self._ser)


def make_serial(port: str, baud: int, dry_run: bool):
    """Factory — returns DryRunSerial or RealSerial based on dry_run flag."""
    if dry_run:
        return DryRunSerial()
    return RealSerial(port, baud)
