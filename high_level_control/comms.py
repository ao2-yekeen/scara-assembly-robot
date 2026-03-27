# serial_comms.py
# Low-level serial layer for SCARA firmware communication.
# Used by both pc_software/main.py and tests/test_serial.py

import serial
import time

# ─── PROTOCOL CONSTANTS ──────────────────────────────────────────────────────
BAUD         = 115200
TIMEOUT_S    = 10.0   # seconds to wait for any response
READY_WAIT_S = 15.0   # seconds to wait for READY on connect

OK_RESPONSES = {
    "HOME":     "OK:HOME",
    "MOVE":     "OK:MOVE",
    "GRIP":     "OK:GRIP",
    "RELEASE":  "OK:RELEASE",
    "REHOME_Z": "OK:REHOME_Z",
    "SPEED":    "OK:SPEED",
    "ACCEL":    "OK:ACCEL",
}


def connect(port, baud=BAUD, wait_ready=True):
    """
    Open serial port and optionally wait for READY from firmware.
    Returns open serial.Serial object.
    Raises RuntimeError if READY not received within READY_WAIT_S.
    """
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)  # allow Arduino to reset after DTR toggle
    ser.reset_input_buffer()

    # if wait_ready:
    #     deadline = time.time() + READY_WAIT_S
    #     while time.time() < deadline:
    #         line = ser.readline().decode("utf-8", errors="ignore").strip()
    #         if line:
    #             print(f"  << {line}")
    #         if line == "READY":
    #             print("[connect] READY received")
    #             return ser
    #     raise RuntimeError(f"[connect] READY not received within {READY_WAIT_S}s")

    return ser


def send_command(ser, cmd, timeout=TIMEOUT_S):
    """
    Send one command string (no newline needed) and block until response.

    Returns the response string.
    Raises RuntimeError on timeout.
    Prints ERR/NACK lines as warnings but still returns them — caller decides.
    """
    full_cmd = cmd.strip() + "\n"
    ser.write(full_cmd.encode("utf-8"))
    ser.flush()
    print(f"  >> {cmd}")

    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        print(f"  << {line}")
        # Accept any non-empty line as the response
        # (firmware may emit intermediate lines like HOMED Jx during HOME)
        if (line.startswith("OK:")
                or line.startswith("NACK:")
                or line.startswith("ERR:")
                or line.startswith("STATUS:")):
            return line

    raise RuntimeError(f"[send_command] Timeout waiting for response to: {cmd}")


def disconnect(ser):
    if ser and ser.is_open:
        ser.close()
        print("[disconnect] Port closed")
