"""
DH Robotics AG Gripper - Modbus-RTU Test Script v2
===================================================

Improvements over the basic version:
- CRC validation on responses
- Motion completion detected via status register (no blind sleep)
- Position accuracy scan
- Force comparison test
- Grasp / drop detection

Usage:
    python dh_gripper_test_v2.py              # run default suite (basic + position)
    python dh_gripper_test_v2.py basic        # basic open/close
    python dh_gripper_test_v2.py position     # position accuracy scan
    python dh_gripper_test_v2.py force        # force comparison (needs object)
    python dh_gripper_test_v2.py grasp        # grasp + drop detection (needs object)
"""

import serial
import struct
import time
import sys

# ------------- Config -------------
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
DEVICE_ID = 0x01
TIMEOUT = 0.5

# ------------- Register addresses (from manual section 2.3.2) -------------
REG_INIT              = 0x0100   # Init: write 0x01 for normal, 0xA5 for full calibration
REG_FORCE             = 0x0101   # Force 20-100 (%)
REG_POSITION          = 0x0103   # Target position 0-1000 (per-mille)
REG_INIT_STATE        = 0x0200   # Init state (read-only): 0=not initialized, 1=initialized
REG_GRIP_STATE        = 0x0201   # Gripper state (read-only):
                                 #   0=moving, 1=arrived (no object),
                                 #   2=object caught, 3=object dropped
REG_CURRENT_POSITION  = 0x0202   # Current actual position (read-only)

GRIP_STATE_NAMES = {
    0: "moving",
    1: "arrived (no object)",
    2: "object caught",
    3: "object dropped",
}


# ------------- Modbus-RTU low level -------------
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _send_frame(ser, frame: bytes, expected_len: int) -> bytes:
    """Send a frame and read the response, validating CRC."""
    ser.reset_input_buffer()
    ser.write(frame)
    resp = ser.read(expected_len)
    if len(resp) < 5:
        raise IOError(f"Response too short: {resp.hex()}")
    body, recv_crc = resp[:-2], resp[-2:]
    calc = crc16(body)
    if struct.unpack('<H', recv_crc)[0] != calc:
        raise IOError(f"CRC check failed: {resp.hex()}")
    return resp


def write_reg(ser, reg: int, value: int) -> None:
    """Modbus function 0x06: write single register."""
    frame = struct.pack('>BBHH', DEVICE_ID, 0x06, reg, value)
    frame += struct.pack('<H', crc16(frame))
    _send_frame(ser, frame, expected_len=8)


def read_reg(ser, reg: int) -> int:
    """Modbus function 0x03: read single holding register, returns 16-bit value."""
    frame = struct.pack('>BBHH', DEVICE_ID, 0x03, reg, 1)
    frame += struct.pack('<H', crc16(frame))
    # Response: ID(1) + func(1) + bytecount(1) + data(2) + crc(2) = 7
    resp = _send_frame(ser, frame, expected_len=7)
    return struct.unpack('>H', resp[3:5])[0]


# ------------- High-level helpers -------------
def wait_initialized(ser, timeout_s: float = 10.0) -> None:
    """Poll init state register until it reports 1 (initialized)."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if read_reg(ser, REG_INIT_STATE) == 1:
            return
        time.sleep(0.1)
    raise TimeoutError("Initialization timed out")


def wait_motion_done(ser, timeout_s: float = 5.0) -> int:
    """
    Poll gripper state until it leaves 0 (moving).
    Returns the final state code (1/2/3).
    """
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        state = read_reg(ser, REG_GRIP_STATE)
        if state != 0:
            return state
        time.sleep(0.05)
    raise TimeoutError("Motion timed out")


def initialize(ser, full: bool = True) -> None:
    """Initialize the gripper. full=True performs full calibration (finds both limits)."""
    print(f"[init] Sending {'full' if full else 'normal'} initialization...")
    write_reg(ser, REG_INIT, 0xA5 if full else 0x01)
    t0 = time.time()
    wait_initialized(ser)
    print(f"[init] Done in {time.time()-t0:.2f}s")


def set_force(ser, force_pct: int) -> None:
    assert 20 <= force_pct <= 100, "force must be 20-100"
    write_reg(ser, REG_FORCE, force_pct)


def move_to(ser, position: int, wait: bool = True) -> int:
    """
    Move to target position (0 = closed, 1000 = fully open).
    If wait=True, block until motion ends and return the final state code.
    """
    assert 0 <= position <= 1000, "position must be 0-1000"
    write_reg(ser, REG_POSITION, position)
    if not wait:
        return -1
    return wait_motion_done(ser)


# ------------- Tests -------------
def test_basic(ser):
    """Basic open/close, using state feedback instead of fixed sleeps."""
    print("\n=== [basic] Basic open/close ===")
    set_force(ser, 50)

    print("-> Open to 1000 (fully open)")
    state = move_to(ser, 1000)
    pos = read_reg(ser, REG_CURRENT_POSITION)
    print(f"   final state: {GRIP_STATE_NAMES[state]}, actual position: {pos}")

    print("-> Close to 0")
    state = move_to(ser, 0)
    pos = read_reg(ser, REG_CURRENT_POSITION)
    print(f"   final state: {GRIP_STATE_NAMES[state]}, actual position: {pos}")


def test_position(ser):
    """Position accuracy scan: command several targets, compare actual vs target."""
    print("\n=== [position] Position accuracy scan ===")
    set_force(ser, 50)
    targets = [0, 250, 500, 750, 1000, 500, 100, 900]
    print(f"{'target':>7} {'actual':>7} {'error':>7}  state")
    print("-" * 45)
    for tgt in targets:
        state = move_to(ser, tgt)
        actual = read_reg(ser, REG_CURRENT_POSITION)
        err = actual - tgt
        print(f"{tgt:>7} {actual:>7} {err:>+7}  {GRIP_STATE_NAMES[state]}")


def test_force(ser):
    """
    Force comparison: close on a compressible object with different force levels.
    Place a soft object (sponge, foam) between the fingers before running.
    """
    print("\n=== [force] Force comparison ===")
    print("!! Place a compressible object (sponge/foam) between the fingers,")
    print("   then press Enter to continue.")
    input()

    print(f"{'force':>6} {'stopped at':>12} {'time(s)':>10}  state")
    print("-" * 55)
    for force in [20, 50, 100]:
        # Fully open first
        set_force(ser, 100)
        move_to(ser, 1000)

        # Set target force and close
        set_force(ser, force)
        t0 = time.time()
        state = move_to(ser, 0)
        elapsed = time.time() - t0
        pos = read_reg(ser, REG_CURRENT_POSITION)
        print(f"{force:>5}% {pos:>12} {elapsed:>10.2f}  {GRIP_STATE_NAMES[state]}")
        time.sleep(0.5)


def test_grasp(ser):
    """
    Grasp + drop detection: after catching an object, monitor state for 15s.
    Manually remove the object during the window and watch state go to 3 (dropped).
    """
    print("\n=== [grasp] Grasp and drop detection ===")
    print("!! Place an object between the fingers, then press Enter to continue.")
    input()

    set_force(ser, 50)
    move_to(ser, 1000)   # open first
    print("-> Closing to grasp...")
    state = move_to(ser, 0)
    pos = read_reg(ser, REG_CURRENT_POSITION)
    print(f"   initial state: {GRIP_STATE_NAMES[state]}, position: {pos}")

    if state != 2:
        print("   Gripper did not report 'object caught', aborting.")
        return

    print("-> Monitoring for 15s. You can pull the object out and watch for a drop.")
    t0 = time.time()
    last_state = state
    while time.time() - t0 < 15.0:
        s = read_reg(ser, REG_GRIP_STATE)
        if s != last_state:
            p = read_reg(ser, REG_CURRENT_POSITION)
            print(f"   [{time.time()-t0:5.1f}s] state change: "
                  f"{GRIP_STATE_NAMES[s]}, position: {p}")
            last_state = s
        time.sleep(0.1)
    print("Monitoring done.")


# ------------- main -------------
TESTS = {
    'basic':    test_basic,
    'position': test_position,
    'force':    test_force,
    'grasp':    test_grasp,
}


def main():
    args = sys.argv[1:]
    if args and args[0] in ('-h', '--help'):
        print(__doc__)
        return

    which = args[0] if args else 'all'
    if which != 'all' and which not in TESTS:
        print(f"Unknown test: {which}")
        print(f"Available: {', '.join(TESTS.keys())}, all")
        sys.exit(1)

    with serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT) as ser:
        # Every test requires initialization first
        initialize(ser, full=True)

        if which == 'all':
            test_basic(ser)
            test_position(ser)
            # force / grasp need a physical object; not run by default.
            print("\n(Skipping 'force' and 'grasp' - they need a physical object.")
            print(" Run them individually when you are ready.)")
        else:
            TESTS[which](ser)

    print("\nDone.")


if __name__ == '__main__':
    main()
