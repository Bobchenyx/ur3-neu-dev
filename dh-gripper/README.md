# DH AG Gripper Test Script

Modbus-RTU test script for the DH Robotics AG gripper.

## Serial Port Permissions (Linux)

```bash
sudo chmod 666 /dev/ttyUSB0
```

If your adapter is on a different device, check with `ls /dev/ttyUSB*` and
update the `PORT` constant at the top of the script.

## Usage

```bash
python dh_gripper_test_v2.py              # default: basic + position
python dh_gripper_test_v2.py basic        # basic open/close
python dh_gripper_test_v2.py position     # position accuracy scan
python dh_gripper_test_v2.py force        # force comparison (needs object)
python dh_gripper_test_v2.py grasp        # grasp + drop detection (needs object)
```

Every run starts with a full initialization, which drives the fingers to
both limits. Takes a few seconds.

## Tests and Expected Output

### `basic` - open/close sanity check

```
=== [basic] Basic open/close ===
-> Open to 1000 (fully open)
   final state: arrived (no object), actual position: 1000
-> Close to 0
   final state: arrived (no object), actual position: 0
```

Check: both rows say `arrived (no object)`, positions near 0 and 1000.

---

### `position` - accuracy scan

```
=== [position] Position accuracy scan ===
 target  actual   error  state
---------------------------------------------
      0       0      +0  arrived (no object)
    250     251      +1  arrived (no object)
    500     500      +0  arrived (no object)
    750     749      -1  arrived (no object)
   1000    1000      +0  arrived (no object)
    500     501      +1  arrived (no object)
    100      99      -1  arrived (no object)
    900     900      +0  arrived (no object)
```

Check: `error` within +/- 2. Larger errors suggest a bad initialization.

---

### `force` - force comparison

Put a sponge or piece of foam between the fingers first.

```
=== [force] Force comparison ===
 force   stopped at    time(s)  state
-------------------------------------------------------
   20%          620       1.85  object caught
   50%          540       1.20  object caught
  100%          480       0.90  object caught
```

Check:
- All three rows say `object caught` (state 2)
- Higher force -> fingers stop further in (smaller position)
- Higher force -> shorter time

---

### `grasp` - drop detection

Put an object between the fingers, then manually pull it out during the
15-second monitor window.

```
=== [grasp] Grasp and drop detection ===
-> Closing to grasp...
   initial state: object caught, position: 380
-> Monitoring for 15s. You can pull the object out and watch for a drop.
   [  4.2s] state change: moving, position: 260
   [  4.5s] state change: object dropped, position: 0
Monitoring done.
```

Check: initial state is `object caught`, pulling the object out triggers
`object dropped` (state 3).

## Troubleshooting

- **Permission denied on serial port** -> run the `chmod` above
- **CRC check failed / response too short** -> check 24V power and RS485 A/B wiring
- **Initialization timed out** -> fingers blocked or something preventing full travel
- **Gripper does not move** -> it did not initialize; re-run and watch for `[init] Done`

## Registers Used

| Address  | Description                                          |
|----------|------------------------------------------------------|
| `0x0100` | Init (0x01=normal, 0xA5=full)                        |
| `0x0101` | Force, 20-100 (%)                                    |
| `0x0103` | Target position, 0-1000                              |
| `0x0200` | Init state: 0=not ready, 1=ready                     |
| `0x0201` | Grip state: 0=moving, 1=arrived, 2=caught, 3=dropped |
| `0x0202` | Current actual position                              |
