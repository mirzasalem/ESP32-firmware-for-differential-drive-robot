# Required Arduino libraries

## ESP32Encoder (required)

The firmware uses quadrature encoders on the ESP32 PCNT peripheral.

| Item | Value |
|------|--------|
| Name | **ESP32Encoder** |
| Author | Kevin Harrington (madhephaestus) |
| Min version | 0.11.7 (tested) |
| Repo | https://github.com/madhephaestus/ESP32Encoder |

### Install option A — Arduino Library Manager

1. **Sketch → Include Library → Manage Libraries**
2. Search **ESP32Encoder**
3. Install **ESP32Encoder** by Kevin Harrington

If you see *“already installed but different version”*, either keep 0.11.7 or remove and reinstall:

```bash
rm -rf ~/Arduino/libraries/ESP32Encoder
```

Then install again from the Manager or use option B.

### Install option B — copy from this workspace

```bash
cp -r ~/ros2_ws/ESP32Encoder ~/Arduino/libraries/ESP32Encoder
```

Restart the Arduino IDE after installing.

### Do not flash the library examples alone

Folders like `ESP32Encoder/examples/*.ino` are **tests only**. Always upload:

`esp2ros2/firmware/ROSArduinoBridge/ROSArduinoBridge.ino`

## Cytron driver (optional, not default)

If you enable `CYTRON_MDD3A` in `ROSArduinoBridge.ino`, install the **CytronMotorDriver** library used by your board vendor. Default buddy esp2ros2 build uses **L298N** only.
