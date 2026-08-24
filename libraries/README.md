# Required Arduino Libraries

## ESP32Encoder (required)

Quadrature encoder counting uses the ESP32 PCNT peripheral through this library.

| Item | Value |
|------|--------|
| Name | **ESP32Encoder** |
| Author | Kevin Harrington (madhephaestus) |
| Minimum tested version | 0.11.7 |
| Repository | https://github.com/madhephaestus/ESP32Encoder |

### Install via Library Manager

1. **Sketch → Include Library → Manage Libraries**
2. Search **ESP32Encoder**
3. Install **ESP32Encoder** by Kevin Harrington

To force a clean reinstall:

```bash
rm -rf ~/Arduino/libraries/ESP32Encoder
```

Then install again from the Library Manager.

### Install by copy

```bash
cp -r /path/to/ESP32Encoder ~/Arduino/libraries/ESP32Encoder
```

Restart the Arduino IDE after installing.

### Upload target

Do not flash library example sketches for robot operation. Always upload:

```text
firmware/ROSArduinoBridge/ROSArduinoBridge.ino
```

## Cytron motor driver (optional)

Only required if `CYTRON_MDD3A` is enabled in `ROSArduinoBridge.ino`. The default configuration uses **L298N** and does not need this library.

## IMU

The MPU9250 driver uses the built-in Arduino `Wire` API. No additional IMU library is required.
