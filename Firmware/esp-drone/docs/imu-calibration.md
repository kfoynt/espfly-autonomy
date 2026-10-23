# ESP-FLY level calibration, torque trim and USB diagnostics

This branch builds on the manual-flight firmware that was already on the
drone (`ESP_FLY_MANUAL_ONLY_V1`, commit `fee775a`). It adds:

1. **Accelerometer level calibration.** It removes the difference between
   "level" as the MPU6050 reports it and level as the airframe sits.
2. **Ground integrator hold.** The roll/pitch PID integrators stay at zero
   below 20000 thrust, so they cannot wind up while the drone is still on
   the ground.
3. **Torque trim.** A constant roll/pitch motor correction for a known,
   repeatable imbalance. It is applied from lift-off, so the controller does
   not have to rebuild the correction on every take-off.
4. **USB diagnostics console** on the XIAO's USB-C port.
5. **RAM flight recorder.** It keeps the last 60 s of flight at 100 Hz, in
   PSRAM. You can download it over USB after landing as long as the battery
   stays connected.

The level offset and the trim are stored in NVS. With neither stored, the
firmware flies like the original.

## Why this matters for "throttle only" flight

In Angle mode with centred sticks, the controller holds whatever attitude the
estimator calls level. A level error θ pushes the drone sideways at about
g·tan θ:

| level error | sideways acceleration | 1 m of drift from rest |
|---|---|---|
| 3°   | 0.51 m/s² | 2.0 s |
| 1.3° (this drone, before calibration) | 0.22 m/s² | 3.0 s |
| 0.3° | 0.05 m/s² | 6.2 s |

A second effect hits every take-off: if one corner needs more thrust, the
drone leans toward it until the integrators catch up, and it leaves the
ground already sliding. The torque trim removes most of that. The IMU cannot
sense position, so nothing in the firmware can stop a slide once it has
started.

## Batteries and brownouts

The 250 mAh battery sags hard under motor current. On a half-charged battery
the chip reset the moment the motors spun up: the props stop within a
second, and the drone's Wi-Fi drops because it reboots. `status` shows
`reset=brownout` after such a reset; `vbatMin` is the lowest voltage since
boot. Fly on a well-charged battery.

The firmware eases the battery in two ways, and neither changes how the
drone is controlled:

- **Motor PWM pulse placement.** The motors share one PWM timer (`pwm`,
  40 kHz). At or below 50 % duty (spin-up) the four pulses start at
  spread-out times. Above 50 % (hover) M1 and M3 pulse at the start of each
  period and M2 and M4 at its end (user decision 2026-10-13, evening). That
  cuts the period-to-period swing in battery current by 18-35 % at 65-85 %
  duty, for an estimated +0.5-2 % of air per charge.
- **Wi-Fi.** The access point uses 11 dBm (2 dBm during take-off with
  `tkoff txlo`) and, since 2026-10-13 (evening), 20 MHz bandwidth (HT20)
  instead of the IDF default of 40 MHz. HT20 is set at each boot and never
  saved to the Wi-Fi NVS, so flashing back an older build restores 40 MHz.

Details are in `docs/autonomy-a2b.md` section 15. `pwm` and `net` show what
is in effect.

## Commands

Run from `Firmware/esp-drone` with any Python 3 that has pyserial. ESP-IDF's
Python works: `. $HOME/esp/esp-idf-v5.0.7/export.sh`. Opening the port does
not reset the drone.

```bash
python3 tools/espfly_diag.py status            # firmware, reset reason, battery, offsets, trim, attitude, motors
python3 tools/espfly_diag.py cal show          # level offset and its source
python3 tools/espfly_diag.py trim show         # torque trim
python3 tools/espfly_diag.py dump --out f.csv  # download + summarise the flight recorder
python3 tools/espfly_diag.py stream --hz 20    # live attitude/gyro/acc/motors
```

`measure`, `cal set/clear` and `trim set/clear` only run with no thrust
requested and all motor outputs at zero. While they run, the motor driver
forces zero duty. Afterwards the app must send zero throttle before the
motors can start again.

## Level calibration (two-position method)

1. Power the drone and leave it still for about 5 s.
2. Put it on a flat, firm surface on its legs. Keep the USB cable slack so
   it doesn't tug.
3. Measure position A:

   ```bash
   python3 tools/espfly_diag.py measure --ms 5000 --out pos0.json
   ```

4. Turn the drone 180° on the same spot and measure position B:

   ```bash
   python3 tools/espfly_diag.py measure --ms 5000 --out pos180.json
   ```

   Repeat a measurement if it reports `still=False`.

5. Apply and save:

   ```bash
   python3 tools/espfly_diag.py level-cal pos0.json pos180.json --apply --save
   ```

   Without `--apply` it is a dry run. Averaging A and B cancels the
   surface's tilt.

Offsets are limited to ±0.15 g (≈8.6°). Keep the app's Pitch/Roll trim at 0.

## Torque trim

1. Fly a few hands-off throttle-only hops of at least 3 s each.
2. Dump each round to a CSV.
3. Dry run, then apply and save:

   ```bash
   python3 tools/espfly_diag.py trim-learn round1.csv round2.csv
   python3 tools/espfly_diag.py trim-learn round1.csv round2.csv --apply --save
   ```

`trim-learn` recovers the controller's steady hover correction from the motor
commands by inverting the X mixer, and suggests 70% of the median across
flights. Re-learn after a crash, a motor or prop change, or a change in
payload.

If the drone tips toward the **front-right** at take-off, the trim is too
strong. Re-learn it, or remove it:

```bash
python3 tools/espfly_diag.py trim clear
```

The trim ramps in between 20000 and 30000 thrust, and is never applied at
zero thrust.

## Flight recorder workflow

1. Fly short hops. Land, and set the throttle to zero.
2. Keep the battery connected and plug in USB-C.
3. Run `python3 tools/espfly_diag.py dump --out hop.csv`.

The summary shows, per flight and per time window after the throttle comes
up:

- commanded vs estimated attitude
- the PID integrators
- mean motor commands
- motor saturation
- whether any roll/pitch stick input was present

Motor values are commanded PWM, not measured thrust.

## Build and flash

```bash
mkdir -p build-imu && cp sdkconfig build-imu/sdkconfig
```

```bash
idf.py -B build-imu -D SDKCONFIG=build-imu/sdkconfig build
```

```bash
python -m esptool --chip esp32s3 -p /dev/cu.usbmodem2101 write_flash --flash_mode keep --flash_freq keep --flash_size keep 0x10000 build-imu/ESPDrone.bin
```

The `-D SDKCONFIG` copy keeps the tracked `sdkconfig` untouched.

Flash only the app partition at 0x10000. The bootloader and partition table
on the drone are left as they are, and NVS is never written by flashing, so
the calibration and trim survive reflashing.

## Rollback

| Goal | How |
|---|---|
| Remove the trim only | `python3 tools/espfly_diag.py trim clear` |
| Remove the level calibration only | `python3 tools/espfly_diag.py cal clear` |
| Restore the whole original flash, bit for bit (8 MB image) | `python -m esptool --chip esp32s3 -p /dev/cu.usbmodem2101 write_flash 0x0 ../../firmware-backups/espfly-original-2026-10-10.bin` |
| Restore the original source | commit `fee775a` on this branch; stock upstream is `main` |

Restoring the whole original flash also restores the original NVS, so the
calibration and trim records disappear.
