# espfly-autonomy

This repository holds the firmware and the Mac side tools I used to fly a 28 gram ESP-FLY micro drone around my apartment on its own. Claude flies it: it looks at the photos the drone sends back and decides what the drone should do next. The missions sounded simple when I wrote them down, and each one took far more work than I expected: find the frog toy and land in front of it, or land on the table in front of the TV.

The drone is a kit designed by Max Imagination and sold by Seeed Studio. It carries a single forward facing OV3660 camera on a XIAO ESP32-S3 Sense module and a six axis IMU, so height, position and speed all have to be worked out from those two sources, and most of the code in this repository is there to make that estimate good enough to fly on.

I wrote the full story up in an article called *Building a 28-gram autonomous drone*, and the link to it sits in the description of this repository.

## What is in the repository

Everything lives under `Firmware/esp-drone`, and the paths below are relative to that folder. I started from the version of Espressif's [ESP-Drone](https://github.com/espressif/esp-drone) project that ships with the Seeed [Co-Create ESP-FLY](https://github.com/Seeed-Projects/Co-Create_ESP-FLY) kit, a flight stack that itself grew out of the Crazyflie firmware, and built the autonomy layer on top of it.

The autonomy code is in `components/core/crazyflie/utils`. It covers the flight state machine (`auto_fsm`), the move planner (`auto_motion`), the camera based position hold (`camera_hold`), the observer that estimates vertical speed and height from the camera (`cam_vz_obs`), the vertical controller (`vert_ctrl`), the optical flow fit (`optical_flow`), the thermal compensation for the gyro and the accelerometer (`gyro_tempco`, `accel_tempco`), the cooling spin the drone performs on the ground (`ground_cool`) and the tables of guards and settings (`auto_glue_core`).

The camera pipeline, the wireless link to the Mac, the flight recorder and the USB diagnostic console are in `components/core/crazyflie/modules` (`camera_flow`, `cam_split`, `camera_jpeg`, `auto_link`, `usb_diag`).

On the Mac side, `tools/espfly_auto.py` is the daemon and command line client that the AI pilot talks to, and through it the pilot can take off, move, yaw, change height, take a photo and land. `tools/espfly_diag.py` talks to the USB console for settings, calibration, log dumps and trim checks. The host side C and Python tests are in `tools/tests`, and several of them replay recorded flights. The design notes I kept while the work was going on are in `docs`; `autonomy-a2b.md` is the main one, and `autonomy-bench.md` collects the bench procedures and the settings table.

To build the firmware you need ESP-IDF 5.0.7, after which `idf.py build` inside `Firmware/esp-drone` produces an application image that I flash at `0x10000`.

The recorded flight logs, the photos and the raw camera frames that two older flow tests relied on stay private, so I left those two tests and their fixtures out of the published tree. The replay fixtures and the design notes refer to the two people who appear in the flights as p1 and p2.

## Credits

Max Imagination designed the ESP-FLY and published a [build video](https://www.youtube.com/watch?v=3Y_drsQtMs4), and Seeed Studio sells the kit. The base firmware comes from Espressif's ESP-Drone under GPL-3.0. I developed the native 320x240 camera path, the JPEG photos and the cadence aware observer gates in a separate run of sessions with ChatGPT, and I wrote the rest of the autonomy layer together with Claude.

## Licence

The code is released under GPL-3.0, the same licence as the upstream firmware, and the full text is in `LICENSE`.
