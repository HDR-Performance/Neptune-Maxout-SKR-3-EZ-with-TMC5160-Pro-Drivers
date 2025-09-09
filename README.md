# Neptune-Maxout-SKR-3-EZ-with-TMC5160-Pro-Drivers

This tutorial guides you through upgrading an Elegoo Neptune 3 Max to a BIGTREETECH SKR 3 EZ mainboard with TMC5160 Pro EZ drivers, using a tested Klipper printer.cfg. If upgrading from a Maxout mod on the stock Robin Nano 2.2 board, you’ll primarily need to update the printer.cfg and verify the MCU serial ID. For a full upgrade, follow the steps below to wire components, flash firmware, set up the BTT Pad 7 (or Raspberry Pi 4/5), and configure Klipper. The complete printer.cfg is provided at the end for reference.
Note: This guide is tailored for the Neptune 3 series printers (including the Neptune 3 Max). All stock harness connectors from the Neptune 3 series are fully plug-and-play with the SKR 3 EZ board, requiring no rewiring or pin adaptations. The SKR 3 EZ configures TMC5160 drivers for SPI mode via software in the printer.cfg, so no physical SPI jumpers are needed. The display is either the BTT Pad 7 or no physical display if using a Raspberry Pi 4/5, accessing Mainsail via the device’s IP address in a web browser.
Prerequisites

Tools: Screwdrivers, computer with internet access.
Components: SKR 3 EZ mainboard, TMC5160 Pro EZ drivers (for X, Y, Z, extruder), BTT Pad 7 or Raspberry Pi 4/5, Elegoo Neptune 3 Max components (stepper motors, hotend, heated bed, fans, probe, filament sensor, 24V LED lights).
Software: PuTTY (SSH client), Raspberry Pi Imager, Klipper firmware for STM32H723 or STM32H743.
Safety: Disconnect power before wiring. Double-check connections to avoid short circuits.

Step-by-Step Instructions
Step 1: Wiring the SKR 3 EZ to Neptune 3 Max Components
For the Neptune 3 series printers (including the Neptune 3 Max), all stock harness connectors (motors, heaters, fans, endstops, thermistors, probe, filament sensor, and LED lights) are fully compatible with the SKR 3 EZ board, making the installation completely plug-and-play. No rewiring or pin adaptations are needed. Refer to the SKR 3 EZ pinout diagram (available in the BIGTREETECH documentation) to confirm port locations.
1.1 Install TMC5160 Pro EZ Drivers

Install TMC5160 Pro EZ drivers into the X, Y, Z, and E0 slots on the SKR 3 EZ board before mounting the board in the printer’s enclosure.
Ensure drivers are firmly seated and oriented correctly (check pin alignment to avoid damage).
Note: The SKR 3 EZ configures TMC5160 drivers for SPI mode via software in the printer.cfg (see [tmc5160] sections). No physical SPI jumpers need to be set.
Caution: Handle drivers carefully to avoid static discharge. Install them on a clean, non-conductive surface.

1.2 Prepare the Harness

Disconnect all harnesses from the stock Robin Nano board.
Identify the main bundles: motor harnesses (X, Y, Z, Extruder), heater harnesses (hotend and bed), fan harnesses, sensor harnesses (endstops, probe, filament sensor), power harness, and LED light harness.
Label connectors if needed to avoid confusion during reconnection.

1.3 Mount the SKR 3 EZ

Secure the SKR 3 EZ board (with drivers installed) in the printer’s electronics enclosure, replacing the stock board.
Ensure proper ventilation and secure mounting with screws or standoffs.

1.4 Connect the Main Power Harness

Locate the 24V power input harness from the Neptune 3 series PSU (usually a 2-pin connector with red and black wires).
Plug it directly into the POWER IN terminals on the SKR 3 EZ (V+ for positive/red, V- for negative/black).
Do not power on yet—continue with other connections.

1.5 Connect Motor Harnesses (Plug-and-Play)

X-Axis Motor: Plug the X-motor harness (4-pin JST connector) directly into the X-MOT port on the SKR 3 EZ.
Y-Axis Motor: Plug into the Y-MOT port.
Z-Axis Motor: Plug into the Z-MOT port.
Extruder Motor: Plug into the E0-MOT port.
Note: These are standard 4-pin stepper motor connectors (A1, A2, B1, B2) and are fully plug-and-play.

1.6 Connect Heater and Fan Harnesses (Plug-and-Play)

Hotend Heater: Plug the hotend heater harness (2-pin connector) into the HE0 output.
Heated Bed: Plug the bed heater harness into the BED output (polarity is typically not critical, but align red to positive if marked).
Hotend Fan: Plug into the HEATER_FAN port.
Part Cooling Fan: Plug into the FAN0 port.
Note: These are standard 2-pin connectors and plug-and-play.

1.7 Connect Sensor and Display Harnesses (Plug-and-Play)
All sensor harnesses are plug-and-play. The stock Neptune 3 series display is not used; instead, use the BTT Pad 7 as the display, or skip a physical display if using a Raspberry Pi 4/5 and access Mainsail via a web browser.

X-Axis Endstop: Plug into the X-STOP port (signal to PC1).
Y-Axis Endstop: Plug into the Y-STOP port (signal to PC3).
Hotend Thermistor: Plug into the TH0 port (signal to PA2).
Bed Thermistor: Plug into the TB port (signal to PA1).
Probe: Plug into the PROBE port (signal to PC0, includes 5V and GND).
Filament Sensor: Plug into the FIL-DET port (signal to PC2, includes 5V/24V as per sensor specs).
Display:

BTT Pad 7: Serves as the primary display and control interface. Connect it to the SKR 3 EZ via USB (for communication, configured in Step 8) and power it separately (per the BTT Pad 7 manual). The stock Neptune 3 series display harness (for EXP1/EXP2) is not used.
Raspberry Pi 4/5 (No Physical Display): Skip connecting a physical display. Access Mainsail by entering the Pi’s IP address in your preferred web browser (configured in Step 6). The EXP1 and EXP2 headers on the SKR 3 EZ remain unused.



1.8 Verify Plug-and-Play Compatibility

All Neptune 3 series harnesses (motors, heaters, fans, endstops, thermistors, probe, filament sensor, power, and LED lights) are designed to plug directly into the SKR 3 EZ ports (X-MOT, Y-MOT, Z-MOT, E0-MOT, HE0, BED, HEATER_FAN, FAN0, X-STOP, Y-STOP, TH0, TB, PROBE, FIL-DET). Keyed connectors should align without force.
Test fit all connectors before powering on to ensure secure connections.

1.9 Optional Components

LED (Optional): The stock Neptune 3 series LED lights are 24V and should be connected to a fan PWM output for control.

Connection: Plug the LED harness into the fan PWM output PB9 (signal, as indicated in the commented-out [led LED_Light] section). Connect the positive wire to 24V (from a 24V power terminal on the SKR 3 EZ) and the negative wire to PB9 for PWM control. The stock LED connector is plug-and-play and PWM-compatible.
Settings: Controlled via SET_LED commands (e.g., 25% brightness in START_PRINT). Uncomment and configure the [led LED_Light] section in printer.cfg if needed:
text[led LED_Light]
white_pin: PB9
shutdown_speed: 1.0
cycle_time: 0.010



ADXL345 (Accelerometer): Connect to the SPI bus on the BTT Pad 7 or Raspberry Pi 4/5 as specified in [adxl345] (spi_bus: spidev1.1).

Step 2: Install Required Software

Download PuTTY: Install PuTTY for SSH access to the BTT Pad 7 or Raspberry Pi. Download from https://puttygen.com/download-putty.
Install Raspberry Pi Imager: Download and install from https://www.raspberrypi.com/software/ to flash the SD card for the BTT Pad 7 or Raspberry Pi.

Step 3: Flash the BTT Pad 7 or Raspberry Pi

Download the Firmware Image: Obtain the Neptune Maxout SKR 3 EZ image for the BTT Pad 7 or a Klipper-compatible image for the Raspberry Pi 4/5. Check with your supplier or community forums for the download link.
Flash the SD Card: Use Raspberry Pi Imager to flash the downloaded image onto a microSD card.
Insert SD Card: Insert the flashed microSD card into the BTT Pad 7 or Raspberry Pi and power it on (do not connect the USB cable to the SKR 3 EZ yet).

Step 4: Flash the SKR 3 EZ Firmware

Identify the Processor: Check your SKR 3 EZ board to confirm the processor type (STM32H723 or STM32H743). Newer boards typically use the STM32H723 (550 MHz for better performance).
Compile Klipper Firmware:

Compile Klipper for the STM32H723 (or STM32H743) with a 128KiB bootloader and USB communication on pins PA11/PA12.
Refer to Klipper documentation: https://www.klipper3d.org/Installation.html.


Flash the Firmware:

Rename the compiled klipper.bin to firmware.bin.
Copy firmware.bin to the root of a blank microSD card.
Insert the SD card into the SKR 3 EZ and power it on to flash the firmware.
Verify the update by checking for a confirmation file (e.g., firmware.cur) on the SD card.



Step 5: Configure USB Mode on SKR 3 EZ

Set USB Mode: Locate the jumper or switch behind the USB port on the SKR 3 EZ. Set it to USB mode (highest position). Most boards ship in CAN mode (lowest position).
Do Not Connect USB Yet: Keep the USB cable between the BTT Pad 7 (or Raspberry Pi) and SKR 3 EZ disconnected.

Step 6: Connect BTT Pad 7 or Raspberry Pi to Network

Boot the Device: Power on the BTT Pad 7 or Raspberry Pi with the flashed SD card.
Connect to Network:

Use the BTT Pad 7’s settings menu (under Wi-Fi) or Raspberry Pi’s configuration to connect to your Wi-Fi network, or plug in an RJ45 LAN cable.
Find the device’s IP address in the settings menu or via your router’s device list (e.g., 192.168.1.100).



Step 7: SSH into BTT Pad 7 or Raspberry Pi and Find MCU Serial ID

Open PuTTY: Launch PuTTY on your computer.
SSH into Device:

Enter the BTT Pad 7 or Raspberry Pi’s IP address in PuTTY’s “Host Name” field.
For BTT Pad 7, use default credentials: Username biqu, Password biqu (unless changed).
For Raspberry Pi, use default credentials: Username pi, Password raspberry (unless changed).
Click “Open” to connect.


Find Serial ID:

In the PuTTY terminal, type: ls /dev/serial/by-id/
The output will look like: usb-Klipper_stm32h723xx_3A000F001251333031373138-if00
Copy this serial ID for the next step.



Step 8: Update printer.cfg with Serial ID

Access Mainsail:

Open a web browser and enter the BTT Pad 7 or Raspberry Pi’s IP address (e.g., 192.168.1.100) to access the Mainsail interface.


Edit printer.cfg:

Navigate to the printer.cfg file in Mainsail’s configuration section.
Locate the [mcu] section: [mcu] serial: /dev/serial/by-id/
Paste the serial ID from Step 7, e.g.: serial: /dev/serial/by-id/usb-Klipper_stm32h723xx_3A000F001251333031373138-if00


Save and Restart: Click “Save & Restart” in the top-right corner of Mainsail to apply changes and reboot Klipper.

Step 9: Test the Setup

Power On: Connect the USB cable between the SKR 3 EZ and BTT Pad 7 or Raspberry Pi, then power on both devices.
Verify Connections:

In Mainsail, run G28 to home all axes.
Test the hotend with M104 S200 (set hotend to 200°C).
Test the bed with M140 S60 (set bed to 60°C).
Test the fan with M106 S255 (full speed).
Test the LED lights with SET_LED LED=LED_Light WHITE=0.25 (25% brightness, if the [led LED_Light] section is enabled).
Check the probe with QUERY_PROBE.
Check the filament sensor with QUERY_FILAMENT_SENSOR SENSOR=filament_sensor.


Calibrate PID Settings:

Hotend PID Tuning: Run NOZZLE_PID_TUNE TEMP=200 FAN_SPEED=0 to tune the hotend PID at 200°C with the part cooling fan off. This macro (defined in printer.cfg) homes the printer, disables the hotend fan, runs the PID calibration, and saves the results to printer.cfg. Check the Mainsail terminal for the new PID values after completion.
Bed PID Tuning: Run BED_PID_TUNE TEMP=60 to tune the bed PID at 60°C. This macro homes the printer, runs the PID calibration, and saves the results to printer.cfg. Check the Mainsail terminal for the new PID values.
Note: These temperatures (200°C for hotend, 60°C for bed) are suitable defaults for common filaments like PLA. Adjust the TEMP parameter if using different materials (e.g., 240°C for ABS on the hotend, 80°C for PETG on the bed). Ensure the printer is in a well-ventilated area during PID tuning to avoid overheating.



Step 10: Final Configuration

Copy Additional Files: Ensure included files (KAMP_Settings.cfg, mainsail.cfg, timelapse.cfg, Line_Purge.cfg, Smart_Park.cfg, Adaptive_Meshing.cfg) are in your Klipper config directory.
Test a Print: Use the START_PRINT macro to run a test print and verify all components work together.
Save Config: After PID tuning and other calibrations, run SAVE_CONFIG to store settings like PID values to printer.cfg.

Troubleshooting

No MCU Connection: Verify the USB mode jumper, serial ID in printer.cfg, and USB connection.
Motor Issues: Check that motor connectors are securely plugged into the correct ports (X-MOT, Y-MOT, Z-MOT, E0-MOT). Adjust dir_pin inversion (!) in printer.cfg if motors move incorrectly.
Probe Not Triggering: Ensure the probe connector is securely plugged into the PROBE port and test with QUERY_PROBE.
SPI Errors: Verify TMC5160 CS and SPI bus settings in printer.cfg (no physical jumpers needed).
LED Issues: Ensure the 24V LED connector is plugged into PB9 (fan PWM output) and 24V power, and uncomment the [led LED_Light] section in printer.cfg.
Mainsail Access Issues: Ensure the BTT Pad 7 or Raspberry Pi is on the network and the IP address is correct.
PID Tuning Issues: Ensure the printer is homed (G28) before running NOZZLE_PID_TUNE or BED_PID_TUNE. Check the Mainsail terminal for errors if tuning fails.

Conclusion
Your Neptune 3 Max is now upgraded with the SKR 3 EZ and TMC5160 Pro EZ drivers, running Klipper with advanced features like adaptive meshing and input shaping. All Neptune 3 series harnesses are plug-and-play, simplifying the wiring process. The BTT Pad 7 or Raspberry Pi 4/5 with Mainsail provides a modern interface, and the stock 24V LED lights are controlled via the PB9 fan PWM output. PID tuning ensures optimal temperature control for the hotend and bed. Test thoroughly and enjoy improved performance! For further assistance, refer to the Klipper documentation or BIGTREETECH support.
Reference: Klipper printer.cfg
Below is the complete printer.cfg used for this setup. Copy this into your Klipper configuration directory, ensuring the MCU serial ID is updated as described in Step 8. To enable LED control, uncomment and configure the [led LED_Light] section as noted.
