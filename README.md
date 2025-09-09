# Neptune-Maxout-SKR-3-EZ-with-TMC5160-Pro-Drivers

Neptune Maxout: Upgrading Elegoo Neptune 3 Max to SKR 3 EZ with TMC5160 Pro Drivers
This tutorial guides you through upgrading an Elegoo Neptune 3 Max to a BIGTREETECH SKR 3 EZ mainboard with TMC5160 Pro EZ drivers, using a tested Klipper printer.cfg. If upgrading from a Maxout mod on the stock Robin Nano 2.2 board, you’ll primarily need to update the printer.cfg and verify the MCU serial ID. For a full upgrade, follow the steps below to wire components, flash firmware, set up the BTT Pad 7 (or Raspberry Pi 4/5), and configure Klipper.
Note: This guide is tailored for the Neptune 3 series printers (including the Neptune 3 Max). All stock harness connectors are fully plug-and-play with the SKR 3 EZ board, requiring no rewiring. The SKR 3 EZ configures TMC5160 drivers for SPI mode via software in the printer.cfg, so no physical SPI jumpers are needed. The display is either the BTT Pad 7 or no physical display if using a Raspberry Pi 4/5, accessing Mainsail via the device’s IP address in a web browser.
Prerequisites

Tools: Screwdrivers, computer with internet access.
Components: SKR 3 EZ mainboard, TMC5160 Pro EZ drivers (for X, Y, Z, extruder), BTT Pad 7 or Raspberry Pi 4/5, Elegoo Neptune 3 Max components (stepper motors, hotend, heated bed, fans, probe, filament sensor, 24V LED lights).
Software: PuTTY (SSH client), Raspberry Pi Imager, Klipper firmware for STM32H723 or STM32H743.
Safety: Disconnect power before wiring. Double-check connections to avoid short circuits.

Step-by-Step Instructions
Step 1: Wiring the SKR 3 EZ to Neptune 3 Max Components
All stock harness connectors for the Neptune 3 series (motors, heaters, fans, endstops, thermistors, probe, filament sensor, and LED lights) are fully compatible with the SKR 3 EZ board, making the installation plug-and-play. Refer to the SKR 3 EZ pinout diagram for port locations.
1.1 Install TMC5160 Pro EZ Drivers

Install TMC5160 Pro EZ drivers into the X, Y, Z, and E0 slots on the SKR 3 EZ board before mounting in the printer’s enclosure.
Ensure drivers are firmly seated and oriented correctly (check pin alignment).
Note: The SKR 3 EZ configures drivers for SPI mode via the printer.cfg (see [tmc5160] sections). No physical SPI jumpers are needed.
Caution: Handle drivers carefully to avoid static discharge. Install on a clean, non-conductive surface.

1.2 Prepare the Harness

Disconnect all harnesses from the stock Robin Nano board.
Identify the main bundles: motor harnesses (X, Y, Z, Extruder), heater harnesses (hotend and bed), fan harnesses, sensor harnesses (endstops, probe, filament sensor), power harness, and LED light harness.
Label connectors if needed to avoid confusion.

1.3 Mount the SKR 3 EZ

Secure the SKR 3 EZ board (with drivers installed) in the printer’s electronics enclosure, replacing the stock board.
Ensure proper ventilation and secure mounting with screws or standoffs.

1.4 Connect the Main Power Harness

Locate the 24V power input harness from the Neptune 3 series PSU (usually a 2-pin connector with red and black wires).
Plug it into the POWER IN terminals on the SKR 3 EZ (V+ for positive/red, V- for negative/black).
Do not power on yet—continue with other connections.

1.5 Connect Motor Harnesses (Plug-and-Play)

X-Axis Motor: Plug the X-motor harness (4-pin JST connector) into the X-MOT port.
Y-Axis Motor: Plug into the Y-MOT port.
Z-Axis Motor: Plug into the Z-MOT port.
Extruder Motor: Plug into the E0-MOT port.
Note: These are standard 4-pin stepper motor connectors (A1, A2, B1, B2) and plug-and-play.

1.6 Connect Heater and Fan Harnesses (Plug-and-Play)

Hotend Heater: Plug the hotend heater harness (2-pin connector) into the HE0 output.
Heated Bed: Plug the bed heater harness into the BED output (polarity is typically not critical, but align red to positive if marked).
Hotend Fan: Plug into the HEATER_FAN port.
Part Cooling Fan: Plug into the FAN0 port.
Note: These are standard 2-pin connectors and plug-and-play.

1.7 Connect Sensor and Display Harnesses (Plug-and-Play)
All sensor harnesses are plug-and-play. The stock Neptune 3 series display is not used; instead, use the BTT Pad 7 or skip a physical display with a Raspberry Pi 4/5.

X-Axis Endstop: Plug into the X-STOP port (signal to PC1).
Y-Axis Endstop: Plug into the Y-STOP port (signal to PC3).
Hotend Thermistor: Plug into the TH0 port (signal to PA2).
Bed Thermistor: Plug into the TB port (signal to PA1).
Probe: Plug into the PROBE port (signal to PC0, includes 5V and GND).
Filament Sensor: Plug into the FIL-DET port (signal to PC2, includes 5V/24V as per specs).
Display:
BTT Pad 7: Serves as the primary display and control interface. Connect to the SKR 3 EZ via USB (configured in Step 8) and power separately (per the BTT Pad 7 manual). The stock display harness (EXP1/EXP2) is not used.
Raspberry Pi 4/5 (No Physical Display): Skip connecting a display. Access Mainsail via the Pi’s IP address in a web browser (configured in Step 6). EXP1 and EXP2 headers remain unused.



1.8 Verify Plug-and-Play Compatibility

All Neptune 3 series harnesses (motors, heaters, fans, endstops, thermistors, probe, filament sensor, power, and LED lights) plug directly into the SKR 3 EZ ports (X-MOT, Y-MOT, Z-MOT, E0-MOT, HE0, BED, HEATER_FAN, FAN0, X-STOP, Y-STOP, TH0, TB, PROBE, FIL-DET). Keyed connectors should align without force.
Test fit all connectors before powering on to ensure secure connections.

1.9 Optional Components

LED (Optional): The stock Neptune 3 series LED lights are 24V and use a fan PWM output for control.
Connection: Plug the LED harness into the fan PWM output PB9 (signal, as in the commented-out [led LED_Light] section). Connect the positive wire to 24V (from a 24V power terminal on the SKR 3 EZ) and the negative wire to PB9. The stock LED connector is plug-and-play and PWM-compatible.
Settings: Controlled via SET_LED commands (e.g., 25% brightness in START_PRINT). Uncomment and configure the [led LED_Light] section in printer.cfg if needed:[led LED_Light]
white_pin: PB9
shutdown_speed: 1.0
cycle_time: 0.010




ADXL345 (Accelerometer): Connect to the SPI bus on the BTT Pad 7 or Raspberry Pi 4/5 as specified in [adxl345] (spi_bus: spidev1.1).

Step 2: Install Required Software

Download PuTTY for SSH access to the BTT Pad 7 or Raspberry Pi: https://puttygen.com/download-putty.
Download Raspberry Pi Imager to flash the SD card: https://www.raspberrypi.com/software/.

Step 3: Flash the BTT Pad 7 or Raspberry Pi

Download the Firmware Image: Obtain the Neptune Maxout SKR 3 EZ image for the BTT Pad 7 or a Klipper-compatible image for the Raspberry Pi 4/5. Check with your supplier or community forums for the download link.
Flash the SD Card: Use Raspberry Pi Imager to flash the image onto a microSD card.
Insert SD Card: Insert the card into the BTT Pad 7 or Raspberry Pi and power it on (do not connect the USB cable to the SKR 3 EZ yet).

Step 4: Flash the SKR 3 EZ Firmware

Identify the Processor: Check your SKR 3 EZ board for the processor type (STM32H723 or STM32H743). Newer boards typically use the STM32H723 (550 MHz).
Compile Klipper Firmware:
Compile Klipper for the STM32H723 (or STM32H743) with a 128KiB bootloader and USB on PA11/PA12.
See Klipper documentation: https://www.klipper3d.org/Installation.html.


Flash the Firmware:
Rename klipper.bin to firmware.bin.
Copy firmware.bin to the root of a blank microSD card.
Insert the card into the SKR 3 EZ and power it on to flash.
Verify the update by checking for a firmware.cur file on the SD card.



Step 5: Configure USB Mode on SKR 3 EZ

Set USB Mode: Locate the jumper/switch behind the USB port on the SKR 3 EZ. Set it to USB mode (highest position). Most boards ship in CAN mode (lowest position).
Do Not Connect USB Yet: Keep the USB cable between the BTT Pad 7 (or Raspberry Pi) and SKR 3 EZ disconnected.

Step 6: Connect BTT Pad 7 or Raspberry Pi to Network

Boot the Device: Power on the BTT Pad 7 or Raspberry Pi with the flashed SD card.
Connect to Network:
Use the BTT Pad 7’s settings menu (Wi-Fi) or Raspberry Pi’s configuration to connect to Wi-Fi, or use an RJ45 LAN cable.
Find the device’s IP address in the settings or via your router (e.g., 192.168.1.100).



Step 7: SSH into BTT Pad 7 or Raspberry Pi and Find MCU Serial ID

Open PuTTY: Launch PuTTY on your computer.
SSH into Device:
Enter the BTT Pad 7 or Raspberry Pi’s IP address in PuTTY’s “Host Name” field.
BTT Pad 7 credentials: Username biqu, Password biqu (unless changed).
Raspberry Pi credentials: Username pi, Password raspberry (unless changed).
Click “Open” to connect.


Find Serial ID:
In PuTTY, type: ls /dev/serial/by-id/
Output example: usb-Klipper_stm32h723xx_3A000F001251333031373138-if00
Copy the serial ID.



Step 8: Update printer.cfg with Serial ID

Access Mainsail:
Open a browser and enter the BTT Pad 7 or Raspberry Pi’s IP address (e.g., 192.168.1.100) to access Mainsail.


Edit printer.cfg:
Navigate to printer.cfg in Mainsail’s configuration section.
Locate [mcu] serial: /dev/serial/by-id/.
Paste the serial ID from Step 7, e.g.: serial: /dev/serial/by-id/usb-Klipper_stm32h723xx_3A000F001251333031373138-if00.


Save and Restart: Click “Save & Restart” in Mainsail to apply changes and reboot Klipper.

Step 9: Test the Setup

Power On: Connect the USB cable between the SKR 3 EZ and BTT Pad 7 or Raspberry Pi, then power on both devices.
Verify Connections:
In Mainsail, run G28 to home all axes.
Test the hotend: M104 S200 (set to 200°C).
Test the bed: M140 S60 (set to 60°C).
Test the fan: M106 S255 (full speed).
Test LED lights: SET_LED LED=LED_Light WHITE=0.25 (25% brightness, if [led LED_Light] is enabled).
Check probe: QUERY_PROBE.
Check filament sensor: QUERY_FILAMENT_SENSOR SENSOR=filament_sensor.


Calibrate PID Settings:
Hotend PID Tuning: Run NOZZLE_PID_TUNE TEMP=200 FAN_SPEED=0 to tune the hotend PID at 200°C with the part cooling fan off. This macro homes the printer, disables the hotend fan, runs PID calibration, and saves results to printer.cfg. Check Mainsail’s terminal for new PID values.
Bed PID Tuning: Run BED_PID_TUNE TEMP=60 to tune the bed PID at 60°C. This macro homes the printer, runs PID calibration, and saves results to printer.cfg. Check the terminal for new PID values.
Note: Use 200°C (hotend) and 60°C (bed) for PLA. Adjust TEMP for other materials (e.g., 240°C for ABS hotend, 80°C for PETG bed). Perform tuning in a well-ventilated area.



Step 10: Final Configuration

Copy Additional Files: Ensure included files (KAMP_Settings.cfg, mainsail.cfg, timelapse.cfg, Line_Purge.cfg, Smart_Park.cfg, Adaptive_Meshing.cfg) are in your Klipper config directory.
Test a Print: Use the START_PRINT macro to run a test print and verify all components.
Save Config: After PID tuning, run SAVE_CONFIG to store settings to printer.cfg.

Troubleshooting

No MCU Connection: Verify USB mode jumper, serial ID in printer.cfg, and USB connection.
Motor Issues: Ensure motor connectors are secure in X-MOT, Y-MOT, Z-MOT, E0-MOT. Adjust dir_pin inversion (!) in printer.cfg if motors move incorrectly.
Probe Not Triggering: Ensure the probe connector is secure in PROBE. Test with QUERY_PROBE.
SPI Errors: Verify TMC5160 CS and SPI settings in printer.cfg.
LED Issues: Ensure the 24V LED connector is in PB9 (fan PWM output) and 24V power. Uncomment [led LED_Light] in printer.cfg.
Mainsail Access Issues: Confirm the BTT Pad 7 or Raspberry Pi is on the network and the IP is correct.
PID Tuning Issues: Ensure the printer is homed (G28) before running NOZZLE_PID_TUNE or BED_PID_TUNE. Check Mainsail terminal for errors.

Conclusion
Your Neptune 3 Max is upgraded with the SKR 3 EZ and TMC5160 Pro EZ drivers, running Klipper with advanced features like adaptive meshing and input shaping. All harnesses are plug-and-play, and SPI mode is software-configured. The BTT Pad 7 or Raspberry Pi 4/5 with Mainsail offers a modern interface, with 24V LED lights controlled via PB9. PID tuning ensures optimal temperature control. Test thoroughly and enjoy improved performance! For help, see the Klipper documentation or BIGTREETECH support.
Reference: Klipper printer.cfg
Below is the complete printer.cfg. Copy it to your Klipper configuration directory, updating the MCU serial ID (Step 8). Uncomment and configure [led LED_Light] for LED control.
# Adapted Klipper printer.cfg for Elegoo Neptune 3 Max with BIGTREETECH SKR 3 EZ and TMC5160 Pro EZ Drivers
# This config merges your stock Robin Nano config with the base SKR 3 EZ setup.
# Firmware: Compile for STM32H743 with 128KiB bootloader, USB on PA11/PA12.
# Flash: Rename klipper.bin to firmware.bin, copy to SD card root, power on to update.
# See docs/Config_Reference.md for a description of parameters.
[mcu]
serial: /dev/serial/by-id/usb-Klipper_stm32h723xx_3A000F001251333031373138-if00

[include KAMP_Settings.cfg]
[include timelapse.cfg]
[include mainsail.cfg]
[include Line_Purge.cfg]
[include Smart_Park.cfg]
[include Adaptive_Meshing.cfg]
[exclude_object] # Comment out for Sonic Pad

[virtual_sdcard]
#path: ~/gcode_files

[pause_resume]

[display_status]

[gcode_macro START_PRINT]
gcode:
    SET_LED LED=LED_Light WHITE=0.25 SYNC=0 TRANSMIT=1 ; 25 Percent Led Light
    {% set BED_TEMP = params.BED_TEMP|default(60)|float %}
    {% set EXTRUDER_TEMP = params.EXTRUDER_TEMP|default(200)|float %}
    M140 S{BED_TEMP}
    M104 S{EXTRUDER_TEMP}
    G28
    M190 S{BED_TEMP}
    M109 S{EXTRUDER_TEMP}
    SETUP_KAMP_MESHING DISPLAY_PARAMETERS=1 LED_ENABLE=1 FUZZ_ENABLE=0 ADAPTIVE_ENABLE=1 ; Setup KAMP
    BED_MESH_CALIBRATE ADAPTIVE=1 ; Calibrate bed mesh
    Smart_Park
    LINE_PURGE

[gcode_macro CANCEL_PRINT]
description: Cancels the print, parks the toolhead like PAUSE, and shuts down like END_PRINT
gcode:
    M117 Cancelling print
    # Park the toolhead like PAUSE
    {% set x_park = printer.toolhead.axis_maximum.x|float - 5.0 %}
    {% set y_park = printer.toolhead.axis_maximum.y|float - 5.0 %}
    {% set max_z = printer.toolhead.axis_maximum.z|float %}
    {% set act_z = printer.toolhead.position.z|float %}
    {% if act_z < (max_z - 2.0) %}
        {% set z_safe = 2.0 %}
    {% else %}
        {% set z_safe = max_z - act_z %}
    {% endif %}
    G91 ; Relative positioning
    {% if "xyz" in printer.toolhead.homed_axes %}
        G1 Z{z_safe} F900 ; Lift Z like PAUSE
        G90 ; Absolute positioning
        G1 X{x_park} Y{y_park} F6000 ; Move to park position like PAUSE
    {% else %}
        {action_respond_info("Printer not homed, skipping park")}
    {% endif %}
    # Shut down like END_PRINT
    M104 S0 ; Turn off extruder heater
    M140 S0 ; Turn off bed heater
    M106 S0 ; Turn off fan and reset override
    SET_GCODE_VARIABLE MACRO=SET_FAN_SPEED VARIABLE=fan_override VALUE="False" ; Disable fan override
    SET_LED LED=LED_Light WHITE=0.0 SYNC=0 TRANSMIT=1 ; Turn off LED
    M84 ; Disable motors
    BED_MESH_CLEAR ; Clear adaptive mesh
    CLEAR_PAUSE ; Clear any pause state
    SDCARD_RESET_FILE ; Reset SD print file (if using SD)
    M117 Print cancelled

[gcode_macro PAUSE]
description: Pause the actual running print
rename_existing: PAUSE_BASE
# change this if you need more or less extrusion
variable_extrude: 1.0
gcode:
    ##### read E from pause macro #####
    {% set E = printer["gcode_macro PAUSE"].extrude|float %}
    ##### set park positon for x and y #####
    # default is your max posion from your printer.cfg
    {% set x_park = printer.toolhead.axis_maximum.x|float - 5.0 %}
    {% set y_park = printer.toolhead.axis_maximum.y|float - 5.0 %}
    ##### calculate save lift position #####
    {% set max_z = printer.toolhead.axis_maximum.z|float %}
    {% set act_z = printer.toolhead.position.z|float %}
    {% if act_z < (max_z - 2.0) %}
        {% set z_safe = 2.0 %}
    {% else %}
        {% set z_safe = max_z - act_z %}
    {% endif %}
    ##### end of definitions #####
    PAUSE_BASE
    G91
    {% if printer.extruder.can_extrude|lower == 'true' %}
      G1 E-{E} F2100
    {% else %}
      {action_respond_info("Extruder not hot enough")}
    {% endif %}
    {% if "xyz" in printer.toolhead.homed_axes %}
      G1 Z{z_safe} F900
      G90
      G1 X{x_park} Y{y_park} F6000
    {% else %}
      {action_respond_info("Printer not homed")}
    {% endif %}

[gcode_macro RESUME]
description: Resume the actual running print
rename_existing: RESUME_BASE
gcode:
    ##### read E from pause macro #####
    {% set E = printer["gcode_macro PAUSE"].extrude|float %}
    #### get VELOCITY parameter if specified ####
    {% if 'VELOCITY' in params|upper %}
      {% set get_params = ('VELOCITY=' + params.VELOCITY) %}
    {%else %}
      {% set get_params = "" %}
    {% endif %}
    ##### end of definitions #####
    {% if printer.extruder.can_extrude|lower == 'true' %}
      G91
      G1 E{E} F2100
    {% else %}
      {action_respond_info("Extruder not hot enough")}
    {% endif %}
    RESUME_BASE {get_params}

[gcode_macro END_PRINT]
gcode:
    M104 S0 ; Turn off extruder heater
    M140 S0 ; Turn off bed heater
    M106 S0 ; Turn off fan and reset override
    SET_GCODE_VARIABLE MACRO=SET_FAN_SPEED VARIABLE=fan_override VALUE="False" ; Disable fan override
    G91 ; Relative positioning
    G1 Z10 F300 ; Move Z axis up 10mm to avoid collision
    G90 ; Absolute positioning
    G1 Y200 F3000 ; Move Y axis forward to deliver the print
    SET_LED LED=LED_Light WHITE=0.0 SYNC=0 TRANSMIT=1
    M84 ; Disable motors

[homing_override]
gcode:
    G28 X Y ; Home X and Y axes to endstops
    G90 ; Absolute positioning
    G0 X241 Y193 F6000 ; Move to safe Z homing position at 100 mm/s
    G28 Z ; Home Z axis with probe
    G91 ; Relative positioning
    G0 Z10 F900 ; Z-hop 10 mm after homing (matches safe_z_home z_hop: 10)
    G90 ; Back to absolute positioning
set_position_z: 0 ; Reset Z position to 0 after homing

[gcode_macro MANUAL_BED_TRAMMING]
description: Home and measure for manual leveling adjustments
gcode:
    G28
    SCREWS_TILT_CALCULATE
    G90
    G1 F3000 X213 Y213 Z25

[gcode_macro MANUAL_Z_OFFSET_ADJUST]
description: Manually adjust Z offset by moving to current offset and allowing incremental changes
gcode:
    # Get current Z offset from config (fallback to 0 if undefined)
    {% set current_z_offset = printer.configfile.config.probe.z_offset | float | default(0.0) %}
    {% set center_x = printer.toolhead.axis_maximum.x / 2 | float %} ; Center of X (215 mm)
    {% set center_y = printer.toolhead.axis_maximum.y / 2 | float %} ; Center of Y (215 mm)
    {% set travel_speed = printer.toolhead.max_velocity * 60 | float %} ; Travel speed (15000 mm/min)
    # Display current Z offset
    {action_respond_info("Current Z offset: %.3f mm" % current_z_offset)}
    # Auto-home all axes
    G28
    {action_respond_info("Printer homed.")}
    # Move to center of bed at current Z offset
    G90 ; Absolute positioning
    G0 X{center_x} Y{center_y} Z10 F{travel_speed} ; Start at Z:10 mm for safety
    G0 Z{current_z_offset} F900 ; Move to current Z offset at slower speed (15 mm/s)
    # Instructions for adjustment
    {action_respond_info("Nozzle is at current Z offset (%.3f mm)." % current_z_offset)}
    {action_respond_info("Use G0 Z+0.1 or G0 Z-0.1 to adjust (e.g., 0.1 mm steps).")}
    {action_respond_info("Use G0 Z+0.01 or G0 Z-0.01 for finer adjustments.")}
    {action_respond_info("When satisfied, note the final Z position and run SAVE_Z_OFFSET Z=<value>.")}

[gcode_macro CALIBRATE_SHAPER]
description: Run input shaper calibration for X and Y axes with limited frequency range
gcode:
    G28 ; Home all axes
    TEST_RESONANCES AXIS=X FREQ_START=20 FREQ_END=60 ; X-axis calibration
    TEST_RESONANCES AXIS=Y FREQ_START=20 FREQ_END=60 ; Y-axis calibration
    SAVE_CONFIG ; Save results to printer.cfg

#[led LED_Light]
#white_pin: PB9
#shutdown_speed: 1.0
#cycle_time:0.010

[printer]
kinematics: cartesian
max_velocity: 800
max_accel: 5000
#minimum_cruise_ratio: 0.5
max_z_velocity: 25
max_z_accel: 200
square_corner_velocity: 5
# Use those higher values just to configure Input Shaper
#max_accel: 10000
#max_accel_to_decel: 10000

[stepper_x]
step_pin: PD4
dir_pin: !PD3
enable_pin: !PD6
microsteps: 16 # Set via driver jumpers/SPI
rotation_distance: 40
endstop_pin: PC1 # Adapted from stock PA13
position_endstop: 0
position_min: 0
position_max: 430
homing_speed: 70

[stepper_y]
step_pin: PA15
dir_pin: !PA8
enable_pin: !PD1
microsteps: 16
rotation_distance: 40
endstop_pin: PC3 # Adapted from stock PB8
position_endstop: -6
position_min: -6
position_max: 430
homing_speed: 40

[stepper_z]
step_pin: PE2
dir_pin: PE3
enable_pin: !PE0
rotation_distance: 8.0
microsteps: 16
position_min: -4.0
position_max: 506
endstop_pin: probe:z_virtual_endstop
homing_speed: 10
homing_retract_speed: 15

[extruder]
max_extrude_only_distance: 100.0
max_extrude_cross_section: 10
step_pin: PD15
dir_pin: !PD14
enable_pin: !PC7
microsteps: 16
nozzle_diameter: 0.600
filament_diameter: 1.750
heater_pin: PB3 # Adapted from stock PA6
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PA2 # Adapted from stock PC1
min_temp: 0
max_temp: 320
rotation_distance: 6.9
pressure_advance: 0.056
#control: pid
#pid_kp: 30.437
#pid_ki: 2.670
#pid_kd: 86.746 # From your saved config

[verify_heater extruder]
hysteresis: 5
max_error: 120
check_gain_time: 25

[firmware_retraction]
retract_length: 0.5
retract_speed: 35
unretract_extra_length: 0
unretract_speed: 35

[heater_bed]
heater_pin: PD7 # Adapted from stock PA5
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PA1 # Adapted from stock PC0
pwm_cycle_time: 0.020
max_temp: 110
min_temp: 0
control: pid
pid_kp: 71.562
pid_ki: 0.778
pid_kd: 1645.031

[heater_fan hotend_fan]
pin: PB5 # Adapted from stock PB0
heater: extruder
heater_temp: 50.0

[fan]
pin: PB6 # Adapted from stock PA7

[gcode_macro SET_FAN_SPEED]
description: Sets a manual fan speed with optional override persistence
variable_fan_speed: 0
variable_fan_override: False
gcode:
    {% set SPEED = params.SPEED|default(100)|float %}
    {% set OVERRIDE = params.OVERRIDE|default(0)|int %}
    {% if SPEED >= 0 and SPEED <= 100 %}
        {% set FAN_SPEED = (SPEED / 100.0) * 255|int %}
        M106 S{FAN_SPEED}
        G4 P100
        SET_GCODE_VARIABLE MACRO=SET_FAN_SPEED VARIABLE=fan_speed VALUE={FAN_SPEED}
        SET_GCODE_VARIABLE MACRO=SET_FAN_SPEED VARIABLE=fan_override VALUE={OVERRIDE > 0}
        M117 Fan speed set to {SPEED}% {", override " + ("enabled" if OVERRIDE > 0 else "disabled")}
    {% else %}
        M117 Invalid speed! Use 0-100.
    {% endif %}

[gcode_macro UPDATE_FAN_SPEED]
description: Internal macro to enforce fan speed override during print
gcode:
    {% if printer['gcode_macro SET_FAN_SPEED'].fan_override %}
        M106 S{printer['gcode_macro SET_FAN_SPEED'].fan_speed}
    {% endif %}

[force_move]
enable_force_move: True

[probe]
pin: ^PC0 # SKR Probe pin, adapted from stock ^PA8 (use pull-up if needed)
speed: 15
lift_speed: 40
samples: 3
x_offset: -28.5
y_offset: 22
#z_offset: 1.620 # From your saved config

[filament_switch_sensor filament_sensor]
pause_on_runout: true
switch_pin: PC2 # SKR Fil-DET pin, adapted from stock PB4

[bed_mesh]
probe_count: 11,9
algorithm: bicubic
speed: 350
horizontal_move_z: 10
mesh_min: 33, 16
mesh_max: 397, 415
fade_start: 1.0
fade_end: 10.0

[temperature_sensor raspberry_pi]
sensor_type: temperature_host

[temperature_sensor mcu_temp]
sensor_type: temperature_mcu
#sensor_temperature1: 25
#sensor_adc1: 0.210317

[gcode_macro M420]
description: Load the current mesh
gcode:
    BED_MESH_PROFILE LOAD=default

[gcode_macro G29]
description: creates automated homing and bed mesh
gcode:
    G28
    BED_MESH_CALIBRATE
    DATA_SAVE # Assuming this is a custom save macro; adjust if needed

[screws_tilt_adjust]
screw_thread: CW-M3
speed: 200
screw1: 243.5, 193
screw1_name: center
screw2: 421, 370.5
screw2_name: right back screw
screw3: 421, 193
screw3_name: right middle screw
screw4: 421, 15.5
screw4_name: right front screw
screw5: 66, 15.5
screw5_name: left front screw
screw6: 66, 193
screw6_name: left middle screw
screw7: 66, 370.5
screw7_name: left back screw

[mcu CB1] # Keep if using CB1/RPi for host
serial: /tmp/klipper_host_mcu

[adxl345]
cs_pin: CB1:None
spi_bus: spidev1.1
spi_speed: 5000000
axes_map: z,y,-x

[resonance_tester]
accel_chip: adxl345
probe_points:
    215, 215, 20
accel_per_hz: 50
max_smoothing: 0.2

[input_shaper]
#shaper_type_y: ei
#shaper_freq_y: 40.2
#shaper_type_x: ei
#shaper_freq_x: 59.8 # From your saved config

[gcode_macro BED_PID_TUNE]
description: Run PID tuning for the heated bed at a specified temperature. Usage: BED_PID_TUNE TEMP=<value>
gcode:
    {% set TEMP = params.TEMP|default(60)|float %}
    {% if TEMP >= 0 and TEMP <= 110 %}
        M117 Running Bed PID Tuning at {TEMP}°C
        G28 ; Home all axes before tuning
        PID_CALIBRATE HEATER=heater_bed TARGET={TEMP}
        M117 Bed PID Tuning Complete! Check terminal for values.
        SAVE_CONFIG
    {% else %}
        M117 Error: TEMP must be between 0 and 110°C
        {action_respond_info("Invalid temperature! Use TEMP between 0 and 110°C.")}
    {% endif %}

[gcode_macro NOZZLE_PID_TUNE]
description: Run PID tuning for the extruder at a specified temperature and fan speed. Usage: NOZZLE_PID_TUNE TEMP=<value> FAN_SPEED=<0-100>
gcode:
    {% set TEMP = params.TEMP|default(200)|float %}
    {% set FAN_SPEED = params.FAN_SPEED|default(0)|float %}
    {% if TEMP >= 0 and TEMP <= 290 %}
        {% if FAN_SPEED >= 0 and FAN_SPEED <= 100 %}
            M117 Running Nozzle PID Tuning at {TEMP}°C with fan at {FAN_SPEED}%
            G28 ; Home all axes before tuning
            SET_HEATER_FAN_SPEED HEATER_FAN=hotend_fan VALUE=0 ; Disable hotend fan
            SET_FAN_SPEED FAN=fan SPEED={FAN_SPEED} ; Set part cooling fan
            PID_CALIBRATE HEATER=extruder TARGET={TEMP}
            M117 Nozzle PID Tuning Complete! Check terminal for values.
            SET_FAN_SPEED FAN=fan SPEED=0 ; Reset part cooling fan
            SET_HEATER_FAN_SPEED HEATER_FAN=hotend_fan VALUE=255 ; Restore hotend fan
            SAVE_CONFIG
        {% else %}
            M117 Error: FAN_SPEED must be between 0 and 100%
            {action_respond_info("Invalid fan speed! Use FAN_SPEED between 0 and 100%.")}
        {% endif %}
    {% else %}
        M117 Error: TEMP must be between 0 and 290°C
        {action_respond_info("Invalid temperature! Use TEMP between 0 and 290°C.")}
    {% endif %}

[gcode_arcs]
resolution: 0.5

########################################
# EXP1 / EXP2 (display) pins
########################################
[board_pins]
aliases:
    # EXP1 header
    EXP1_1=PC5, EXP1_3=PB1, EXP1_5=PE9, EXP1_7=PE11, EXP1_9=<GND>,
    EXP1_2=PB0, EXP1_4=PE8, EXP1_6=PE10, EXP1_8=PE12, EXP1_10=<5V>,
    # EXP2 header
    EXP2_1=PA6, EXP2_3=PE7, EXP2_5=PB2, EXP2_7=PC4, EXP2_9=<GND>,
    EXP2_2=PA5, EXP2_4=PA4, EXP2_6=PA7, EXP2_8=<RST>, EXP2_10=<NC>
# See the sample-lcd.cfg file for definitions of common LCD displays.

########################################
# TMC5160 configuration (SPI mode)
########################################
[tmc5160 stepper_x]
cs_pin: PD5
spi_software_miso_pin: PE15
spi_software_mosi_pin: PE13
spi_software_sclk_pin: PE14
run_current: 1.0
hold_current: 0.5
sense_resistor: 0.075
interpolate: true
stealthchop_threshold: 999999 # Disable; tune if needed

[tmc5160 stepper_y]
cs_pin: PD0
spi_software_miso_pin: PE15
spi_software_mosi_pin: PE13
spi_software_sclk_pin: PE14
run_current: 1.6
hold_current: 0.7
sense_resistor: 0.075
interpolate: true
stealthchop_threshold: 999999

[tmc5160 stepper_z]
cs_pin: PE1
spi_software_miso_pin: PE15
spi_software_mosi_pin: PE13
spi_software_sclk_pin: PE14
run_current: 0.8
hold_current: 0.5
sense_resistor: 0.075
interpolate: true
stealthchop_threshold: 999999

[tmc5160 extruder]
cs_pin: PC6
spi_software_miso_pin: PE15
spi_software_mosi_pin: PE13
spi_software_sclk_pin: PE14
run_current: 0.8
hold_current: 0.4
sense_resistor: 0.075
interpolate: true
stealthchop_threshold: 999999

#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [probe]
#*# z_offset = 1.420
#*#
#*# [input_shaper]
#*# shaper_type_y = ei
#*# shaper_freq_y = 40.2
#*# shaper_type_x = ei
#*# shaper_freq_x = 50.0
#*#
#*# [extruder]
#*# control = pid
#*# pid_kp = 32.771
#*# pid_ki = 3.361
#*# pid_kd = 79.880
#*#
#*# [bed_mesh default]
#*# version = 1
#*# points =
#*# -0.007500, 0.033333, 0.002500, 0.036667
#*# -0.004167, 0.022500, 0.000833, 0.045000
#*# -0.024167, -0.000833, 0.001667, 0.008333
#*# x_count = 4
#*# y_count = 3
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = lagrange
#*# tension = 0.2
#*# min_x = 170.0
#*# max_x = 249.98000000000002
#*# min_y = 170.0
#*# max_y = 250.0
