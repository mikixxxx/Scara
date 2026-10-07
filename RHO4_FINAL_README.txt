SCARA / RHO4 FINAL STACK
=========================

PC
--
rho4_driver.py
rho4_gcode_runner.py
rho4_test_final.gcode

Controller
----------
RHO4_FINAL_BAPS.zip

Final controller baseline:
- PERMPROG v8c
  - probe state via input 801
  - spindle output on X11 via OUTPUT BINARY:801
  - CMD34/35/36/38/39
  - existing HYRING/motion commands preserved
- PCPROBE v8b
  - MOVE LINEAR UNTIL fast input 611
  - @MPOS capture
  - WC(@MPOS)
  - normal PROGRAM_END

Physically verified:
- TCP/PERMPROG
- PTP / LINEAR / CIRCULAR
- HYRING mixed native streaming
- CAM G-code motion
- fast input
- G38.2 Z probe
- @MPOS trigger capture
- G92 Z from probe
- post-probe retract
- X11 physical output via BAPS 801

G-code I/O:
- M3 = spindle ON
- M5 = spindle OFF
- requires --arm-io
- Sxxxx is not yet physical speed control
- M4 not implemented
- G38.2 intentionally Z-only

Safe test:
python .\rho4_gcode_runner.py .\rho4_test_final.gcode --host 192.168.4.1 --run

This is the FINAL baseline for the current feature set.
Future spindle-speed control, additional I/O, or probing enhancements
should branch from this package.
