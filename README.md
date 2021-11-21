# Freight Frenzy Common

Retain the files updated for the common autonomous program for the 2021 First Tech Challange Freight Frenzy.

- This version is updated to use actual measurements for movements instead of motor sensor pulses.
- It has be highly structured to simplify code verification and resuse.
- It uses just a few variables to determin the start position and alliance.
- It uses a sequence of calls to methods to specifiy when the robot goes to different locations.

Files changed:
- _Auto_runs.java
- AutoCommon.java
- AutoHardware.java

Future change potential
- Ramp speed up and down
- Use sensors to fine tune location
- Provide a method to go diagonally to a new location (the current version runs strictly on an x/y coordinate system)
