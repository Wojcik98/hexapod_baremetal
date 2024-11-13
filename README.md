# Bare-metal hexapod control

This repository contains code for the control of a hexapod robot.
It is using the NUCLEO-F446RE board.

Currently, only tripod gait is implemented.
It accepts twist commands $(v_x, v_y, \omega_z)$, and all can be non-zero as the robot is holonomic.

It can be remotely controlled using [Wojcik98/hex_remote](https://github.com/Wojcik98/hex_remote).
