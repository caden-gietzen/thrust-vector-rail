# Encoder Module

Custom Raspberry Pi Pico MicroPython firmware module for reading a quadrature encoder using Programmable Input/Output and C-level firmware support.

This module was developed after MicroPython-side encoder handling proved unreliable for high-rate quadrature signals. The current architecture uses Programmable Input/Output for low-level quadrature handling, a C module for count access and zeroing, and MicroPython only as the high-level control layer.

Current status: initial firmware files added. Full documentation and cleanup pending.