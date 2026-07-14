"""Chassis manual control (teleop jog) via the unitree SDK binary.

Mirrors the proven control path used by the standalone g1d_remote_control_server
(:18086): it drives the base by launching `g1d_simple_control <iface> <action>
<speed> <duration>` as a long-lived process while a button is held, and stops by
terminating that process + issuing an explicit `stop`.
"""
from .jog import JogController

__all__ = ["JogController"]
