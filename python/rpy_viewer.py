#!/usr/bin/env python3
"""Live 3D body-frame viewer for ESP32 roll,pitch,yaw CSV serial data."""

import argparse
import math
from typing import Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


def parse_rpy_line(line: str) -> Optional[Tuple[float, float, float]]:
    """Parse one 'roll,pitch,yaw' line in degrees; ignore all other lines."""
    parts = line.strip().split(",")
    if len(parts) != 3:
        return None
    try:
        values = tuple(float(p) for p in parts)
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in values):
        return None
    return values  # type: ignore[return-value]


def rotation_matrix_rpy(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Body-to-world rotation R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])

    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


class OrientationViewer:
    def __init__(self, port: str, baud: int = 115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.rpy = (0.0, 0.0, 0.0)

        self.fig = plt.figure(figsize=(8, 7))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.fig.canvas.manager.set_window_title("ESP32-S3-Matrix Madgwick Orientation")

    def _read_latest(self) -> None:
        while self.ser.in_waiting:
            raw = self.ser.readline().decode("utf-8", errors="ignore")
            parsed = parse_rpy_line(raw)
            if parsed is not None:
                self.rpy = parsed

    def _draw(self, _frame):
        self._read_latest()
        roll, pitch, yaw = self.rpy
        R = rotation_matrix_rpy(roll, pitch, yaw)

        self.ax.cla()
        self.ax.set_xlim(-1.2, 1.2)
        self.ax.set_ylim(-1.2, 1.2)
        self.ax.set_zlim(-1.2, 1.2)
        self.ax.set_box_aspect((1, 1, 1))
        self.ax.set_xlabel("World X")
        self.ax.set_ylabel("World Y")
        self.ax.set_zlabel("World Z")
        self.ax.set_title(
            f"Roll {roll:7.2f}°   Pitch {pitch:7.2f}°   "
            f"Yaw {yaw:7.2f}° (relative)"
        )

        # Faint world-frame reference axes.
        self.ax.quiver(0, 0, 0, 1.0, 0, 0, linewidth=1, alpha=0.25)
        self.ax.quiver(0, 0, 0, 0, 1.0, 0, linewidth=1, alpha=0.25)
        self.ax.quiver(0, 0, 0, 0, 0, 1.0, linewidth=1, alpha=0.25)

        # Rotating body-frame triad. A triad, rather than one arrow, makes yaw visible.
        ex = R @ np.array([1.0, 0.0, 0.0])
        ey = R @ np.array([0.0, 1.0, 0.0])
        ez = R @ np.array([0.0, 0.0, 1.0])

        self.ax.quiver(0, 0, 0, *ex, linewidth=3, label="Body X")
        self.ax.quiver(0, 0, 0, *ey, linewidth=3, label="Body Y")
        self.ax.quiver(0, 0, 0, *ez, linewidth=3, label="Body Z")

        self.ax.text(*(1.08 * ex), "Xb")
        self.ax.text(*(1.08 * ey), "Yb")
        self.ax.text(*(1.08 * ez), "Zb")
        return ()

    def run(self) -> None:
        self.animation = FuncAnimation(self.fig, self._draw, interval=30, blit=False)
        try:
            plt.show()
        finally:
            self.ser.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Render ESP32 roll,pitch,yaw serial data as a live 3D body-frame triad."
    )
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyACM0 or COM5")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    OrientationViewer(args.port, args.baud).run()


if __name__ == "__main__":
    main()
