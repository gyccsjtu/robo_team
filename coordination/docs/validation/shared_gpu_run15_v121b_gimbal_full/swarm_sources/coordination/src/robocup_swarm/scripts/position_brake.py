"""Persistent local XY stopping anchor; transport-free ROS setpoint fields."""
import math


class PositionBrake:
    def __init__(self):
        self.anchor = None
        self.transform = None

    def encode(self, velocity, yaw_rate, position, transform):
        if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity) or not math.isfinite(yaw_rate):
            raise ValueError('NONFINITE_SETPOINT')
        moving = math.hypot(*velocity[:2]) > 1e-9
        if self.transform != transform:
            self.anchor = None
        if moving:
            self.anchor = None
            self.transform = transform
        elif position is not None:
            if len(position) != 2 or not all(math.isfinite(v) for v in position):
                raise ValueError('INVALID_STOP_POSE')
            if self.anchor is None or self.transform != transform:
                self.anchor = tuple(position)
                self.transform = transform
        holding = not moving and self.anchor is not None
        return dict(coordinate_frame=1, type_mask=1500 if holding else 1479,
                    position_xy=self.anchor if holding else (0., 0.),
                    velocity_xyz=tuple(velocity), yaw_rate=yaw_rate)
