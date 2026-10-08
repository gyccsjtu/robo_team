"""Persistent local XY/XYZ stopping anchor; transport-free setpoint fields."""
import math


class PositionBrake:
    def __init__(self):
        self.anchor = None
        self.transform = None
        self.altitude = None

    def encode(self, velocity, yaw_rate, position, transform, hold_altitude=None,
               moving_altitude=None):
        if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity) or not math.isfinite(yaw_rate):
            raise ValueError('NONFINITE_SETPOINT')
        moving = math.hypot(*velocity[:2]) > 1e-9
        if self.transform != transform:
            self.anchor = None
            self.altitude = None
        if moving:
            self.anchor = None
            self.altitude = None
            self.transform = transform
        elif position is not None:
            if len(position) != 2 or not all(math.isfinite(v) for v in position):
                raise ValueError('INVALID_STOP_POSE')
            if self.anchor is None or self.transform != transform:
                self.anchor = tuple(position)
                self.transform = transform
        holding = not moving and self.anchor is not None
        if hold_altitude is None or not holding or position is None or abs(velocity[2]) > 1e-9:
            self.altitude = None
        elif not math.isfinite(hold_altitude):
            raise ValueError('INVALID_STOP_ALTITUDE')
        elif self.altitude is None:
            self.altitude = hold_altitude
        holding_xyz = holding and self.altitude is not None
        # Explicit vertical velocity (takeoff, landing, height emergency) wins.
        # Moving XY with a requested Z position ignores VZ, not VX/VY.
        moving_z = (moving and moving_altitude is not None and position is not None
                    and abs(velocity[2]) <= 1e-9)
        if moving_z and not math.isfinite(moving_altitude):
            raise ValueError('INVALID_MOVING_ALTITUDE')
        return dict(coordinate_frame=1, type_mask=1528 if holding_xyz else 1500 if holding else 1507 if moving_z else 1479,
                    position_xy=self.anchor if holding else (0., 0.),
                    position_z=self.altitude if holding_xyz else moving_altitude if moving_z else 0.,
                    velocity_xyz=(0.,0.,0.) if holding_xyz else tuple(velocity), yaw_rate=yaw_rate)
