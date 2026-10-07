"""Observation-position guidance from fresh accepted visual motion only."""
import math


def companion_guidance(position, target, velocity, age_s, standoff_m=10.,
                       plan_distance_m=9., gain=.8, maximum_speed=2.6):
    values = tuple(position)+tuple(target)+tuple(velocity)+(age_s,standoff_m,plan_distance_m,gain,maximum_speed)
    if (not all(math.isfinite(v) for v in values) or not 0 <= age_s <= 1.5
            or min(standoff_m,plan_distance_m,gain,maximum_speed) <= 0):
        return None
    vx, vy = velocity
    speed = math.hypot(vx,vy)
    if speed < .15:
        vx = vy = speed = 0.
    if speed > maximum_speed:
        vx, vy = vx*maximum_speed/speed, vy*maximum_speed/speed
        speed = maximum_speed
    # Project for navigation; this never updates the original observation.
    tx, ty = target[0]+vx*age_s, target[1]+vy*age_s
    dx, dy = position[0]-tx, position[1]-ty
    distance = math.hypot(dx,dy)
    if distance < 1e-6:
        return None
    view = (tx+standoff_m*dx/distance, ty+standoff_m*dy/distance)
    command = (vx+gain*(view[0]-position[0]), vy+gain*(view[1]-position[1]))
    desired_speed = min(maximum_speed, math.hypot(*command))
    horizon = min(4.5,plan_distance_m/speed) if speed else 0.
    goal = (view[0]+vx*horizon,view[1]+vy*horizon)
    return dict(goal=goal,look_at=(tx,ty),speed_mps=desired_speed,
                observation_age_s=age_s,standoff_m=standoff_m)
