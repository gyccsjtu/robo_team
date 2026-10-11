"""Auxiliary camera bearings and finite handoff; no actor coordinates or credit.

The native PX4 Gazebo gimbal sends message 285 with simulation milliseconds.
Only measured attitudes on that clock are accepted. Commands are never poses.
MAVLink frames: https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS
"""
import bisect
import math
import struct
import numpy as np

COLORS = {'red', 'blue', 'green', 'white', 'brown'}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def selected_uavs(raw):
    ids = [int(v.strip()) for v in raw.split(',')]
    if not ids or len(ids) != len(set(ids)) or any(i not in range(6) for i in ids):
        raise ValueError('gimbal UAV IDs must be distinct integers 0..5')
    return ['typhoon_h480_%d' % i for i in ids]


def quat_matrix(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or not .9 <= np.dot(q,q) <= 1.1:
        raise ValueError('invalid measured quaternion')
    x,y,z,w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def measured_packet(msg, now):
    if msg.msgid != 285 or msg.framing_status != 1 or not 37 <= msg.len <= 64:
        return None
    if msg.len > len(msg.payload64)*8:
        return None
    payload = struct.pack('<%dQ' % len(msg.payload64), *msg.payload64)[:msg.len]
    values = struct.unpack('<I4f3fIHBB', payload[:40].ljust(40,b'\0'))
    stamp, w,x,y,z = values[:5]
    stamp /= 1000.
    failure, flags = values[8:10]
    # Native roll/pitch stabilized FRD; reject unsupported modern reference flags.
    if failure or flags & ~28 or flags & 12 != 12 or not 0. <= now-stamp <= .3:
        return None
    try:
        quat_matrix((x,y,z,w))
    except ValueError:
        return None
    return stamp, (x,y,z,w), flags


class AttitudeHistory:
    def __init__(self):
        self.rows = []

    def add(self, stamp, q, flags=0):
        quat_matrix(q)
        if not math.isfinite(stamp) or stamp <= 0.:
            return
        if self.rows and stamp <= self.rows[-1][0]:
            return
        self.rows = [r for r in self.rows if stamp-r[0] <= 2.][-119:]
        self.rows.append((stamp, tuple(q), flags))

    def at(self, stamp):
        i = bisect.bisect_left([r[0] for r in self.rows], stamp)
        if i < len(self.rows) and self.rows[i][0] == stamp:
            return self.rows[i][1:]
        if not 0 < i < len(self.rows):
            return None
        a,b = self.rows[i-1:i+1]
        if b[0]-a[0] > .15 or a[2] != b[2]:
            return None
        q0,q1 = np.array(a[1]),np.array(b[1])
        if np.dot(q0,q1) < 0: q1 = -q1
        q = q0+(q1-q0)*((stamp-a[0])/(b[0]-a[0]))
        q /= np.linalg.norm(q)
        return tuple(q), a[2]


def camera_rotation(gimbal, body):
    q,flags = gimbal
    rn = quat_matrix(q)
    if not flags & 16:
        rb = quat_matrix(body[0])
        heading = math.pi/2-math.atan2(rb[1,0],rb[0,0])
        c,s = math.cos(heading),math.sin(heading)
        rn = np.array([[c,-s,0],[s,c,0],[0,0,1]]) @ rn
    return np.array([[0,1,0],[1,0,0],[0,0,-1]]) @ rn @ np.diag([1,-1,-1])


def image_bearing(box, calibration, rotation):
    width,height,K,D = calibration
    if len(K) != 9 or not all(math.isfinite(v) for v in (*K,*D)) or any(abs(v)>1e-9 for v in D):
        raise ValueError('requires calibrated native rectilinear CGO3 camera')
    x1,y1,x2,y2 = box
    if (not all(math.isfinite(v) for v in box) or
            not 1 <= x1 < x2 <= width-1 or not 1 <= y1 < y2 <= height-1 or
            x2-x1 < 4 or y2-y1 < 10 or not .9 <= (y2-y1)/(x2-x1) <= 6. or
            K[0] <= 0 or K[4] <= 0):
        raise ValueError('unusable auxiliary image box')
    # Native camera link is FLU, optical is right/down/forward.
    ray = rotation @ np.array([1., -((x1+x2)/2-K[2])/K[0],
                              -((y1+y2)/2-K[5])/K[4]])
    horizontal = math.hypot(ray[0],ray[1])
    if horizontal < .3:
        raise ValueError('near-vertical bearing')
    return math.atan2(ray[1],ray[0]),math.atan2(ray[2],horizontal)


class AuxiliaryTracks:
    """Fresh YOLO frames only; ambiguous equal-colour associations get no lease."""
    def __init__(self, session):
        self.session,self.next_id,self.rows,self.last_s = session,0,{},-1.

    def update(self, stamp, detections):
        if stamp <= self.last_s:
            return []
        self.last_s = stamp
        old = [v for v in self.rows.values() if 0 < stamp-v['sample_s'] <= .7]
        choices = [[v for v in old if v['color']==d['color'] and
                    abs(wrap(v['bearing']-d['bearing'])) <= .16 and
                    abs(v['elevation']-d['elevation']) <= .14] for d in detections]
        rows = {}
        for index,d in enumerate(detections):
            if d['color'] not in COLORS or d['confidence'] < .55:
                continue
            same = [other for j,other in enumerate(detections) if j != index and
                    other['color']==d['color'] and
                    abs(wrap(other['bearing']-d['bearing'])) < .24]
            if same:
                continue
            matches = choices[index]
            prior = (matches[0] if len(matches)==1 and
                     sum(any(v['candidate_id']==matches[0]['candidate_id'] for v in c)
                         for c in choices)==1 else None)
            if prior is None:
                self.next_id += 1
                cid = '%s:%d' % (self.session,self.next_id)
            else:
                cid = prior['candidate_id']
            rows[cid] = dict(d,candidate_id=cid,sample_s=stamp,
                             first_s=prior['first_s'] if prior else stamp,
                             hits=prior['hits']+1 if prior else 1)
        self.rows = rows
        return [v for v in rows.values() if v['hits'] >= 3 and stamp-v['first_s'] >= .4-1e-6]


class ScanAndHandoff:
    def __init__(self):
        self.active=None
        self.cooldowns=[]
        self.scan_started=None

    def offer(self, rows, now, origin):
        self.cooldowns=[r for r in self.cooldowns if now < r['until']]
        if self.active:
            for row in rows:
                if row['candidate_id']==self.active['candidate_id']:
                    self.active.update(row)
                    break
            return
        usable=[r for r in rows if not any(
            r['color']==c['color'] and math.dist(origin,c['origin']) < 3. and
            abs(wrap(r['bearing']-c['bearing'])) < .25 for c in self.cooldowns)]
        if usable:
            row=max(usable,key=lambda r:(r['hits'],r['confidence']))
            self.active=dict(row,started_s=now,until_s=now+6.,origin=tuple(origin))

    def finish(self, now):
        if self.active:
            self.cooldowns.append(dict(self.active,until=now+12.))
        self.active=None

    def command(self, now, body_yaw, available):
        if not available:
            self.finish(now)
            return None
        a=self.active
        if a and (now >= a['until_s'] or not 0 <= now-a['sample_s'] <= .65):
            self.finish(now)
            a=None
        if a:
            relative=wrap(a['bearing']-body_yaw)
            if abs(relative) > math.radians(80.):
                self.finish(now)
                return None
            # MountControl yaw is clockwise/FRD relative to vehicle heading.
            return max(-45.,min(-8.,math.degrees(a['elevation']))), -math.degrees(relative)
        if self.scan_started is None:self.scan_started=now
        # Finite +/-45 degree sweep, 20 deg/s, 0.8 s dwell at each end.
        phase=(now-self.scan_started)%10.6
        yaw=(-45.+20.*phase if phase<4.5 else
             45. if phase<5.3 else 45.-20.*(phase-5.3) if phase<9.8 else -45.)
        return -15.,yaw


def validate_handoff(data, uid, now):
    if (data.get('purpose')!='gimbal_handoff' or data.get('camera')!='cgo3' or
            data.get('source_uav')!=uid or data.get('real_detection') is not True or
            data.get('official_view_ok') is not False or data.get('mode')!='handoff' or
            data.get('attitude_source')!='mavlink_285' or data.get('color') not in COLORS or
            not isinstance(data.get('candidate_id'),str) or
            not 0 < len(data['candidate_id']) <= 128 or
            any(k in data for k in ('x','y','xyz','target_id'))):
        raise ValueError('invalid gimbal attention schema')
    vals={k:float(data[k]) for k in ('sample_s','first_s','started_s','until_s',
                                  'bearing','confidence','attitude_sample_s')}
    if (not all(math.isfinite(v) for v in vals.values()) or
            not 0. <= now-vals['sample_s'] <= .35 or
            abs(vals['sample_s']-vals['attitude_sample_s']) > 1e-6 or
            not vals['first_s'] <= vals['sample_s'] or
            vals['sample_s']-vals['first_s'] < .4-1e-6 or
            not vals['started_s'] <= now < vals['until_s'] <= vals['started_s']+6.+1e-6 or
            int(data['hits']) < 3 or not .55 <= vals['confidence'] <= 1. or
            abs(vals['bearing']) > math.pi):
        raise ValueError('stale or unqualified gimbal attention')
    return dict(data,**vals)
