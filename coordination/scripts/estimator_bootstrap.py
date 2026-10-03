"""Development SITL estimator selection, applied before estimator startup."""
SETTINGS={'EKF2_MULTI_IMU':(3,0),'SENS_IMU_MODE':(0,1),
          'EKF2_MULTI_MAG':(2,0),'SENS_MAG_MODE':(0,1)}


def single_estimator(text):
    for name,(before,after) in SETTINGS.items():
        original='param set-default %s %s'%(name,before)
        if text.count(original)!=1:raise ValueError('UNSUPPORTED_ESTIMATOR_BOOTSTRAP:'+name)
        text=text.replace(original,'param set-default %s %s'%(name,after),1)
    return text
