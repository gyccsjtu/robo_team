from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from estimator_bootstrap import single_estimator,SETTINGS


class EstimatorBootstrapTests(unittest.TestCase):
    def test_changes_only_four_startup_defaults(self):
        original='\n'.join('param set-default %s %s'%(name,v[0]) for name,v in SETTINGS.items())
        original='param set-default EKF2_REQ_GPS_H 0.5\n'+original+'\nstart estimator\n'
        result=single_estimator(original)
        for name,v in SETTINGS.items():self.assertIn('param set-default %s %s'%(name,v[1]),result)
        self.assertIn('param set-default EKF2_REQ_GPS_H 0.5',result)
        self.assertTrue(result.endswith('start estimator\n'))

    def test_missing_or_ambiguous_defaults_do_not_claim_configuration(self):
        for text in ('start estimator', 'param set-default EKF2_MULTI_IMU 3\n'*2):
            with self.assertRaisesRegex(ValueError,'UNSUPPORTED_ESTIMATOR_BOOTSTRAP'):
                single_estimator(text)
