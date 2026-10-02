from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/robocup_swarm/scripts'))
from publisher_authority import PublisherAuthority


@unittest.skipIf(sys.platform == 'win32', 'POSIX flight runtime')
class AuthorityLockTests(unittest.TestCase):
    def test_duplicate_model_cannot_take_authority_and_release_is_reusable(self):
        with tempfile.TemporaryDirectory() as directory:
            with PublisherAuthority('typhoon_h480_0', directory):
                with self.assertRaisesRegex(RuntimeError, 'ALREADY_HELD'):
                    with PublisherAuthority('typhoon_h480_0', directory):
                        pass
            with PublisherAuthority('typhoon_h480_0', directory):
                pass

    def test_peer_models_have_separate_authorities(self):
        with tempfile.TemporaryDirectory() as directory:
            with PublisherAuthority('typhoon_h480_0', directory):
                with PublisherAuthority('typhoon_h480_1', directory):
                    pass
