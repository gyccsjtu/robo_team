"""Pinned hashes must not depend on a stale local remote-tracking branch."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from tools import official_reference as ref
from tools import check_official_map as maps


class OfficialReferenceTests(unittest.TestCase):
    def test_valid_working_bytes_are_usable_without_git_object(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'robocup').mkdir()
            (root/'robocup/map_generator.py').write_bytes(b'official')
            spec = dict(commit='a'*40, sha256={'map_generator.py':ref.digest(b'official')})
            with patch.object(ref, 'contract', return_value=spec), \
                    patch.object(maps, 'SOURCE_FILES', ('map_generator.py',)), \
                    patch.object(maps.subprocess, 'run', side_effect=AssertionError('no Git required')):
                self.assertEqual(maps.load_reference(root, 'a'*40), {'map_generator.py':b'official'})

    def test_cached_reference_does_not_excuse_modified_live_actor(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'robocup').mkdir()
            (root/'cache').mkdir()
            (root/'robocup/control_actor.py').write_bytes(b'changed')
            (root/'cache/control_actor.py').write_bytes(b'official')
            spec = dict(commit='a'*40, sha256={'control_actor.py':ref.digest(b'official')})
            with patch.object(ref, 'contract', return_value=spec), \
                    patch.object(ref, 'cache_dir', return_value=root/'cache'):
                self.assertEqual(ref.pinned_blobs(root, ['control_actor.py']), {'control_actor.py':b'official'})
                self.assertEqual(ref.check_local(root, ['control_actor.py']), ['control_actor.py'])

    def test_corrupt_cache_and_unknown_working_bytes_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'map_generator.py').write_bytes(b'changed')
            spec = dict(commit='a'*40, sha256={'map_generator.py':ref.digest(b'official')})
            with patch.object(ref, 'contract', return_value=spec), \
                    patch.object(ref, 'cache_dir', return_value=root):
                with self.assertRaises(ValueError):
                    ref.pinned_blobs(root, ['map_generator.py'])


if __name__ == '__main__':
    unittest.main()
