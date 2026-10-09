import json
from pathlib import Path
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from evidence_writer import JsonlEvidence, audit_jsonl


class EvidenceWriterTests(unittest.TestCase):
    def test_concurrent_recording_preserves_every_row_and_close_fences_callbacks(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'records.jsonl'
            recorder = JsonlEvidence(path)
            def worker(owner):
                for seq in range(500):
                    self.assertTrue(recorder.write(dict(owner=owner, seq=seq, text='证据'*100)))
            with ThreadPoolExecutor(max_workers=8) as pool:
                list(pool.map(worker, range(8)))
            recorder.close()
            self.assertFalse(recorder.write(dict(late=True)))
            recorder.close()
            report = audit_jsonl([path])
            self.assertTrue(report['valid'])
            self.assertEqual(report['parsed_rows'][str(path)], 4000)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len({(r['owner'],r['seq']) for r in rows}), 4000)

    def test_truncated_null_and_nonfinite_records_cannot_pass_audit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'records.jsonl'
            path.write_bytes(b'{}\n{\n\x00\n{"x":NaN}\n')
            result = audit_jsonl([path])
            self.assertFalse(result['valid'])
            self.assertEqual(len(result['errors']), 3)

    def test_missing_file_is_missing_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertFalse(audit_jsonl([Path(directory)/'missing'])['valid'])
