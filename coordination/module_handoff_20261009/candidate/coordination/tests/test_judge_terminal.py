import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from judge_terminal import write_terminal,completed


class JudgeTerminalTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path=Path(self.directory.name)/'terminal.json'
        self.sha=hashlib.sha256(b'executed judge source').hexdigest()
        self.state=dict(mission_finished=True,left_actors=[],target_finish=6,score=2169.432)

    def test_final_deletion_is_complete_even_with_stale_old_topic(self):
        write_terminal(self.path,'current',self.sha,self.state)
        self.assertEqual(completed(self.path,'current',self.sha,0)['target_finish'],6)

    def test_normal_exit_without_terminal_is_not_success(self):
        self.assertIsNone(completed(self.path,'current',self.sha,0))

    def test_wrong_run_or_source_cannot_supply_completion(self):
        write_terminal(self.path,'current',self.sha,self.state)
        self.assertIsNone(completed(self.path,'old',self.sha,0))
        self.assertIsNone(completed(self.path,'current','other-source',0))

    def test_live_or_crashed_judge_cannot_supply_completion(self):
        write_terminal(self.path,'current',self.sha,self.state)
        for code in [None,1,-9,False]:self.assertIsNone(completed(self.path,'current',self.sha,code))

    def test_timeout_or_incomplete_target_list_is_not_success(self):
        for changes in [dict(mission_finished=False),dict(left_actors=[1]),dict(target_finish=5)]:
            value=dict(schema_version=1,run_id='current',judge_sha256=self.sha,**self.state)
            value.update(changes);self.path.write_text(json.dumps(value))
            self.assertIsNone(completed(self.path,'current',self.sha,0))

    def test_zero_or_invalid_score_cannot_claim_completion(self):
        for score in [0,-1,float('nan'),float('inf'),True]:
            value=dict(schema_version=1,run_id='current',judge_sha256=self.sha,**self.state)
            value['score']=score;self.path.write_text(json.dumps(value))
            self.assertIsNone(completed(self.path,'current',self.sha,0))

    def test_partial_or_malformed_file_is_not_completion(self):
        for text in ['{','null','[]','{}']:
            self.path.write_text(text)
            self.assertIsNone(completed(self.path,'current',self.sha,0))

    def test_writing_nan_cannot_publish_terminal(self):
        self.state['score']=float('nan')
        with self.assertRaises(ValueError):write_terminal(self.path,'current',self.sha,self.state)
        self.assertFalse(self.path.exists())
