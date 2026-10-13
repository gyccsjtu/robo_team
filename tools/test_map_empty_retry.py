"""Recover the recorded empty-world failure without altering official source."""
import json, unittest, re
from unittest.mock import patch
import test_prepare_official_map as fixture
prepare=fixture.prepare

class EmptyWorldRetryTests(unittest.TestCase):
    setUp=fixture.PrepareOfficialMapTests.setUp
    old_files_unchanged=fixture.PrepareOfficialMapTests.old_files_unchanged

    def generator(self,always=False,malformed=False):
        body=fixture.fake_generator().decode()
        line=next(s for s in body.splitlines() if s.startswith("Path(output_path, 'robocup.world').write_text("))
        if malformed:changed="Path(output_path, 'robocup.world').write_text('<sdf>')"
        elif always:changed="Path(output_path, 'robocup.world').write_text('')"
        else:
            changed="first = not Path('placement_attempt.marker').exists()\n"+line.replace('.write_text(',".write_text('' if first else ")+"\nPath('placement_attempt.marker').write_text('done')"
        return body.replace(line,changed).encode()

    def invoke(self,body):
        ref=dict(self.reference);ref['map_generator.py']=body
        with patch.object(prepare,'load_reference',return_value=ref),patch.object(prepare,'active_simulation_processes',return_value=[]):
            return prepare.prepare(self.xt,self.world,stage_parent=self.stage_parent)

    def test_first_empty_then_valid_installs_unmodified_official_generator(self):
        body=self.generator();stage,_=self.invoke(body)
        self.assertEqual((self.robocup/'map_generator.py').read_bytes(),body)
        attempts=json.loads((stage/'generation_attempts.json').read_text())
        self.assertEqual([r['valid'] for r in attempts],[False,True])
        self.assertTrue(attempts[0]['empty_world'])
        self.assertTrue((stage/'generator_attempt_01.log').exists())
        self.assertEqual(json.loads((stage/'manifest.json').read_text())['generation_attempts'],attempts)
        self.assertIn(b'<actor',self.world.read_bytes())

    def test_three_empty_outputs_stop_and_preserve_every_original(self):
        with self.assertRaisesRegex(RuntimeError,'连续3次产生空地图'):self.invoke(self.generator(always=True))
        self.old_files_unchanged()
        stage=next(self.stage_parent.iterdir());attempts=json.loads((stage/'generation_attempts.json').read_text())
        self.assertEqual(len(attempts),3);self.assertTrue(all(not r['valid'] for r in attempts))

    def test_invalid_nonempty_xml_is_not_retried(self):
        with self.assertRaises(RuntimeError):self.invoke(self.generator(malformed=True))
        self.old_files_unchanged();stage=next(self.stage_parent.iterdir())
        attempts=json.loads((stage/'generation_attempts.json').read_text());self.assertEqual(len(attempts),1)
        self.assertFalse(attempts[0]['empty_world'])

    def test_command_failure_is_not_retried(self):
        with self.assertRaisesRegex(RuntimeError,'退出码 4'):self.invoke(fixture.fake_generator(fail=True))
        self.old_files_unchanged();stage=next(self.stage_parent.iterdir())
        self.assertFalse((stage/'generator_attempt_02.log').exists())

    def test_retry_cannot_install_stale_metadata_from_failed_layout(self):
        body=self.generator().decode()
        body=body.replace("Path('black_box.txt').write_text", "if first: Path('black_box.txt').write_text")
        body=body.replace("Path('obstacle.txt').write_text", "if first: Path('obstacle.txt').write_text")
        with self.assertRaisesRegex(RuntimeError,'black_box.txt'):self.invoke(body.encode())
        self.old_files_unchanged()
        stage=next(self.stage_parent.iterdir())
        self.assertTrue((stage/'generation_failure_01/black_box.txt').exists())
        self.assertEqual(len(json.loads((stage/'generation_attempts.json').read_text())),2)

    def test_total_budget_does_not_restart_for_each_attempt(self):
        stage=self.base/'budget';stage.mkdir();robocup=stage/'official/robocup';robocup.mkdir(parents=True);generated=stage/'generated';generated.mkdir()
        def fake_run(*args):
            (generated/'robocup.world').write_text('');(stage/'generator.log').write_text('empty after placement failure')
        with patch.object(prepare.time,'monotonic',side_effect=[0.,0.,91.]),patch.object(prepare,'_run_generator',side_effect=fake_run) as run:
            with self.assertRaisesRegex(RuntimeError,'总时间预算耗尽'):prepare._generate_validated(robocup,generated,stage,90)
        self.assertEqual(run.call_count,1);self.old_files_unchanged()

if __name__=='__main__':unittest.main()
