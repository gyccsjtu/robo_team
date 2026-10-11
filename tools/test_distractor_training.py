"""Dataset review, leakage and native sensor preservation gates."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parent/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


training = load('distractor_training', 'train_distractor.py')
capture = load('distractor_capture', 'training/capture_distractor.py')
evaluation = load('distractor_evaluation', 'training/evaluate_distractor.py')


class DatasetReviewTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.rows = []
        for i, split in enumerate(('train', 'val', 'test')):
            image, label = self.root/('%d.png' % i), self.root/('%d.txt' % i)
            image.write_bytes(('unique_image_%d' % i).encode())
            label.write_text(''.join('%d .5 .5 .2 .4\n' % cls for cls in range(6)))
            self.rows.append(dict(image=str(image), label=str(label), split=split,
                                  group='scene_'+split, reviewed=True, reviewer='visual_review',
                                  sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
                                  label_sha256=hashlib.sha256(label.read_bytes()).hexdigest()))

    def audit(self):
        manifest = self.root/'manifest.jsonl'
        manifest.write_text(''.join(json.dumps(r)+'\n' for r in self.rows))
        return training.audit_manifest(manifest)

    def test_reviewed_six_classes_in_independent_groups(self):
        rows, report = self.audit()
        self.assertEqual(len(rows), 3)
        self.assertEqual(report['missing_classes'], [])

    def test_adjacent_frames_from_same_recording_cannot_leak(self):
        self.rows[1]['group'] = self.rows[0]['group']
        with self.assertRaisesRegex(ValueError, 'leaks'):
            self.audit()

    def test_duplicate_content_cannot_masquerade_as_test(self):
        self.rows[1].update(image=self.rows[0]['image'], sha256=self.rows[0]['sha256'])
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            self.audit()

    def test_unreviewed_and_post_review_label_edits_rejected(self):
        self.rows[0]['reviewed'] = False
        with self.assertRaisesRegex(ValueError, 'Unreviewed'):
            self.audit()
        self.rows[0]['reviewed'] = True
        Path(self.rows[0]['label']).write_text('5 .5 .5 .3 .3\n')
        with self.assertRaisesRegex(ValueError, 'label checksum'):
            self.audit()

    def test_missing_actor_class_in_holdout_blocks_training(self):
        label = Path(self.rows[-1]['label'])
        label.write_text('5 .5 .5 .3 .3\n')
        self.rows[-1]['label_sha256'] = hashlib.sha256(label.read_bytes()).hexdigest()
        with self.assertRaisesRegex(ValueError, 'test:red'):
            self.audit()

    def prepare(self):
        self.audit()
        return training.prepare(self.root/'manifest.jsonl', self.root/'prepared')

    def test_prepared_yaml_cannot_redirect_train_to_test(self):
        import yaml
        data = self.prepare()
        training.audit_prepared_dataset(data)
        config = yaml.safe_load(data.read_text())
        config['train'] = 'images/test'
        data.write_text(yaml.safe_dump(config))
        with self.assertRaisesRegex(ValueError, 'YAML'):
            training.audit_prepared_dataset(data)

    def test_prepared_yaml_rejects_download_hook_and_hidden_images(self):
        import yaml
        data = self.prepare()
        original = data.read_text()
        config = yaml.safe_load(original)
        config['download'] = 'not_allowed'
        data.write_text(yaml.safe_dump(config))
        with self.assertRaisesRegex(ValueError, 'YAML'):
            training.audit_prepared_dataset(data)
        data.write_text(original)
        (data.parent/'images/train/unlisted.png').write_bytes(b'new')
        with self.assertRaisesRegex(ValueError, 'Extra or missing'):
            training.audit_prepared_dataset(data)

    def test_prepared_symlink_cannot_swap_test_into_train(self):
        data = self.prepare()
        link = data.parent/'images/train/000000.png'
        link.unlink()
        link.symlink_to(self.rows[2]['image'])
        with self.assertRaisesRegex(ValueError, 'differs'):
            training.audit_prepared_dataset(data)

    def test_candidate_provenance_must_bind_weights_and_exclude_selection_data(self):
        candidate = self.root/'run/weights/best.pt'
        candidate.parent.mkdir(parents=True)
        candidate.write_bytes(b'candidate')
        with self.assertRaisesRegex(ValueError, 'lacks'):
            training.verify_candidate_provenance(candidate, [self.rows[2]])
        provenance = dict(provenance_version=2, class_names=training.NAMES,
                          checkpoint_sha256={'best.pt': training.sha256(candidate)},
                          input_audit_timing='before_training',
                          dataset_inputs={'splits': {s:[r for r in self.rows if r['split']==s]
                                                      for s in ('train','val','test')}})
        path=candidate.parent.parent/'training_provenance.json'
        path.write_text(json.dumps(provenance))
        training.verify_candidate_provenance(candidate, [self.rows[2]])
        with self.assertRaisesRegex(ValueError, 'checkpoint selection'):
            training.verify_candidate_provenance(candidate, [self.rows[1]])
        same_recording=dict(self.rows[2],group=self.rows[0]['group'])
        with self.assertRaisesRegex(ValueError, 'recording/layout group'):
            training.verify_candidate_provenance(candidate,[same_recording])
        provenance['class_names']=training.NAMES[:5]
        path.write_text(json.dumps(provenance))
        with self.assertRaisesRegex(ValueError, 'mismatch'):
            training.verify_candidate_provenance(candidate, [self.rows[2]])
        provenance['class_names']=training.NAMES
        path.write_text(json.dumps(provenance))
        candidate.write_bytes(b'other_candidate')
        with self.assertRaisesRegex(ValueError, 'mismatch'):
            training.verify_candidate_provenance(candidate, [self.rows[2]])

    def test_offline_world_preserves_native_optics_and_official_file(self):
        world = self.root/'official.world'
        world.write_text('<sdf><world name="official"><state/><actor name="actor_0"/>'
                         '<plugin name="judge"/><model name="casual_female"/></world></sdf>')
        original = world.read_bytes()
        camera = self.root/'camera.sdf'
        camera.write_text('<sdf><model name="stereo_camera"><link><sensor>'
                          '<camera><horizontal_fov>1.5708</horizontal_fov>'
                          '<image><width>752</width><height>480</height></image></camera>'
                          '<plugin><cameraName>/stereo_camera</cameraName></plugin>'
                          '</sensor></link></model></sdf>')
        out = capture.make_world(world, camera, self.root/'offline.world')
        w = ET.parse(out).getroot().find('world')
        self.assertEqual(world.read_bytes(), original)
        self.assertEqual(w.get('name'), 'distractor_training_only')
        self.assertIsNone(w.find('actor'))
        self.assertIsNone(w.find('plugin'))
        self.assertEqual(w.find('.//horizontal_fov').text, '1.5708')
        self.assertEqual(w.find('.//image/width').text, '752')
        with self.assertRaisesRegex(ValueError, 'never an official source'):
            capture.make_world(world,camera,world)
        with self.assertRaisesRegex(ValueError, 'never an official source'):
            capture.make_world(world,camera,camera)
        self.assertEqual(world.read_bytes(),original)


class EvaluationTests(unittest.TestCase):
    def test_classification_training_preserves_frozen_bn_and_regression(self):
        import torch
        from torch import nn
        from PIL import Image

        class Head(nn.Module):
            def __init__(self):
                super().__init__()
                self.cv2=nn.ModuleList([nn.Sequential(nn.Conv2d(3,1,1),nn.BatchNorm2d(1)) for _ in range(3)])
                self.cv3=nn.ModuleList([nn.Conv2d(3,2,1) for _ in range(3)])

            def forward(self,x):
                return [m(x) for m in self.cv2], [m(x) for m in self.cv3]

        class Net(nn.Module):
            def __init__(self):
                super().__init__()
                self.model=nn.ModuleList([nn.Sequential(nn.Conv2d(3,3,1),nn.BatchNorm2d(3)),Head()])

            def forward(self,x):
                return self.model[1](self.model[0](x))

        with tempfile.TemporaryDirectory() as directory:
            image=Path(directory)/'probe.png'; Image.new('RGB',(8,8),(20,30,40)).save(image)
            model=Net()
            audit=training.ClassificationOnlyAudit(model,image)
            trainer=SimpleNamespace(model=model,freeze_layer_names=['model.0.'])
            def model_train():
                model.train()
                for name,module in model.named_modules():
                    if isinstance(module,nn.BatchNorm2d) and any(p in name for p in trainer.freeze_layer_names):
                        module.eval()
            trainer._model_train=model_train
            threads=torch.get_num_threads();torch.set_num_threads(1)
            try:
                optimizer=torch.optim.SGD(model.parameters(),lr=.01)
                audit.start(trainer)
                before=model.model[1].cv3[0].weight.detach().clone()
                model_train()
                _,classes=model(torch.ones(2,3,8,8))
                sum(x.square().mean() for x in classes).backward();optimizer.step()
                self.assertFalse(torch.equal(before,model.model[1].cv3[0].weight))
                audit.finish(trainer)
                self.assertTrue(audit.report['regression_output_exactly_unchanged'])
                self.assertTrue(all(n.startswith('model.1.cv3.') for n in audit.report['trainable_parameters']))
                model.model[0][1].running_mean.add_(.01)
                with self.assertRaisesRegex(RuntimeError,'BatchNorm state drifted'):
                    audit.finish(trainer)
            finally:
                torch.set_num_threads(threads)

    def test_auto_optimizer_must_not_ignore_requested_learning_rate(self):
        with self.assertRaisesRegex(ValueError, 'auto ignores'):
            training.retention_options(SimpleNamespace(optimizer='auto',lr0=.0001))
        self.assertEqual(training.retention_options(SimpleNamespace()),{'optimizer':'auto'})
        self.assertEqual(training.retention_options(SimpleNamespace(optimizer='AdamW',lr0=.0001,lrf=.1,freeze=10)),
                         {'optimizer':'AdamW','lr0':.0001,'lrf':.1,'freeze':10})

    def test_true_actor_class_conflict_does_not_count_as_usable_recall(self):
        labels=[(2,[0,0,20,60]), (5,[40,0,60,60])]
        predictions=[(2,[0,0,20,60],.94), (5,[0,0,20,60],.83), (5,[40,0,60,60],.9)]
        score=evaluation.score_image(labels,predictions)
        self.assertEqual(score['matched'][2],1)
        self.assertEqual(score['usable'][2],0)
        self.assertEqual(len(score['actor_conflicts']),1)
        self.assertEqual(score['false_decoy'],[])

    def test_neighbour_actor_overlap_is_not_same_instance_or_false_decoy(self):
        labels=[(2,[0,0,20,60]), (5,[8,0,28,60])]
        predictions=[(2,[0,0,20,60],.94), (5,[8,0,28,60],.93)]
        score=evaluation.score_image(labels,predictions)
        self.assertEqual(score['usable'][2],1)
        self.assertEqual(score['false_decoy'],[])

    def test_head_or_torso_actor_box_inside_decoy_is_false_positive(self):
        labels=[(5,[0,0,20,80])]
        predictions=[(2,[5,3,15,18],.7)]
        score=evaluation.score_image(labels,predictions)
        self.assertLess(evaluation.iou(labels[0][1],predictions[0][1]),.3)
        self.assertEqual(len(score['false_decoy']),1)
        self.assertEqual(len(score['partial_decoy']),1)

    def test_neighbour_actor_cannot_excuse_extra_sculpture_torso_false_box(self):
        labels=[(2,[0,0,20,60]), (5,[4,0,24,60])]
        predictions=[(2,[0,0,20,60],.94), (2,[4,0,24,50],.91),
                     (5,[4,0,24,60],.93)]
        score=evaluation.score_image(labels,predictions)
        self.assertEqual(score['usable'][2],1)
        self.assertEqual(score['false_decoy'],[predictions[1]])

    def test_box_closer_to_sculpture_cannot_impersonate_neighbour_actor(self):
        labels=[(2,[0,0,20,60]), (5,[4,0,24,60])]
        predictions=[(2,[4,0,24,50],.91), (5,[4,0,24,60],.93)]
        score=evaluation.score_image(labels,predictions)
        self.assertEqual(score['usable'][2],0)
        self.assertEqual(score['false_decoy'],[predictions[0]])

    def test_low_distractor_confidence_does_not_impose_runtime_conflict(self):
        score=evaluation.score_image([(2,[0,0,20,60])],
                                     [(2,[0,0,20,60],.94),(5,[0,0,20,60],.49)])
        self.assertEqual(score['usable'][2],1)

    def test_runtime_blue_vote_can_remove_white_even_below_blue_class_threshold(self):
        import numpy as np
        image=np.zeros((80,60,3),dtype=np.uint8); image[:,:,0]=220
        raw=[(3,[10,10,40,70],.8),(2,[10,10,40,70],.35)]
        survivors=evaluation.runtime_color_survivors(image,raw)
        self.assertEqual(survivors,{1})
        score=evaluation.score_image([(3,raw[0][1])],[raw[0]],set())
        self.assertEqual(score['matched'][3],1)
        self.assertEqual(score['usable'][3],0)
        self.assertEqual(len(score['color_suppressed']),1)

    def test_runtime_color_vote_does_not_remove_neighbouring_people(self):
        import numpy as np
        image=np.zeros((80,80,3),dtype=np.uint8);image[:,:,0]=220
        raw=[(2,[0,10,30,70],.8),(3,[20,10,50,70],.8)]
        self.assertEqual(evaluation.runtime_color_survivors(image,raw),{0,1})


if __name__ == '__main__':
    unittest.main()
