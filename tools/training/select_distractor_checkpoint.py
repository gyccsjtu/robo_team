#!/usr/bin/env python3
"""Select saved epochs using validation gates only; never infer/score test images.

Every actor color must retain raw and usable recall within one percentage
point of v1 on original340 and new validation layouts. Sculptures must have
zero complete/partial actor false boxes and old negative false frames must
not increase. Among passes prefer sculpture recall, mean actor recall, the
weakest color's recall change, then the earlier epoch. No weight is deployed.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import time

spec=importlib.util.spec_from_file_location('distractor_evaluation',Path(__file__).with_name('evaluate_distractor.py'))
evaluation=importlib.util.module_from_spec(spec);spec.loader.exec_module(evaluation)
training=evaluation.training


def brief(report):
    return {k:v for k,v in report.items() if k!='frames'}


def rank_pass(original, new_layout, baseline, epoch):
    recall=original['usable_actor_recall']
    changes=[recall[n]-baseline['usable_actor_recall'][n] for n in training.NAMES[:5]]
    return (new_layout['recall']['distractor_person'] or 0.,
            sum(recall.values())/len(recall),min(changes),-epoch)


def failure_reasons(scope, baseline, candidate):
    failures=[]
    for metric in ('recall','usable_actor_recall'):
        for color in training.NAMES[:5]:
            old,new=baseline[metric][color],candidate[metric][color]
            if old is None or new is None or new+1e-12<old-.01:
                failures.append(dict(scope=scope,reason='actor_recall_drop',metric=metric,
                                     color=color,baseline=old,candidate=new,allowed_drop=.01))
    if candidate['false_decoy_frames']:
        failures.append(dict(scope=scope,reason='complete_or_partial_actor_false_boxes_on_sculpture',
                             frames=candidate['false_decoy_frames'],allowed_frames=0))
    return failures


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',required=True);p.add_argument('--baseline',required=True)
    p.add_argument('--run',required=True);p.add_argument('--output',required=True)
    p.add_argument('--policy',required=True);p.add_argument('--policy-sha256',required=True)
    p.add_argument('--device',default='cpu')
    args=p.parse_args()
    if training.sha256(args.policy)!=args.policy_sha256:
        raise ValueError('Selection policy changed from the predeclared SHA256')
    rows,_=training.audit_manifest(args.manifest)
    original=[r for r in rows if r.get('original_split')=='val' and
              r['group']=='legacy_original_val' and Path(r['image']).parents[2].name=='yolo_ds']
    negatives=[r for r in rows if r.get('original_split')=='val' and
               r['group']=='legacy_original_val' and r not in original]
    new_layout=[r for r in rows if r['split']=='val' and r['group']!='legacy_original_val']
    if len(original)!=340 or len(negatives)!=34 or not new_layout:
        raise ValueError('Expected original340, negative34 and reviewed new-layout validation')
    if any(training.read_labels(r['label']) for r in negatives):
        raise ValueError('Original negatives must have reviewed empty labels')
    paths=sorted((Path(args.run)/'weights').glob('epoch[0-9]*.pt'),key=lambda x:int(x.stem[5:]))
    if not paths:
        raise ValueError('No native epoch checkpoints; train with --save-period 1')
    policy=json.loads(Path(args.policy).read_text())
    if policy.get('defined_wall_s',float('inf'))>min(x.stat().st_mtime for x in paths):
        raise ValueError('Selection policy must predate the saved candidates')
    base={name:evaluation.evaluate(args.baseline,group,args.device)
          for name,group in [('original340',original),('new_layout',new_layout),('negative34',negatives)]}
    result=dict(start_wall_s=time.time(),evaluation_scope='validation_only',
                manifest_sha256=training.sha256(args.manifest),selection_policy=policy,
                selection_policy_sha256=args.policy_sha256,
                runtime_source_sha256=evaluation.RUNTIME_SOURCE_SHA256,
                evaluator_source_sha256=training.sha256(evaluation.__file__),
                selector_source_sha256=training.sha256(__file__),
                baseline={k:brief(v) for k,v in base.items()},epochs=[],selected=None,
                test_images_inferred=0,test_used_for_selection=False,deployed=False)
    selected_rank=None
    output=Path(args.output)
    for path in paths:
        epoch=int(path.stem[5:])+1
        provenance=training.verify_candidate_provenance(path,original+negatives+new_layout,independent=False)
        current=evaluation.evaluate(path,original,args.device,candidate=True)
        gate=evaluation.compare(base['original340'],current)
        row=dict(epoch=epoch,weights=str(path),provenance=provenance,
                 checkpoint_sha256=training.sha256(path),
                 original340=brief(current),original340_gate=gate,accepted=False,
                 failure_reasons=failure_reasons('original340',base['original340'],current))
        if gate['image_acceptance']:
            extra=evaluation.evaluate(path,new_layout,args.device,candidate=True)
            extra_gate=evaluation.compare(base['new_layout'],extra)
            row.update(new_layout=brief(extra),new_layout_gate=extra_gate)
            row['failure_reasons'].extend(failure_reasons('new_layout',base['new_layout'],extra))
            if extra_gate['image_acceptance']:
                negative=evaluation.evaluate(path,negatives,args.device,candidate=True)
                negative_ok=negative['empty_negative_false_frames']<=base['negative34']['empty_negative_false_frames']
                row.update(negative34=brief(negative),negative_gate=negative_ok,accepted=negative_ok)
                if not negative_ok:
                    row['failure_reasons'].append(dict(scope='negative34',reason='empty_negative_actor_false_frames_increased',
                                                      baseline=base['negative34']['empty_negative_false_frames'],
                                                      candidate=negative['empty_negative_false_frames']))
                if negative_ok:
                    rank=rank_pass(current,extra,base['original340'],epoch)
                    row['rank']=rank
                    if selected_rank is None or rank>selected_rank:
                        selected_rank=rank
                        result['selected']=dict(epoch=epoch,weights=str(path),sha256=training.sha256(path),rank=rank)
        result['epochs'].append(row)
        result['updated_wall_s']=time.time()
        output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(dict(epoch=epoch,accepted=row['accepted'],
                              usable_recall=current['usable_actor_recall']),ensure_ascii=False),flush=True)
    result['finish_wall_s']=time.time()
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(selected=result['selected'],test_images_inferred=0,test_used_for_selection=False,deployed=False),indent=2))
    if result['selected'] is None:
        raise SystemExit(1)


if __name__=='__main__':
    main()
