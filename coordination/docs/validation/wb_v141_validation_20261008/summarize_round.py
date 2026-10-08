#!/usr/bin/env python3
"""Summarise one six-aircraft validation round (read-only).

Produces every mechanical number WORKBUDDY_SIM_RESULT_20261008.md needs:
budget/result provenance, judge deletions, trajectory span and travel, true
height, contact classification, visual observations, authority timeline,
observer outputs and the white near-range evidence channels.

Run:
  python3 summarize_round.py --root <root> [--json-out out.json]
"""
import argparse
import collections
import json
import os
import sys

GROUND = ("ground_plane",)
ACTOR_PREFIX = ("actor_",)


def load_jsonl(path):
    out = []
    if not os.path.isfile(path):
        return out
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except Exception:
                pass
    return out


def classify_pair(a, b):
    names = (a or ""), (b or "")
    low = " ".join(names)
    if any(g in low for g in GROUND):
        return "ground"
    if low.count("typhoon_h480") >= 2:
        return "uav_uav"
    if any(p in low for p in ACTOR_PREFIX):
        return "actor"
    if "typhoon_h480" in low:
        return "uav_obstacle"
    return "other"


def read_budget(root):
    p = os.path.join(root, "budget.json")
    try:
        return json.load(open(p, encoding="utf-8"))
    except Exception as error:
        return {"error": str(error)}


def judge_deletions(events):
    """first moment each actor id disappeared from left_actors, and the last set"""
    first_seen = {}
    last_value = None
    prev = None
    for row in events:
        if row.get("kind") != "left_actors":
            continue
        cur = set(row.get("value") or [])
        last_value = (row.get("sample_s"), sorted(cur))
        if prev is not None:
            for gone in sorted(prev - cur):
                first_seen.setdefault(gone, row.get("sample_s"))
        prev = cur
    return first_seen, last_value


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", required=True)
    ap.add_argument("--json-out", default="")
    args = ap.parse_args()

    root = args.root.rstrip("/")
    round_dir = None
    for name in sorted(os.listdir(root)) if os.path.isdir(root) else []:
        if name.startswith("round_"):
            round_dir = os.path.join(root, name)
    flight = os.path.join(round_dir or root, "flight")
    out = {"root": root, "round_dir": round_dir}

    # ---- provenance ---------------------------------------------------------
    budget = read_budget(root)
    attempt = (budget.get("attempts") or [{}])[-1] if isinstance(budget, dict) else {}
    out["budget"] = {k: attempt.get(k) for k in ("index", "seed", "sim_limit_s",
                                                 "status", "wall_seconds",
                                                 "run_id", "started_utc",
                                                 "ended_utc", "output")}
    result_path = os.path.join(flight, "result.json")
    if os.path.isfile(result_path):
        try:
            out["result_raw_keys"] = sorted(json.load(open(result_path, encoding="utf-8")).keys())
        except Exception as error:
            out["result_raw_keys"] = "unreadable: %s" % error
    src = os.path.join(flight, "execution_sources.json")
    out["execution_sources"] = json.load(open(src, encoding="utf-8")) if os.path.isfile(src) else {}

    # ---- judge --------------------------------------------------------------
    events = load_jsonl(os.path.join(flight, "city_events.jsonl"))
    kinds = collections.Counter(r.get("kind") for r in events)
    out["city_events"] = dict(rows=len(events), kinds=dict(kinds))
    deletions, last_left = judge_deletions(events)
    out["judge"] = dict(actors_deleted=sorted(deletions),
                        deleted_count=len(deletions),
                        first_deletion_s=min(deletions.values()) if deletions else None,
                        deletion_times={str(k): v for k, v in sorted(deletions.items())},
                        last_left_actors=last_left)

    visual = [r for r in events if r.get("kind") == "visual"]
    by_target = collections.Counter((r.get("value") or {}).get("target_id") for r in visual)
    out["visual_observations"] = dict(
        rows=len(visual), by_target=dict(by_target),
        rows_detail=[dict(sample_s=r.get("sample_s"),
                          target=(r.get("value") or {}).get("target_id"),
                          uav=(r.get("value") or {}).get("uav_id"),
                          xyz=(r.get("value") or {}).get("xyz"),
                          confidence=(r.get("value") or {}).get("confidence"))
                     for r in visual[:200]])

    # ---- trajectory ---------------------------------------------------------
    traj = load_jsonl(os.path.join(flight, "city_trajectory.jsonl"))
    if traj:
        span = [traj[0].get("sample_s"), traj[-1].get("sample_s")]
        ids = sorted((traj[0].get("positions") or {}).keys())
        first_pos = traj[0].get("positions") or {}
        last_pos = traj[-1].get("positions") or {}
        travel, zmax = {}, {}
        for uid in ids:
            try:
                a, b = first_pos[uid], last_pos[uid]
                travel[uid] = round(((b[0]-a[0])**2 + (b[1]-a[1])**2) ** .5, 3)
            except Exception:
                travel[uid] = None
            zmax[uid] = 0.0
        for row in traj:
            for uid, p in (row.get("positions") or {}).items():
                try:
                    zmax[uid] = max(zmax.get(uid, 0.0), float(p[2]))
                except Exception:
                    pass
        out["trajectory"] = dict(rows=len(traj), span_s=span,
                                 span_length_s=round((span[1] or 0)-(span[0] or 0), 3),
                                 displacement_m=travel,
                                 max_z_m={k: round(v, 3) for k, v in sorted(zmax.items())})
    else:
        out["trajectory"] = dict(rows=0)

    # ---- contacts -----------------------------------------------------------
    contacts = load_jsonl(os.path.join(flight, "city_contacts.log"))
    cat = collections.Counter()
    per_pair = collections.Counter()
    for row in contacts:
        if row.get("kind") != "UAV_BODY_CONTACT":
            continue
        c = classify_pair(row.get("collision1"), row.get("collision2"))
        cat[c] += 1
        per_pair[(row.get("collision1"), row.get("collision2"))] += 1
    hb = [r for r in contacts if r.get("kind") == "CONTACT_OBSERVER_HEARTBEAT"]
    out["contacts"] = dict(
        rows=len(contacts),
        kinds=dict(collections.Counter(r.get("kind") for r in contacts)),
        by_category=dict(cat),
        top_pairs=[dict(a=a, b=b, n=n) for (a, b), n in per_pair.most_common(10)],
        coverage=dict(frames=hb[-1].get("frames") if hb else None,
                      first_sample_s=hb[-1].get("first_sample_s") if hb else None,
                      last_sample_s=hb[-1].get("last_sample_s") if hb else None,
                      max_gap_s=hb[-1].get("maximum_sample_gap_s") if hb else None,
                      sonar_pairs=hb[-1].get("sonar_virtual_pairs") if hb else None,
                      body_pairs=hb[-1].get("body_pairs") if hb else None))
    manifest = os.path.join(flight, "contact_observer_manifest.json")
    out["contact_manifest"] = json.load(open(manifest, encoding="utf-8")) if os.path.isfile(manifest) else {}

    # ---- authority timeline -------------------------------------------------
    auth = load_jsonl(os.path.join(flight, "algorithm", "authority_events.jsonl"))
    out["authority"] = dict(
        rows=len(auth),
        by_event=dict(collections.Counter(r.get("event") for r in auth)),
        first_s=min([r.get("sim_s") for r in auth if r.get("sim_s") is not None], default=None),
        last_s=max([r.get("sim_s") for r in auth if r.get("sim_s") is not None], default=None))

    bridge = load_jsonl(os.path.join(flight, "algorithm", "bridge_trace.jsonl"))
    out["bridge"] = dict(rows=len(bridge),
                         kinds=dict(collections.Counter(r.get("kind") for r in bridge)))

    # ---- observers ----------------------------------------------------------
    obs_dir = os.path.join((round_dir or root), "observers")
    obs = {}
    for name in sorted(os.listdir(obs_dir)) if os.path.isdir(obs_dir) else []:
        if name.endswith(".json") and name.startswith("independent_visual_accuracy"):
            try:
                obs[name[:-5]] = json.load(open(os.path.join(obs_dir, name), encoding="utf-8"))
            except Exception:
                pass
        elif name.endswith(".jsonl") and name.startswith("independent_visual_accuracy"):
            rows = load_jsonl(os.path.join(obs_dir, name))
            obs[name[:-6]] = {
                "lines": len(rows),
                "kinds": dict(collections.Counter(r.get("kind") for r in rows)),
                "actor_truth_samples": sum(1 for r in rows if r.get("kind") == "actor_truth"),
                "truth_bounds": ([r["sample"][0] for r in rows if r.get("kind") == "actor_truth"][:1]
                                 + [r["sample"][0] for r in rows if r.get("kind") == "actor_truth"][-1:]),
            }
    out["observers"] = obs
    frames = os.path.join(obs_dir, "frames")
    out["observer_frames"] = len(os.listdir(frames)) if os.path.isdir(frames) else 0

    wd = os.path.join(obs_dir, "white_dev")
    if os.path.isdir(wd):
        idx = load_jsonl(os.path.join(wd, "index.jsonl"))
        out["white_dev"] = dict(
            pngs=len([f for f in os.listdir(wd) if f.endswith(".png")]), rows=len(idx),
            rows_detail=[dict(uav=r.get("uav"), image_time=r.get("image_time"),
                              depth=r.get("depth"), horiz_m=r.get("horiz_m"),
                              expected_h_px=r.get("expected_h_px"),
                              uv=r.get("uv"),
                              image_pose_delay_s=r.get("image_pose_delay_s"),
                              pose_pull_delay_s=r.get("pose_pull_delay_s"),
                              pose_alignment=r.get("pose_alignment"))
                         for r in idx[:60]],
            skipped=idx[0].get("skipped") if idx else None)

    # ---- N12 evidence channels ---------------------------------------------
    alg = os.path.join(flight, "algorithm")
    chans = {}
    for name in sorted(os.listdir(alg)) if os.path.isdir(alg) else []:
        if not name.startswith("evidence_"):
            continue
        rows = load_jsonl(os.path.join(alg, name, "index.jsonl"))
        pngs = len([f for f in os.listdir(os.path.join(alg, name)) if f.endswith(".png")])
        near = [r for r in rows if isinstance(r.get("range_m"), (int, float)) and r["range_m"] <= 22]
        chans[name] = dict(rows=len(rows), pngs=pngs, near_le_22m=len(near),
                           reject_reasons=dict(collections.Counter(r.get("reject_reason") for r in rows)),
                           white_rows=sum(1 for r in rows if r.get("cls") == "white"),
                           near_white=[dict(range_m=r.get("range_m"), color_conf=r.get("color_conf"),
                                            person_iou=r.get("person_iou"),
                                            n_person_boxes=r.get("n_person_boxes"),
                                            reject_reason=r.get("reject_reason"),
                                            image_stamp=r.get("image_stamp"),
                                            xyz=r.get("xyz"))
                                       for r in near if r.get("cls") == "white"][:40])
    out["evidence_channels"] = chans

    # ---- shared inference --------------------------------------------------
    shared = os.path.join(flight, "shared_inference.json")
    out["shared_inference"] = json.load(open(shared, encoding="utf-8")) if os.path.isfile(shared) else None

    print(json.dumps(out, ensure_ascii=False, indent=1)[:12000])
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as fh:
            json.dump(out, fh, ensure_ascii=False, indent=1)
        print("\nwrote %s" % args.json_out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
