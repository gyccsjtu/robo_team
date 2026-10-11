# Moving-target follow continuity

Base: `280211a`. Reference GUI run: `logs_20261012_034605`, official 6/6 at 457.380 s.

## Evidence and changes

- Recorded green observations changed local camera ID from 3 to 6. The bridge treated the same camera ID change as a spatial alternative, rejecting fresh qualified observations; the owner was later released as lost. The exact report fixture is `tools/fixtures/green_rebuild_034605.json` (team sensor reports only).
- A non-red, single-candidate, same-camera rebuilt ID can now continue the existing trajectory after three unique qualified images spanning at least 0.35 s. Old anchor <=2 s, motion agreement, appearance, distance and normal formal-report guards remain required. Red slots, simultaneous people, stationary intruders, other cameras, stale positions and isolated jumps do not use this shortcut. No relocation/claimed elimination event is generated.
- A fresh, geometrically qualified moving target beyond the near observation radius can raise the follow cap to measured target speed +0.45 m/s, bounded by existing pursuit and flight limits. It no longer waits for the escape-state latch. Static, stale, close or pixel-only targets retain the slow cap; safety checks remain downstream.
- A complete pixel-only frame between fresh world updates no longer parks normal following. Actual clipping, shallow rays, invalid geometry and expired world positions keep bounded re-observation/hold behavior.
- Existing own-target priority is preserved and regression tested: another candidate cannot interrupt assigned tracking/reacquisition.

## Validation before the new GUI run

- 130 bridge/identity/ownership/follow/safety tests passed (`regression.txt`).
- 100 reporting/turn/view/obstacle tests passed (`report_regression.txt`).
- Exact green replay accepts rebuilt ID at image 2014.544, the third agreeing image, 0.428 s after its first appearance. The regression also checks that later control-only evidence does not declare elimination or create relocation.
- This verifies continuity, not absolute coordinate accuracy or a guaranteed match time. New GUI outcome is recorded separately after the run.

Trial configuration: GUI; v3_clean primary model; existing sculpture verifier; existing official map/actors/judge and sensor specification; gimbal off for comparison. Read-only recorders are separate from flight code.
