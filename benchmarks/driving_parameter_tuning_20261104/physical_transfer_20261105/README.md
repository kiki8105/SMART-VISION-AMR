# Hardware-like Gazebo transfer tests — 2026-11-05

These are isolated, headless **Gazebo + actual ROS/Nav2 + web backend API** trials.
They did not drive the physical robot and do not constitute visual browser QA.
The operator guide and current result table are in
[GAZEBO_PHYSICAL_TRANSFER.md](../../../docs/GAZEBO_PHYSICAL_TRANSFER.md).

## Evidence and verdicts

Each archived trial contains its unmodified staged verdict, acceptance and safety
results, copied YAML/model/world, source fingerprints, and SHA-256 archive manifest.
Large ROS timelines are gzip-compressed without removing failed intervals.
`summary.json` is a compact index; the individual verdicts remain authoritative.

| Archive | Whole-trial verdict | Scope |
|---|---|---|
| [cold_rates_before_memory_fix](cold_rates_before_memory_fix/summary.json) | FAIL | Cold inventory/obstacle trial; Home reached, but no live bypass and excessive stopping rotation. Actual reverse motion occurred in this failed trial. |
| [cold_rates_after_memory_turn_fix](cold_rates_after_memory_turn_fix/summary.json) | PASS | Same scene after joint memory-geometry/moving-yaw changes; four box moves, live bypass and Home. Not a single-variable attribution. |
| [ekf_delivery_startup_failed](ekf_delivery_startup_failed/summary.json) | FAIL — transient never started | Nav2 lifecycle startup aborted. No fault profile or mission started; do not count as successful fault recovery. |
| [cold_ekf_delivery_transient_pass](cold_ekf_delivery_transient_pass/summary.json) | PASS | Actual EKF output odom/TF and scan delivery delay/loss, original stamps preserved; same goal resumed after hold. |
| [cold_wheel_asymmetry](cold_wheel_asymmetry/summary.json) | FAIL — invalid obstacle input | Motion/Home/stop passed; no box spawned because fixed-scene preflight failed. No obstacle claim. |
| [cold_actuator_lag](cold_actuator_lag/summary.json) | FAIL | Inventory, four box moves, live bypass and Home passed; stopping distance/rotation failed with the deliberately slow wheel plant. |
| [cold_sensor_common_invalid_spawn](cold_sensor_common_invalid_spawn/summary.json) | FAIL — invalid obstacle input | Motion/Home/stop passed; the actor reused its initial selection-distance window at final spawn. Original failed trial retained. |
| [cold_stress_parity](cold_stress_parity/summary.json) | PASS | Cold 1→2→3→Home in stress world using physical navigation/node values. Fixed-crate gap and stop QA passed; no actual reverse was needed. |
| [cold_stress_permuted](cold_stress_permuted/summary.json) | PASS | Cold 1→3→2→Home, real pre-arm, markers/dwell, static-crate and stop checks. Its source predates the later prefix certificate changes. |
| [cold_sensor_common_validated_failed](cold_sensor_common_validated_failed/summary.json) | FAIL | Valid actor with four moves; six planner-canceled heading starts exhausted the budget. No live bypass or Home completion. |
| [cold_common_prefix_owned](cold_common_prefix_owned/summary.json) | FAIL | Selected-prefix ownership fix executed; Home completed, but only one box move/no live bypass and a missing stop-entry truth sample. Direct acquisition-time TF exceptions remained. |

| [coherent_param_query_failed](coherent_param_query_failed/summary.json) | FAIL before motion | Active AMCL, but the read-only parameter response was lost; no initial-pose/motion verification claimed. |
| [common_coherent_no_live_bypass](common_coherent_no_live_bypass/summary.json) | FAIL | Four box moves, two actual reverses and Home; live bypass was not verified. Narrow full-cell geometry remained blocked. |
| [coherent_reentry_assessment_failed](coherent_reentry_assessment_failed/summary.json) | Original assessor FAIL | Both actual inventory round trips passed; an incorrect second cold-start requirement caused the original whole verdict. Pure reassessment is preserved separately. |
| [final_common_recorded](final_common_recorded/summary.json) | FAIL | Home/stop passed but no live bypass; three recorder captures preserve the quantized near-body observation. |
| [final_wheel_common_coherent_aborted](final_wheel_common_coherent_aborted/summary.json) | Explicit operator abort after stall | New route was dropped after a guard veto; mission was paused to end the stalled trial. This is not an autonomous completion. |

## Final frozen driving-runtime matrix

The driving runtime stayed unchanged throughout these eight fresh trials. The
last EKF run changes only a read-only exact-owner discovery wait within the same
four-second budget. The first failed startup remains in its own archive.

| Archive | Original whole verdict | Scope |
|---|---|---|
| [handoff_common](handoff_common/summary.json) | FAIL | 4 box moves, no live bypass; removal followed by Sector 3 and Home/stop. |
| [handoff_wheel](handoff_wheel/summary.json) | FAIL | 2 box moves, no live bypass; removal followed by Home/stop. |
| [final_inventory_reentry](final_inventory_reentry/summary.json) | PASS | Two real inventory round trips in the same process; pre-arm/reset/settle each time. |
| [final_stress_parity](final_stress_parity/summary.json) | PASS | Cold 1→2→3→Home, physical values, markers/dwell and fixed-crate truth checks. |
| [final_stress_permuted](final_stress_permuted/summary.json) | FAIL — operator abort | Sector 3 approach stalled for over six minutes. No Home completion; repeated NoPath/input waits retained. |
| [final_ekf_delivery](final_ekf_delivery/summary.json) | FAIL before fault | Actual relay worked, but an unknown DDS endpoint name caused early graph pre-arm failure. |
| [final_actuator_lag](final_actuator_lag/summary.json) | FAIL | Box gap 1.77 cm below 2 cm QA. Original stop QA missed cancellation coast; separate audit preserves that gap. |
| [final_ekf_delivery_discovery_fixed](final_ekf_delivery_discovery_fixed/summary.json) | PASS | Correct exact-owner discovery, then real EKF/scan delay and loss; same mission resumed and Home reached. |

[Machine-readable index](final_matrix.json) preserves original verdicts. Additional
[cancellation-stop coverage](cancel_stop_reassessment.json) is assessed separately
and never overwrites them. The slow-actuator safety result changes from PASS to
FAIL (14.42 cm travel versus its existing 11.55 cm profile bound); the original
three whole-trial passes remain passes. This is a posthoc assessment, not a new run.
No physical-ready or statistical success-rate claim is made.

## Independent checks

- [Final handoff/reentry/fault/actuator audit](independent_final_handoff_matrix_audit.json)
- [Final ordered and permuted integration audit](independent_final_integrated_matrix_audit.json)
- [Before/after source and outcome comparison](cold_before_after_comparison.json)
- [After-fix live-obstacle/truth audit](independent_after_fix_audit.json)
- [Actual delayed EKF output and same-goal resumption](independent_v3_fault_resume_audit.json)
- [Fresh stress-world source/model/actual-gap audit](independent_cold_stress_parity_audit.json)
- [Slow-wheel stopping dynamics](independent_actuator_stop_dynamics_audit.json)
- [Planner cancellation and the six heading starts](independent_common_validated_spin_failure_audit.json)
- [Direct TF-future certificate revocations](independent_prefix_owned_tf_revocation_audit.json)
- [Guard-veto route handoff: actual packet order and regression](independent_wheel_guard_handoff_audit.json)
- [Coherent trial geometry and actual reverse motion](independent_common_coherent_v2_geometry_tracking_audit.json)
- [Near-body observation quantization and recorder captures](independent_final_common_recorded_audit.json)
- [Second-cycle cold-start assessor correction](independent_coherent_reentry_assessor_reassessment.json)
- [Read-only AMCL parameter transport failure](independent_cold_parameter_transport_audit.json)
- [Startup failure analysis](startup_failure_analysis/README.md)
- [Archive byte/hash integrity](archive_integrity.json)
- [Build, Python/C++, deployment checks](checks.json)

The configured sensor rates, source-stamp rates, and wall receipt rates are
different measurements. For example, the stress trial's EKF source rate was
18.73 Hz despite a 20 Hz setting. Original `source_rates.json` files retain
per-topic intervals, age, and missing/duplicate/backward-stamp counts.

## Interpretation limits

The nominal/biased wheel and `.15 m/s²` slow-actuator settings are sensitivity
profiles, not coefficients measured on this physical robot. Delaying actual
EKF outputs tests downstream behavior; it does not reproduce the EKF's CPU
workload or the PC–Pi Wi-Fi network. Battery/OpenCR/USB/Dynamixel faults require
separate physical testing. A mission completion does not override a failed
stopping or collision-clearance verdict.

`archive_trial.py` only copies completed domain-176 evidence into a **new** archive
name and refuses to overwrite prior outcomes. See the operator guide for actual
trial commands; a new output directory is required for each run.

Recorder captures use a bounded, sampled before/after window, not a full rosbag.
Incident membership is based on the trigger wall time within the serial domain-176
trial observer interval; these old captures do not contain an embedded trial ID.
The archive preserves capture manifests, gzip data and hashes when available.
Historical PASS rows predate later fixes. The final frozen-runtime matrix is indexed
separately above; an earlier PASS is never carried forward as a new one.
