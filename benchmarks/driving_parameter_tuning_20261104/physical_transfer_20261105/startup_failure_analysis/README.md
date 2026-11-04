# EKF delivery trial v2: startup failed before fault injection

The original trial remains **FAIL**, classified as `STARTUP_LIFECYCLE_ABORT` /
`TRANSIENT_NOT_STARTED`. It did not exercise transient recovery or driving.
The actual controller reported a timed-out `get_state` response; the lifecycle
manager then aborted bringup. Map server and AMCL had been configured but were
never activated. The relay continued forwarding genuine odom/TF/scan while its
profile remained clean. No mission goal or nonzero output occurred. This is
not evidence that an injected sensor outage broke localization.

`witness.json` preserves the exact log excerpt, final relay counters, failed
acceptance reason and raw log/telemetry/installed-header SHA-256. Full original
evidence is retained in the adjacent `ekf_delivery_startup_failed` archive.
No CPU-cause claim is made: a service-response timeout is observed, but its
underlying scheduling or transport cause was not independently measured.

Installed packages are Nav2 1.1.20 (exact Debian revisions in the witness).
The manager exposes `autostart`, `node_names`, bond timeout and respawn options;
it does **not** expose a general lifecycle-service timeout parameter.
`LifecycleServiceClient::get_state()` defaults to two seconds in the installed
header. The [matching official manager source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_lifecycle_manager/src/lifecycle_manager.cpp)
configures all nodes, then activates all nodes, once. Initial failure returns
without the bond-respawn recovery path, which only handles a previously active
system's lost bond.

Blindly repeating `STARTUP` is not idempotent: already configured inactive nodes
would receive configure again. Blind `RESET` is also unsuitable for this mixed
state because an unconfigured node cannot be deactivated. The
[matching service-client source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_util/src/lifecycle_service_client.cpp)
shows explicit state-service timeouts rather than a configurable manager retry.

This change adds assessment labels only. It does not restart or activate any
node, alter a timeout, clear ESTOP, publish a goal or bypass pre-arm. A fresh
fully owned restart is a **new attempt**, retaining this failed evidence.
Automatic startup recovery would require a separate bounded, state-aware
owner that confirms actual transition state after a timeout and never repeats
an action blindly; it was deliberately not added during this investigation.
