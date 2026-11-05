# CHANGELOG — arm_ik

## v0.4.1 — URDF-first runtime and safety corrections

* Runtime model construction now uses only a supplied URDF file or
  transient-local `/robot_description`. TF-only joint-axis inference is
  rejected because dynamic TF does not encode the original URDF joint axes.
* Recognizes the TienKung `left_tcp_link` / `right_tcp_link` TCP frames, with
  legacy `L_base_link` / `R_base_link` support.
* Added a `/joint_states` freshness watchdog and fail-closed behavior on stale
  feedback; partial JointState messages no longer silently reuse old positions.
* Corrected the table safety margin, sampled link segments for table checks,
  and validated fine-control corrections and raised waypoint trajectories.
* Reworked synchronized trapezoidal timing and added acceleration-limit tests.
* Fixed position-only orientation fallback so fine control does not chase an
  intentionally relaxed orientation.
* Fixed launch argument/default mismatches, demo yaw axis, package dependencies,
  executable installation and documentation.
* Fixed pure-Python quaternion normalization, XYZ Euler extraction, vector-bound
  clipping and multidimensional allclose.

## Earlier releases

Historical implementation notes from versions 0.1.0–0.2.2 are retained below.
They describe earlier iterations and may not match the current implementation.
