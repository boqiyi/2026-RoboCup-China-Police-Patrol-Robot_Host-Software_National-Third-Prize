# Accepted competition requirements

This is the review baseline for the current car_app changes. Source files,
drivers, configuration, tests and runtime asset references are all in scope.
Firmware and unrelated parent projects are not modification targets.

- Obstacle detection uses the fixed forward VL53L1X, not color thresholds.
  Valid range <=10 mm sends immediate binary STOP. Below 200 mm compare a
  frozen, reviewed wall interval at current directed-road progress; stop first,
  confirm blockage with three fresh samples, and keep unexplained cases stopped.
  Camera, YOLO, culvert inspection and existing selfcheck PWM remain in use.
- Bottom side opening transitions are the junction/corner evidence. STOP on
  the first credible transition, finish debounce at rest, then validate topology.
  A lateral offset with unchanged wall sum must not become a false junction.
  Turning requires stop/yaw confirmation but no lateral-centering gate.
- After turning, use proportional differential correction. Start the five-second
  target after stable dual walls; exclude scan pauses, preserve progress on
  resume, and continue at <=60 mm/s on timeout. Hardware convergence and braking
  <=100 mm require measured evidence; the 10 mm fallback is not that evidence.
- Calibration has three separately launched empty-field rounds, each reset to
  the same start pose, 58 directed steps and its own 1800-second limit. Retain
  originals and equal-weight round averages, then validate culverts and targets.
  Candidates never auto-activate. Unknown entry/rollback origins cannot borrow
  an unrelated profile. Preserve remote calibration and logs during uploads.
- Serial stays binary 115200 8N1, with HELLO first, motion updates below the
  500 ms lease and STOP for actual stops. Existing 2.2 range layout is retained;
  new firmware/2.3 integration awaits the teammate's actual protocol source.

- Selfcheck and start are separate commands. Start has a fresh 600-second
  monotonic deadline, necessary initialization and motion safety gates only.
- Each run resets mission, obstacle, visited-road, culvert, RFID and playback
  state. Historical logs, model files, map and calibration are preserved.
- Patrol numbers are numeric 0x01..0x0C in sector 0, block 1, byte 0.
  UID is identity only. Actual block data, independent of map labels, drives
  non-stopping queued announcements.
- There are three static obstacles on different roads per match, at most
  one obstacle and one movable culvert per road. Layout changes between matches.
  Four dark tunnels stay fixed. Neither culverts nor obstacles are placed in
  dark tunnel roads (user clarification during review round 1).
- On an obstacle: stop, turn 180 degrees, confirm return to the known entrance.
  The road is immediately forbidden for through-routing. Inspect its other
  entrance using a deduplicated last-in-first-out task stack. Resume old goals
  using newly computed routes. Inaccessible goals are recorded and abandoned.
- Every culvert is handled once per match, with at most one retry within that
  first handling. Empty successful scans are recorded silently. Identified
  targets are announced; incomplete scans remain incomplete. Later passages,
  including reverse passages, do not scan again.
- Budget return using actually traversed complete roads, including the worse
  of successful passage and far-end obstacle/turn/rollback, final entry and
  audio tail. Initial estimates are explicitly uncalibrated. Return takes
  priority over exploration and scanning. Final orientation is unrestricted.
- Movement/exploration ends at 600 seconds. Already queued audio may finish
  while stationary after the deadline, clearly marked late. Failed playback
  must terminate within a bound and is not counted as success.
- Every significant success/failure and reason, periodic one-second status,
  raw exception details and unfinished work appear in terminal and run files.
  Logging, camera reads and inference must not indefinitely block motion.
- After implementation and regression verification, perform 1 to 5 independent
  full reviews of a fixed source version. Fix only evidence-backed, necessary
  high-value issues. Stop early when a full review finds none and checks pass.
  Desktop checks never stand in for unperformed RDK/STM32/vehicle acceptance.
