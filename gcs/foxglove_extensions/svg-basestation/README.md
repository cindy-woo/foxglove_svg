# SVG Basestation

Ground-station panel for the SVG counter-UAS demonstration — the SVG analogue of
the DTC *Robot Control Panel* that anchors the `foxglove_ws` basestation layout.
One panel owns agent selection, the swarm-wide safety command, and the operator's
health picture.

It implements the two **Visual Insert Requirements** from the project brief:

| Requirement | Where it lands |
| --- | --- |
| Multi-tiered topology, dual-link redundancy (Wi-Fi mesh → 5G/4G failover) | Topology diagram at the top of the CommLink section — the path actually carrying each agent's telemetry is solid and coloured by health, the standby path dashed |
| Packet drop rate (<1%), end-to-end RTT (<20 ms), PTP drift, link health state transitions | CommLink metrics table + the transition log beneath it |
| Per-agent SoC, voltage sag under high-rate maneuvers, dynamic remaining mission time | Battery & Power cards |
| Automated RTB thresholds from distance-to-pad energy (>30% nominal, 20–30% gated, <20% failsafe) | RTB verdict chip, threshold ticks on each SoC bar, and the energy-margin row |

## Install

`install.py` picks it up with the other extensions — it copies any directory
here that has a `package.json` into `~/.foxglove-studio/extensions`:

```
python3 gcs/foxglove_extensions/install.py     # installs airlab-cmu.svg-basestation-1.0.0
```

`svg_basestation.json` (one directory up) is a ready-made layout: the panel
alongside a 3D view of `/svg/viz/markers` and SoC / pack-voltage plots. Load it
with **Layout → Import from file**.

## Wiring

Defaults match `robot/ros_ws/src/svg_ground_control` (`swarm_commander.py`); every
one is editable in the panel settings.

| Purpose | Default | Type |
| --- | --- | --- |
| State (position, speed) | `/{name}/odometry_conversion/odometry` | `nav_msgs/Odometry` |
| Primary link (Wi-Fi mesh) | `/{name}/odometry_conversion/odometry` | any stamped message |
| Backup link (5G/4G) | `/{name}/cellular/odometry` | any stamped message |
| Battery, hardware | `/{name}/fmu/out/battery_status` | `px4_msgs/BatteryStatus` |
| Battery, sim | `/{name}/interface/mavros/battery` | `sensor_msgs/BatteryState` |
| Link report (optional) | `/{name}/comms/link_status` | `std_msgs/String`, JSON |
| Lifecycle | `/swarm_commander/{takeoff,start,hold,land,reset_fence}` | `std_srvs/Trigger` |
| Formation | `/svg/formation_command` | `std_msgs/String` |

Set **Roles** to a comma-separated `guard|strike` list, one per agent, in
`drone_names` order — Guard is the defending swarm, Strike the intruding one.
**Position offsets** takes the same flat `x,y,z` per agent as
`drone_position_offsets`, so real and simulated agents share one world frame for
the distance-to-pad calculation.

## How the link metrics are derived

No extra ROS node is required: everything comes from the arrival statistics of
the telemetry already on the wire.

- **Drop rate** — the nominal publish period is the median inter-arrival gap; a
  gap of *k* periods counts as *k−1* missed samples out of *k* expected, over a
  10 s window. Gaps longer than the 1 s loss timeout are outages rather than
  packet loss, so they are excluded here and reported as a state transition
  instead.
- **RTT** — twice the mean one-way delay, where the one-way delay is
  `receive_time − header.stamp`. That identity holds exactly while PTP keeps the
  publisher and the GCS on one timebase.
- **PTP sync floor / drift** — the per-second minima of that same offset are the
  residual clock error plus minimum transit time. The least-squares slope of the
  floor over 120 s is the drift rate; a ramping floor means the clocks are
  separating, and the RTT column should be read against it.

If a deployment publishes a real link report on the link-status topic, any of
`drop_rate`, `rtt_ms`, `ptp_offset_ms`, `ptp_drift_ms` and `active_tier` present
in that JSON override the derived values. Anything with no source reads `--`
rather than showing a fabricated number.

**Link health**: `LOST` when neither tier has been heard from inside the loss
timeout; `DEGRADED` when drop rate or RTT is out of spec; `FAILOVER` when the
mesh is silent but the cellular path is live and in spec; otherwise `HEALTHY`.
Every change is timestamped in the transition log.

## How the power picture is derived

- **SoC / voltage / draw** come straight from the battery message; the SoC bar
  carries ticks at the two RTB thresholds.
- **Sag** is measured against the highest terminal voltage seen this session
  (the least-loaded sample available, so the best open-circuit proxy). Both the
  instantaneous and the session-peak sag are shown, per cell when the pack
  reports a cell count.
- **Mission time** prefers the autopilot's own `time_remaining_s`, falling back
  to SoC divided by the measured burn rate (the SoC slope over 60 s).
- **Return budget** is the distance-to-pad energy: cruise home at the configured
  speed, descend at the land speed, priced at the measured burn rate, plus a
  reserve. `RTB NOW` fires when SoC falls to that budget — ahead of the fixed
  percentage gates, which still apply independently.
