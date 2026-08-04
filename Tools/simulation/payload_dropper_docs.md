# Payload Dropper (SITL / Gazebo) — Implementation & Reproduction Guide

Adds a droppable payload to an `x500`-style PX4 model in Gazebo, triggered by the
**real** `MAV_CMD_DO_GRIPPER` command chain (the same MAVLink command a real drone
uses). This document is a self-contained recipe for reproducing it on a fresh PX4
checkout.

**7 changes across 3 areas**, done in the order below.

---

## How it works

```
MAVLink MAV_CMD_DO_GRIPPER  ─▶  payload_deliverer module  ─▶  uORB "gripper" topic (GRAB/RELEASE)
   (or mission "release" item)      (already in PX4)                    │
                                                                        ▼
                                             [ shim in gz_bridge ] ─▶ gz topic ".../detach"
                                                                        │
                                                                        ▼
                                             gz DetachableJoint system releases the payload
                                                    (coke_can falls under gravity)
```

Everything left of the shim is stock PX4 flight code. The shim + `DetachableJoint`
are the **simulation-only** substitute for a physical release servo (see
[Sim vs. real hardware](#sim-vs-real-hardware) at the end).

---

## Prerequisites

- PX4 SITL builds with Gazebo (`make px4_sitl gz_x500` works).
- The `gz-sim-detachable-joint-system` plugin exists (ships with gz Harmonic / gz-sim8):

  ```bash
  ls /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/ | grep detachable
  ```

---

## 1. New Gazebo model — `Tools/simulation/gz` (git submodule)

> Note: `Tools/simulation/gz` is a **git submodule**. These two files live in the
> submodule, not the main PX4 repo — commit them there (or in your fork of it).

`Tools/simulation/gz/models/x500_mono_cam_down_payload/model.config`

```xml
<?xml version="1.0"?>
<model>
  <name>x500_mono_cam_down_payload</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Your Name</name><email>you@example.com</email></author>
  <description>X500 with downward mono camera and a droppable payload.</description>
</model>
```

`Tools/simulation/gz/models/x500_mono_cam_down_payload/model.sdf`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_mono_cam_down_payload'>
    <self_collide>false</self_collide>

    <include merge='true'><uri>x500</uri></include>
    <include merge='true'>
      <uri>model://mono_cam</uri>
      <pose>0 0 .10 0 1.5707 0</pose>
      <name>mono_cam</name>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose relative_to="base_link">0 0 0 0 1.5707 0</pose>
    </joint>

    <!-- Payload (coke_can is a stock PX4 gz model; its link is named "link") -->
    <include>
      <uri>model://coke_can</uri>
      <name>payload</name>
      <pose>0.1 0 -0.05 0 0 0</pose>
    </include>

    <plugin filename="gz-sim-detachable-joint-system"
            name="gz::sim::systems::DetachableJoint">
      <parent_link>base_link</parent_link>
      <child_model>payload</child_model>
      <child_link>link</child_link>
      <!-- No <detach_topic> on purpose: the plugin defaults to
           /model/<runtime_model_name>/detachable_joint/detach, which carries the
           _N instance suffix the bridge shim also uses. Never hardcode it. -->
    </plugin>
  </model>
</sdf>
```

> ⚠️ **Biggest gotcha:** do NOT set `<detach_topic>` to a hardcoded model name. PX4
> spawns the model as `<name>_<instance>` (e.g. `..._0`), so a hardcoded topic
> without the suffix silently never matches — `Publish()` still succeeds, so it
> *looks* like it works. Omitting it lets the plugin derive the correct runtime name.
> No `server.config` change is needed — this is a model plugin, not a world system.

`coke_can` is a stock PX4 gz model already present in `Tools/simulation/gz/models/` —
no need to create it.

---

## 2. Airframe file

`ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_mono_cam_down_payload`

```sh
#!/bin/sh
#
# @name Gazebo x500 mono cam down with payload
#
# @type Quadrotor
#

PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_mono_cam_down_payload}

. ${R}etc/init.d-posix/airframes/4001_gz_x500

# Gripper machinery: type 0 = Servo enables it (this is already the default).
# There is NO PD_GRIPPER_EN param — do not add one.
param set-default PD_GRIPPER_TYPE 0
```

> Pick the next free `40xx` number for your repo — `4022` may already be taken on a
> different PX4 version.

---

## 3. Register the airframe

In `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`, add to the
`px4_add_romfs_files(...)` list:

```cmake
	4022_gz_x500_mono_cam_down_payload
```

---

## 4. Bridge shim — header

`src/modules/simulation/gz_bridge/GZBridge.hpp`

Add the include (near the other `uORB/topics/*`):

```cpp
#include <uORB/topics/gripper.h>
```

Add the members (in the `private:` section, by the other subscriptions):

```cpp
uORB::Subscription _gripper_sub{ORB_ID(gripper)};
gz::transport::Node::Publisher _payload_detach_pub;
bool _payload_detached{false};
```

---

## 5. Bridge shim — implementation

`src/modules/simulation/gz_bridge/GZBridge.cpp`

In `init()`, alongside the other `_node.Advertise(...)` calls:

```cpp
// Match the DetachableJoint default topic; _model_name carries the _N instance suffix.
std::string detach_topic = "/model/" + _model_name + "/detachable_joint/detach";
_payload_detach_pub = _node.Advertise<gz::msgs::Empty>(detach_topic);
```

In `Run()`, before the final `ScheduleDelayed(10_ms);`:

```cpp
gripper_s gripper;
if (_gripper_sub.update(&gripper)) {
	if (gripper.command == gripper_s::COMMAND_RELEASE && !_payload_detached) {
		gz::msgs::Empty msg;
		if (_payload_detach_pub.Publish(msg)) {
			_payload_detached = true;
			PX4_INFO("Payload released");
		}
	}
}
```

> - `gz::msgs::Empty` is already available via the `<gz/msgs.hh>` include at the top
>   of the header.
> - `_payload_detached` latches after the first release. `DetachableJoint` has no
>   `<attach_topic>` here, so the payload cannot re-attach anyway — **to drop again,
>   restart the sim.**
> - Mind the tabs — run `make check_format` before committing; PX4 CI enforces style.

---

## 6. Trigger script

`Tools/simulation/payload_release_pymavlink.py` — sends `MAV_CMD_DO_GRIPPER`
(param1 = gripper instance, param2 = `0` RELEASE / `1` GRAB) on UDP `14540+id`, and
waits through the `IN_PROGRESS` ack the `payload_deliverer` emits before the final
`ACCEPTED`.

```bash
pip install pymavlink
# take off first (other terminal / QGC), then:
python3 Tools/simulation/payload_release_pymavlink.py            # drone 0, instance 1, RELEASE
python3 Tools/simulation/payload_release_pymavlink.py --drone-id 2
python3 Tools/simulation/payload_release_pymavlink.py --grab     # send GRAB instead
```

Alternatively, add a "Release payload" mission item in QGroundControl — same command.

---

## 7. Build, run, verify

```bash
make px4_sitl gz_x500_mono_cam_down_payload
```

Before flying, confirm the two topics line up (this is what caught the original bug):

```bash
gz topic -l | grep detach
# → /model/x500_mono_cam_down_payload_0/detachable_joint/detach
```

That is exactly what the shim publishes to (`/model/` + `_model_name` +
`/detachable_joint/detach`). In the pxh shell, confirm the command path with
`listener gripper`. Take off, then run the trigger script — expect
`INFO [gz_bridge] Payload released` and the can drops.

---

## 8. Impact scoring — auto-started with the sim

`Tools/simulation/gz/payload_impact_scorer.py` listens on the payload's contact
sensor and prints `HIT`/`MISS` (+ miss distance from the tank), and with `--explode`
spawns an explosion burst and a lingering fire at the impact point.

It does **not** need its own terminal: `px4-rc.gzsim` starts it in the background
right after the model is spawned, for any airframe whose model name matches
`*payload*` (`x500_payload`, `x500_mono_cam_down_payload`). Two changes make that work:

`src/modules/simulation/gz_bridge/gz_env.sh.in` — export the path:

```sh
export PX4_GZ_PAYLOAD_SCORER_SCRIPT=@PX4_SOURCE_DIR@/Tools/simulation/gz/payload_impact_scorer.py
```

`ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim` — a `start_payload_scorer()` helper
(model-name gate, missing-bindings guard, duplicate guard) called with
`${MODEL_NAME_INSTANCE}` at the end of the spawn branch and with
`${PX4_GZ_MODEL_NAME}` in the attach branch. It launches:

```sh
python3 -u "${PX4_GZ_PAYLOAD_SCORER_SCRIPT}" --world "${PX4_GZ_WORLD}" \
	--model "${model_instance}" --wait 60 --exit-with-sim \
	${PX4_GZ_PAYLOAD_SCORER_ARGS:---explode} 2>&1 | tee "${scorer_log}" &
```

- World and carrier model come from the sim, so nothing is hardcoded to `tank` /
  `x500_mono_cam_down_payload_0` any more.
- `--wait 60` covers the contact sensor not being advertised the instant the model
  spawns.
- **Ctrl-C in the sim's terminal stops the scorer too.** This needs an explicit
  `install_signal_handlers()` in the script: POSIX has a non-interactive `sh` set
  `SIGINT`/`SIGQUIT` to `SIG_IGN` for *asynchronous* (`&`) jobs, and that disposition
  survives `exec` — so without restoring the default the scorer ignores Ctrl-C and
  keeps writing over the shell prompt that has already come back. It handles
  `SIGINT`/`SIGTERM`/`SIGHUP` and exits silently (a farewell line would land on top of
  the prompt).
- `--exit-with-sim` is the backstop for a sim that dies *without* signalling us
  (`pkill gz sim`, a crash): it polls the world's `/clock` topic every 3 s and quits
  after 2 misses. Deliberately an "is the topic still advertised" check — a GUI-paused
  sim publishes nothing but keeps its topics, so pausing is not mistaken for a dead sim.
- Output goes to the PX4 console **and** `build/px4_sitl_default/rootfs/payload_impact_scorer_<instance>.log`,
  in PX4's own console format so it reads as part of the sim log:

  ```
  INFO  [payload_scorer] world: tank, carrier: x500_mono_cam_down_payload_0
  INFO  [payload_scorer] armed, scoring impacts (explosion VFX on)
  INFO  [payload_scorer] run 20260803T142011Z, appending drop records to payload_impacts_x500_mono_cam_down_payload_0.jsonl
  INFO  [payload_scorer] re-armed: detach commanded, waiting for impact
  INFO  [payload_scorer] HIT hull on m1-abrams at (+7.05, +6.50, +1.30), peak Fz 0.6 N, 1.78 m from m1-abrams
  INFO  [payload_scorer] impact GPS 47.3980328, 8.5462432 (1.30 m AMSL)
  INFO  [payload_scorer] detonated boom_1
  INFO  [payload_scorer] ignited fire_1
  ```

  The `impact GPS` line follows every scored drop, hit or miss, so a drop can be
  eyeballed against a companion's GPS log or pasted into a map without converting by
  hand. It is derived from the world origin at 7 decimal places (~1 cm); the metres on
  the line above remain the authoritative measurement, and CEP is computed from those.
  Worlds with no geodetic origin simply omit the line.

  The `gz service` CLI echoes its Boolean reply, so `_create()` captures stdout rather
  than letting `data: true` land in the console, and turns a failed spawn into
  `WARN  [payload_scorer] failed to spawn fire_1: …`.

Knobs:

| Env var | Effect |
|---|---|
| `PX4_GZ_PAYLOAD_SCORER=0` | don't auto-start (run it by hand instead) |
| `PX4_GZ_PAYLOAD_SCORER_ARGS="..."` | replace the default `--explode` flags (e.g. score-only: `PX4_GZ_PAYLOAD_SCORER_ARGS=" "`) |
| `PX4_GZ_PAYLOAD_SCORER_SCRIPT` | override the script path |

---

## 9. Drop records — the input to CEP

Alongside the console lines the scorer appends one JSON object per drop to
`build/px4_sitl_default/rootfs/payload_impacts_<carrier>.jsonl` (`--record PATH` to
move it, `--record ''` to disable). **CEP is not computed here** — it is a statistic
over ~20–30 drops, and the drop latch means one drop per sim run, so a CEP sample is
always N processes appending to the same file. The scorer's job is to record truth;
the analysis derives the metric.

The record deliberately does **not** commit to a single "miss distance", because the
reference depends on the question:

| Reference | Answers | Why it isn't the default |
|---|---|---|
| Aimpoint | CEP | The scorer can't know it — see below |
| Nearest target | Effect / lethality | Flatters the system: in `military_recon_mini`, aiming at `abrams_0` and landing on `abrams_2` scores ~0 m |
| Struck target | nothing | Only defined on a HIT, and circular |

So each record carries the impact point **and every target's truth pose at impact**,
and the reference is chosen offline. Fields:

```json
{"schema":"payload_impact/1","run_id":"20260803T142011Z","drop_index":1,
 "sim_time":128.44,"wall_time":1785793889.07,
 "world":"military_recon_mini","carrier":"x500_mono_cam_down_payload_0",
 "impact":{"x":30.4,"y":12.3,"z":1.3},"peak_force_z_n":0.6,
 "outcome":"hit",
 "struck":{"model":"abrams_1","part":"hull","collision":"abrams_1::body::hull_collision"},
 "targets":[{"model":"abrams_0","x":8.0,"y":8.0,"z":-0.2,"q":[w,x,y,z],
             "dx":22.4,"dy":4.3,"horiz":22.81,"downrange":22.4,"crossrange":4.3}, "..."],
 "nearest":{"model":"abrams_1","horiz":0.50},
 "aimpoint":null,"aim_error":null}
```

- **Miss distance is recorded on hits too.** A hit on the far edge of a hull is still
  a ~3 m error; truncating those at the tank silhouette biases CEP low.
- **`dx`/`dy`, not just `horiz`.** A constant offset (bias) and a spread (variance)
  are different bugs. `downrange`/`crossrange` splits them further — release timing
  and ballistics show up downrange, tracking and yaw error show up crossrange.
- **`downrange` is along the target's local +X axis, not a ZYX yaw.** `tank.sdf`
  includes `m1-abrams` with a 90° roll (`<pose>8 8 -0.2 1.57 0 0</pose>`) to stand the
  mesh up, so its extracted yaw is 0 regardless of hull heading. The full quaternion
  is in the record so the analysis can redo the projection if the convention changes.
- **`sim_time`** is the join key against the ulog and the companion's tracking log.

### Aimpoint, and why the scorer doesn't ask the tracker for it

`--aim-model <name>` / `--aimpoint X,Y` stamp the intended target into each record,
for **ballistics-only runs with no tracker in the loop**. With a tracker flying the
drop, leave both unset and join its aimpoint log to these records by `sim_time`
offline. The tracker is the system under test: if it is biased, its aimpoint and its
own post-hoc estimate of where the payload landed share that bias, and it grades
itself as accurate. Measurement has to come from an independent source — the contact
sensor here, a survey/RTK mark in the field.

When an aimpoint *is* declared, the record gains `aim_error` and a `misassigned`
flag. Misassignment is reported as its own rate, never folded into CEP: a drop
dead-centre on the wrong tank is a target-selection failure, and averaging it in as a
0.3 m miss hides it completely.

### Frames: metres are authoritative, lat/lon is a join convenience

A companion computer logs the drop in **GPS**; the scorer measures it in the gz world
frame in **metres**. They reconcile through the world's `<spherical_coordinates>`,
which the scorer reads from the world SDF (there is no gz service to read it back,
only `/world/<w>/set_spherical_coordinates`) and stamps into every record:

```json
"origin":{"lat":47.397971057728974,"lon":8.546163739800146,"elev":0.0,
          "frame":"ENU","source":"tank.sdf"}
```

**Do not convert the Gazebo truth into GPS to compute CEP.** Two reasons:

1. **You don't need to.** The scorer already has the impact point and the target's
   true position in metres, and CEP *is* a distance in metres. Projecting truth into
   degrees so the analysis can project it back into metres only loses precision and
   adds a datum error to a number that was exact.
2. **The conversion runs the other way.** Gazebo is truth; the companion's GPS is the
   estimate under test. Bring the *estimate* into the truth frame, not truth into the
   estimate's frame — otherwise the projection error ends up baked into the reference.

So `x`/`y` stay authoritative and `lat`/`lon` are emitted alongside purely so a record
lines up against a GPS-only log. `enu_to_geodetic()` uses the WGS84 meridional and
prime-vertical radii rather than one spherical radius: at 47° they differ by ~0.17%,
which is under a centimetre at 7 m but **12.7 cm at 100 m** — the wrong order of
magnitude to ignore against a sub-metre CEP. Use the same model on the companion side.

`--origin LAT,LON[,ELEV]` overrides discovery. A world with no
`<spherical_coordinates>`, or one whose `world_frame_orientation` is not `ENU`, gets
`"origin": null` and a warning rather than plausible-looking wrong coordinates.

**Join key:** the companion's `t_utc_us` against the record's `wall_time` (both
wall-clock UTC epoch). `sim_time` is the gz clock and does not track wall time under
lockstep, so it joins to the ulog, not to a companion log.

### Arming: only a commanded release counts as a drop

The payload hangs below `base_link` and **rests on the ground plane while the drone is
parked**, so a scorer that starts armed books that contact as a drop ~2.8 s into every
run. It logs as a miss tens of metres out (21.5 m in the `tank` world), and one such
outlier per sim start is enough to wreck a CEP sample — with one drop per run, half the
records would be phantoms.

So the scorer starts **disarmed** and arms on the detach command:

```
INFO  [payload_scorer] waiting for the detach command before scoring (pre-release ground contact is not a drop)
INFO  [payload_scorer] armed: detach commanded, waiting for impact
INFO  [payload_scorer] HIT hull on m1-abrams at (+6.91, +6.89, +1.30), peak Fz 131.7 N, 1.55 m from m1-abrams
```

The idle re-arm (`REARM_IDLE_SEC`) is disabled in this mode — it is what used to *mask*
the phantom instead of preventing it, and it would also let a bounce or roll after
touchdown book a second record for one release. If the detach subscription fails, the
scorer falls back to the old arm-immediately behaviour and **warns** that records may
contain phantom drops, rather than silently scoring nothing.

Records with `"targets": []` are unusable for CEP (no truth pose, so no miss vector).
The scorer warns at the time; drop them in analysis.

### Multi-target worlds

`--target-pattern` (default `abrams`) is a regex over model names, matching both
`tank.sdf`'s bare `m1-abrams` include and `military_recon_mini.sdf`'s renamed
`abrams_0` / `abrams_1` / `abrams_2`. The hit is attributed to a specific instance by
taking the model segment of the scoped collision name
(`abrams_1::body::hull_collision` → `abrams_1`) — the collision *suffix* is identical
across every tank, so it alone cannot say which one was struck.

`px4-rc.gzsim` needs no change for any of this: every added flag has a default, so the
target-pattern fix applies to the auto-started scorer automatically. To override, put
the flag in `PX4_GZ_PAYLOAD_SCORER_ARGS` — but note that variable *replaces* the
default `--explode`, so pass both if you want VFX:

```bash
PX4_GZ_PAYLOAD_SCORER_ARGS="--explode --target-pattern pickup" \
	PX4_GZ_WORLD=pickup make px4_sitl gz_x500_mono_cam_down_payload
```

### Which worlds this actually works in

The scorer auto-starts for any `*payload*` model in **any** world, but what it can
measure degrades in two tiers:

| World | Miss vector / CEP | HIT/MISS label |
|---|---|---|
| `tank`, `tank_moving` (`m1-abrams`) | yes | yes |
| `military_recon_mini` (`abrams_0/1/2`) | yes | yes |
| `pickup`, `suv`, `hatchback`, `prius_hybrid` | yes, with `--target-pattern` | **no** |
| `default`, `baylands`, `lawn`, … | no target at all | no |

The split is that **miss distance is pose-based and portable, but hit classification
is not.** `TARGETS` keys on `hull_collision` / `turret_collision`, which only
`m1-abrams` defines — `pickup` and `suv` name theirs plain `collision`. Against those,
a strike books as `outcome: "other"` with `struck: null`, while `targets[]`, `nearest`
and the miss vector are all still correct, so CEP still computes. Add the model's
collision suffix to `TARGETS` to get the label back.

In a world with no matching target the record has `"targets": []` and the scorer warns
that it is unusable for CEP.

So the run-time workflow is three terminals, not four:

```bash
# 1) sim + scorer (scorer starts itself)
PX4_GZ_WORLD=tank make px4_sitl gz_x500_mono_cam_down_payload
# 2) QGroundControl — arm, take off, fly over the tank
# 3) drop
python3 Tools/simulation/payload_release_pymavlink.py
```

> `gz_env.sh.in` is a `configure_file` template: after editing it, plain
> `make px4_sitl` may not regenerate `build/px4_sitl_default/rootfs/gz_env.sh` —
> run `cmake build/px4_sitl_default` once.

> The scorer needs `python3-gz-transport13`; without it the sim still starts and the
> init log says `payload impact scorer skipped: no gz-transport13 python bindings`.

---

## Troubleshooting quick-map

| Symptom | Cause | Fix |
|---|---|---|
| `Payload released` prints but nothing drops | detach topic name mismatch (hardcoded vs runtime `_0`) | omit `<detach_topic>`; build shim topic from `_model_name` |
| Payload falls at spawn | `<child_model>`/`<child_link>` name wrong → joint never created | match nested model name (`payload`) and its link (`link`) |
| ACK "not accepted" (`result=5`) | that's `IN_PROGRESS`, not failure | wait past it (script already does) |
| Gripper never acts | expected a `PD_GRIPPER_EN` param | it doesn't exist — use `PD_GRIPPER_TYPE 0` |
| CI format failure | astyle indentation | `make check_format` |
| Can't drop twice in one session | `_payload_detached` latch + no `<attach_topic>` | restart the sim |
| No HIT/MISS lines in the sim console | scorer not started (non-payload model name, or missing gz python bindings) | check the init log for `payload impact scorer`; `apt install python3-gz-transport13` |
| `INFO [payload_scorer] …` lines printed over the shell prompt after Ctrl-C | background job of a non-interactive shell has `SIGINT` set to `SIG_IGN`, so only the `--exit-with-sim` poll could end it | `install_signal_handlers()` restores the default disposition — verify it's still called from `main()` |
| Scorer still running after the sim is gone | started by hand without `--exit-with-sim` | it self-exits when auto-started; otherwise Ctrl-C it |

---

## Sim vs. real hardware

Items **1, 4, 5** (the model + the `gz_bridge` shim + `DetachableJoint`) are
**simulation-only**. A real drone has no detach topic — a physical servo moves a
latch.

On real hardware you instead:

1. Keep `PD_GRIPPER_TYPE = 0` (servo).
2. In the QGC **Actuators** tab (or `PWM_AUX_FUNCn` / actuator params), assign a
   servo output (e.g. AUX1) to output function **Gripper (430)**.
3. Set that output's min/max/disarmed PWM so the endpoints match latched / released.
4. `FunctionGripper` drives the output to `+1` on GRAB and `-1` on RELEASE — the
   servo swings and the payload drops.

Items **2, 3, 6** (airframe params, airframe registration, MAVLink trigger) carry
over unchanged.

> Note: the `DetachableJoint` sim path bypasses the actuator/mixer output entirely
> (uORB → shim → gz), so it validates command/mission logic but **not** the servo
> output mapping. To exercise the exact hardware code path in sim, use the servo-latch
> variant instead: set `SIM_GZ_SV_FUNC1 = 430` and add a `servo_0` revolute joint with
> a `JointPositionController` plugin to the model (see the `standard_vtol` model for
> the pattern).
