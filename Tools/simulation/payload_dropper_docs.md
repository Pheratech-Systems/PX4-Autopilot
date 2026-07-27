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
  INFO  [payload_scorer] re-armed: detach commanded, waiting for impact
  INFO  [payload_scorer] HIT hull at (+7.05, +6.50, +1.30), peak Fz 0.6 N, against m1-abrams::body::hull_collision
  INFO  [payload_scorer] detonated boom_1
  INFO  [payload_scorer] ignited fire_1
  ```

  The `gz service` CLI echoes its Boolean reply, so `_create()` captures stdout rather
  than letting `data: true` land in the console, and turns a failed spawn into
  `WARN  [payload_scorer] failed to spawn fire_1: …`.

Knobs:

| Env var | Effect |
|---|---|
| `PX4_GZ_PAYLOAD_SCORER=0` | don't auto-start (run it by hand instead) |
| `PX4_GZ_PAYLOAD_SCORER_ARGS="..."` | replace the default `--explode` flags (e.g. score-only: `PX4_GZ_PAYLOAD_SCORER_ARGS=" "`) |
| `PX4_GZ_PAYLOAD_SCORER_SCRIPT` | override the script path |

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
