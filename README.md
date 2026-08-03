# iw_hub Autonomous Pick-and-Place — Isaac Sim + ROS2 Nav2

Autonomous navigation and pick-and-place for the idealworks `iw_hub` AMR,
simulated in NVIDIA Isaac Sim and driven entirely through ROS2 Humble + Nav2.
A robot navigates to a dolly, lifts it, carries it across a mapped
environment, drops it off, and clears itself — all triggered by a single
service call (with a browser dashboard on top).

<!-- Replace with an actual GIF/screenshot once you have one -->
<!-- ![demo](docs/demo.gif) -->

## What this does

1. **Maps and localizes** a warehouse-style environment using SLAM Toolbox
   and Nav2's AMCL, built around a dual-lidar (front + rear) diff-drive base.
2. **Navigates autonomously** to named waypoints via Nav2's `NavigateToPose`.
3. **Lifts and lowers a scissor-lift joint** smoothly (not a snap-to-position
   jump) to grab/release a dolly.
4. **Runs a full pick-and-place mission** end-to-end from a single ROS2
   service call: drive to the dolly, lift it, drive to the drop zone, lower
   it, and pull clear — all automatically.
5. **Exposes a browser dashboard** to trigger missions and watch live
   telemetry (lift state, robot pose) without touching the CLI.

## Architecture

```
                     ┌─────────────────────────┐
   Browser  ───ws───▶│      rosbridge_server    │
  dashboard          └───────────┬─────────────┘
                                  │
                        ┌─────────▼─────────┐
                        │ mission_executor_  │──calls──▶ Nav2 (NavigateToPose)
                        │       node          │──pub───▶ /lift_position_cmd
                        │  (/mission service)  │──pub───▶ /cmd_vel (retreat move)
                        └─────────────────────┘
                                  │
                        ┌─────────▼─────────┐
                        │   lift_ramp_node    │──pub───▶ /joint_command
                        └─────────────────────┘
                                  │
                     ┌────────────▼────────────┐
                     │   Isaac Sim (iw_hub)      │
                     │  diff-drive + dual lidar   │
                     │  + scissor lift joint      │
                     └───────────────────────────┘
```

## Packages

| Package | Type | Purpose |
|---|---|---|
| `iw_hub_navigation` | ament_python | Nav2 bringup — SLAM, localization, params |
| `iw_hub_lift_control` | ament_python | Velocity-limited lift joint control |
| `iw_hub_pick_place_msgs` | ament_cmake | Custom `RunMission` service definition |
| `iw_hub_pick_place_executor` | ament_python | On-demand pick-and-place mission service |
| `frontend/iw_hub_console.html` | standalone | Browser dashboard (rosbridge + roslibjs) |

Build order matters — the interfaces package first:
```bash
colcon build --packages-select iw_hub_pick_place_msgs
source install/setup.bash
colcon build --packages-select iw_hub_navigation iw_hub_lift_control iw_hub_pick_place_executor
source install/setup.bash
```

---

## 1. Mapping and localization (`iw_hub_navigation`)

Nav2 stack tuned for the `iw_hub`'s diff-drive base and dual RPLidar S2E
lidars (front + rear used as separate costmap obstacle sources; AMCL
localizes off the front lidar only, since it can't fuse two scan topics).

The map itself was generated using **Isaac Sim's built-in Occupancy Map**
tool (not SLAM) — since the full environment geometry already exists in the
USD stage, Occupancy Map projects it directly into a 2D grid without needing
to drive the robot around first. Set the tool's Z-slice range to bracket the
lidar's actual mounted height before generating, so the exported map matches
what the real 2D lidar scan will detect (a mismatched Z range was an earlier
issue — see debugging notes below).

```bash
# Localize + navigate against the exported occupancy map
ros2 launch iw_hub_navigation isaac_nav2_bringup.launch.py
```

If you don't have geometry to project from — e.g. a physical robot, or a sim
environment you haven't fully modeled — SLAM Toolbox (`slam_toolbox`) is the
alternative: drive the robot around while it builds the map from live lidar
data instead. Not used here, but worth knowing since it needs no upfront
environment authoring.

Turning speed was deliberately reduced (`max_vel_theta`, `acc_lim_theta`,
etc. in `nav2_params.yaml`) below Nav2's usual defaults — the dolly rests on
the lift platform via friction only (no rigid joint), so aggressive turns
were causing it to slide and sit crooked mid-transport.

## 2. Lift control (`iw_hub_lift_control`)

Commanding `/joint_command` directly snaps the lift straight to a target
position. `lift_ramp_node` instead steps the commanded position toward the
target at a bounded velocity (`max_velocity`, default 0.01 m/s), publishing
at 50 Hz, so the motion is smooth and doesn't destabilize whatever's on it.

```bash
ros2 launch iw_hub_lift_control lift_ramp.launch.py
ros2 topic pub --once /lift_position_cmd std_msgs/msg/Float64 "data: 0.09"
```

## 3. Pick-and-place mission service (`iw_hub_pick_place_msgs` + `iw_hub_pick_place_executor`)

A single service call runs the full sequence:

```
go to pick_up  ->  raise lift  ->  go to destination  ->  lower lift  ->  drive straight clear
```

```bash
ros2 run iw_hub_lift_control lift_ramp_node &
ros2 launch iw_hub_pick_place_executor mission_executor.launch.py

ros2 service call /mission iw_hub_pick_place_msgs/srv/RunMission \
  "{pick_up: goal_a, destination: goal_b}"
```

Waypoints live in `iw_hub_pick_place_executor/config/pick_place_goals.yaml`
as named `x`/`y`/`yaw` points in the map frame.

The final "drive straight clear" step exists because the dolly isn't sensed
by the lidars as a costmap obstacle — without it, the very next mission's
path could clip the just-dropped dolly mid-turn. It bypasses Nav2 entirely
(direct `/cmd_vel` publish, distance tracked via `/odom`) so the motion is
guaranteed dead straight.

## 4. Browser dashboard (`frontend/iw_hub_console.html`)

Standalone HTML file, no build step. Connects to `rosbridge_server` over a
websocket, lets you pick waypoints and trigger `/mission`, and shows live
lift state (Lifted / Lowered / Moving) and robot pose.

```bash
sudo apt install ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
Open the HTML file locally in any browser and point it at
`ws://<robot-host>:9090`. A Simulated mode is built in for testing the UI
without a live stack.

## Physics tuning: friction

The dolly rests on the lift platform via contact friction only (no rigid
joint). Contact friction between the lift platform and dolly was increased
directly in Isaac Sim's UI (selecting the PhysX material on both surfaces and
raising `staticFriction`/`dynamicFriction`), combined with the reduced
turning speed above, to stop the dolly sliding during transport.

---

## Debugging notes (things that weren't obvious at the time)

- **Occupancy Map's Z-slice range has to match the lidar's mounted height**,
  not just "capture everything." The tool projects geometry within a
  configurable Z range regardless of where the lidar actually sits — if that
  range doesn't bracket the lidar's real height, the exported map won't
  match what AMCL sees from the live 2D scan, causing localization drift
  even though the map looks fine visually.
- **Nav2 plugin lookup strings are inconsistent across packages** in this
  Humble build: `nav2_navfn_planner` and `nav2_behaviors` need the
  `package/ClassName` slash format; `nav2_controller`, `dwb_core`,
  `nav2_costmap_2d`, `nav2_waypoint_follower` need `namespace::ClassName`.
  Both are correct as checked in — don't "fix" them to be consistent with
  each other.
- **AMCL only supports one scan topic** (`scan_topic` param) — it can't fuse
  front+rear lidar the way the costmap obstacle layers can via
  `observation_sources`. Currently localizing off the front lidar alone.
- **`bt_navigator`'s default BT XML params need full paths**, not bare
  filenames — `nav2_bringup`'s own launch usually injects this, but a custom
  bringup launch has to set it explicitly.
- **The mission executor blocks on Nav2 action results inside a service
  callback** — this requires a `MultiThreadedExecutor` +
  `ReentrantCallbackGroup`; a default single-threaded executor deadlocks
  here since the callback waits on another callback to fire.
- **The executor originally didn't check `send_lift_command`'s return
  value** — it would drive to the next goal even if the lift never
  confirmed reaching target, i.e. moving with the dolly only partially
  lifted. Fixed to abort the mission if the lift doesn't confirm in time.

## Not done yet

- Real map + waypoint coordinates are project-specific — replace the
  placeholder values in `pick_place_goals.yaml` before reusing this.
- No rigid attachment between the lift and dolly (friction-only) — a
  dynamically created/destroyed physics fixed joint at pick/place time would
  be a more robust fix than turn-speed limits and friction tuning alone.
- No obstacle-awareness of the dolly itself during transport or the retreat
  move.

---

## Demo videos

<!--
  Embed videos here once recorded. GitHub READMEs don't reliably autoplay
  local video files, so the common patterns are:

  1. GIF preview + link to full video:
     ![demo](docs/demo.gif)
     [Watch the full demo on YouTube](https://youtube.com/...)

  2. Thumbnail image linking out to YouTube:
     [![Watch the demo](docs/thumbnail.png)](https://youtube.com/watch?v=...)

  3. Upload the .mp4 directly into a GitHub issue/PR comment (drag-and-drop),
     copy the generated githubusercontent.com URL it gives you, then use
     that URL directly in the README - GitHub will render it as an inline
     playable video.
-->

### 1. Map generation (Occupancy Map) + localization
*(video link here)*

### 2. Autonomous navigation to a goal
*(video link here)*

### 3. Smooth lift control
*(video link here)*

### 4. Full pick-and-place mission (dashboard-triggered)
*(video link here)*

---

## License

*(add your license here, e.g. MIT/Apache-2.0)*