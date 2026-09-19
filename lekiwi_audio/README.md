# LeKiwi Audio

Spoken status announcements for LeKiwi. A node watches the robot's services and Nav2 goals and plays a short pre-rendered voice clip for each event, such as "Emergency stop enabled", "Waypoint recorded" or "Battery low".

## Contents

| Path | Purpose |
|------|---------|
| `launch/audio.launch.py` | Starts the indicator node (respawns if it dies) |
| `lekiwi_audio/indicator_node.py` | Watches services and goal status, and plays the matching clip |
| `config/phrases.yaml` | What is said for each event |
| `sounds/` | The rendered clips, one WAV per phrase |
| `lekiwi_audio/render_phrases.py`, `tts.py` | Build-time tools that render `phrases.yaml` to `sounds/` with the Kokoro voice model |

## Requirements

- ROS 2 Kilted with `rclpy`, `std_srvs`, `action_msgs` and `service_msgs`, and `aplay` (`sudo apt install alsa-utils`).
- `pip install -r requirements.txt` (PyYAML) into the interpreter that ROS uses.
- A speaker. On LeKiwi this is the reSpeaker Flex; install the udev rule as described in the [repository README](../README.md#stable-device-names-udev).
- Only to re-render the clips after editing `phrases.yaml`: `pip install -r requirements-build.txt` (`kokoro-onnx`, `onnxruntime`, `soundfile`, `numpy`). The Kokoro model files (about 116 MB) are downloaded on the first render and are not committed.

## Running

```bash
ros2 launch lekiwi_audio audio.launch.py
```

`lekiwi_bringup` starts this for you unless you pass `audio:=false`. With the robot or the simulation running, calling a service the node watches triggers its phrase, for example:

```bash
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"
```

## Configuration

**Speaker.** The node plays clips with `aplay` on the ALSA device `plughw:CARD=C16K6Ch,DEV=0`, which is the reSpeaker Flex. For another speaker, find its name with `aplay -L` and run the node directly:

```bash
ros2 run lekiwi_audio indicator_node --ros-args -p speaker_device:=plughw:CARD=<name>,DEV=0
```

**Phrases.** `config/phrases.yaml` maps each service and outcome to what is said. The defaults:

| Event | Says |
|-------|------|
| `/emergency_stop` on / off | "Emergency stop enabled" / "disabled" |
| `/twist_switch` on / off | "Autonomous mode" / "Tele-op mode" |
| `/record_waypoint` | "Waypoint recorded" (or "Error") |
| `/reset_waypoints` | "Waypoints reset" |
| `/waypoint_follow` on / off | "Waypoint following enabled" (or "No waypoints found") / "disabled" |
| `/save_map` | "Saving map" (or "Error") |
| `/battery_low`, `/battery_critical`, `/battery_full` | "Battery low", "Battery critical", "Battery full" |
| `/charger_connected` on / off | "Charger connected" / "disconnected" |
| Nav2 goal | "Goal reached", "Goal failed" or "Goal canceled" |

To change what is said, edit the text in `phrases.yaml` and rebuild the package (`colcon build --packages-select lekiwi_audio`). The build renders any new or changed phrase and removes clips that are no longer used. To announce a new service, add it under `services:` in `phrases.yaml` and to the node's `services` parameter.

## How it works

The node needs no hard-coded service names. It listens to each service's introspection topic (`<service>/_service_event`) and to the Nav2 `_action/status` topics, and plays the clip for the phrase configured for that outcome. Late joiners hear the current state of the e-stop, mode, waypoint and battery services, so a restart does not go silent. Clips are named after a short hash of the phrase text, so rendering is incremental and the node finds the right file without a manifest.

## Tests

```bash
pytest test -q
```

The tests stub out the voice model, but they still import the packages in `requirements-build.txt`, so install those first.
