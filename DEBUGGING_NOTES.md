# Debugging & Learnings — RO1 Arm, Camera, Ability Hand, ROS2 Bridge

Working notes from a debugging session. Covers the REST/SDK gotchas we hit and the
ongoing ROS2 Realtime API bring-up.

## 1. `controlMode.py` — `JSONDecodeError` on every call

**Two bugs, same symptom** (`json.decoder.JSONDecodeError: Expecting value`):

1. The script had placeholder `url='https://mybot.sb.app'` / `token='token'`. Point it at
   the real robot (`http://192.168.1.3:3000`, real token).
2. **Trailing slash on the URL.** The SDK does `host + path`, and every path already starts
   with `/`. `http://192.168.1.3:3000/` + `/api/v1/...` → `...3000//api/v1/...`, which the
   robot answers with a **404 HTML body**. The SDK then tries `json.loads()` on that HTML and
   crashes with a confusing `JSONDecodeError` instead of an HTTP error.

**Fix:** no trailing slash → `url='http://192.168.1.3:3000'`. `testConnection.py` still has a
trailing slash; it happens to work there but should be cleaned up too.

**Debug technique that found it:** call the raw request manager directly to see the real
status/body:
```python
rm = sdk.status.control._request_manager
r = rm.request('GET', '/api/v1/status/control-mode', headers=rm.json_headers())
print(r.status, r.data)   # revealed 404 + "Cannot GET //api/v1/..."
```

## 2. Control mode (`routine_editor` vs `api`)

- `sdk.status.control.get_configuration_state_control()` returns `routine_editor` or `api`.
- **The mode gates motion ownership, not all calls.** Read-only/state calls and brake on/off
  work in either mode — that's why `unbrake()` succeeds while in `routine_editor`.
- **Motion commands require `api` mode:**
  ```python
  sdk.status.control.set_configuration_control_state(
      models.RobotControlMode(kind=models.RobotControlModeEnum.Api)
  ).ok()
  ```
  Switching to `api` kicks the browser Routine Editor out of control. Switch back when done.

## 3. Camera — continuous stream

- Single frame: `sdk.camera.data.get_color_frame(...)` (base64 JPEG in a data URL).
- Continuous: `sdk.camera.data.get_camera_stream()` → `/api/v1/camera/stream/rgb`,
  `multipart/x-mixed-replace; boundary=frame`, opened with `preload_content=False`.
- The SDK gives you the raw urllib3 response; iterate it yourself. Parse JPEGs on the
  `FFD8`/`FFD9` markers (more robust than the `--frame` boundary text). Always
  `release_conn()` when done.

## 4. `cameramera.py` — saving the stream to video

- Decodes each JPEG with `cv2.imdecode` and writes to `capture.mp4` via `cv2.VideoWriter`
  (`mp4v`). Added `opencv-python` + `numpy` to `requirements.txt` and the venv.
- **VFR caveat (important):** `VideoWriter` assumes a *constant* frame rate — every
  `write()` is treated as `1/fps` of playback time, regardless of when the frame actually
  arrived. With variable-interval MJPEG arrivals the file is slightly time-warped. It's a
  fine approximation for monitoring; for accurate playback timing you need a VFR container
  with real per-frame timestamps (e.g. pipe to `ffmpeg -vsync vfr`). Set `TARGET_FPS` to the
  camera's measured average rate to minimize the warp.

## 5. `combinedtest.py` — process hangs on exit

`AHSerialClient` starts a **non-daemon read thread** (loops on `while self._reading`). The
interpreter waits for it on exit, so the process hangs after printing. **Fix:** always
`hand.close()` in a `finally` (it clears the flags, joins threads, closes serial). If you
don't need the hand in a script, don't construct it.

## 6. Fine tooltip velocity control

- REST/SDK only offer **position setpoints** — no Cartesian-velocity (twist) endpoint.
  `max_tooltip_speed` is a cap, not a setpoint. `set_arm_position_controlled` + 300 ms
  heartbeat is the closest streaming option but far too coarse for fine velocity control.
- **Use the ROS2 Realtime API** (private beta). Docs: *"With ROS you will be able to use fine
  grained controls and force feedback."* Supports Joint Trajectory Controller + Tooltip Pose.
- Implement Cartesian velocity yourself: read the **Jacobian** topic, compute
  `q̇ = J⁺ · v_desired`, integrate over your control period, stream points on the
  `joint_trajectory` topic. Topics:
  `joint_state`, `joint_trajectory`, `pose`, `pose/write`, `end_effector_imu`, `jacobian`.
- Bridge can be toggled programmatically: `sdk.ros.control.update_ros_control_state(...)`.

## 7. Fault recovery

`sdk.recovery.recover.recover()` (`POST /api/v1/recovery/recover`) returns a
`FailureStateResponse`. Inspect `.failed` / `.status` / `.failure`; some faults need manual
action (release E-stop, clear obstacle). `sdk.recovery.recover.get_status()` reads state
without attempting recovery. Re-`unbrake()` after recovering.

---

## 8. ROS2 Bridge bring-up — the main saga

**Goal:** see the robot's topics from a ROS2 Docker container running on a **Raspberry Pi**
(Ubuntu/Humble, aarch64) on the robot's subnet. Symptom: `ros2 topic list` returned nothing.

### What we ruled out (all confirmed GOOD)
- **Host networking:** `run.sh`/`run_shell.sh` use `--net=host` (my earlier `grep network`
  missed it because the flag is `--net=host`). Container sees host `eth0` = `192.168.1.2`.
- **Interface / multicast flag:** `eth0` is `UP`, `MULTICAST`, on `192.168.1.0/24`; robot at
  `192.168.1.3` is pingable.
- **RMW:** `rmw_cyclonedds_cpp` (correct).
- **Domain match:** robot transmits SPDP to `239.255.0.1:7650`; `7650 = 7400 + 250×1` ⇒
  domain 1, matching `ROS_DOMAIN_ID=1`.
- **Multicast group joined on the right NIC:** `ip maddr show eth0` lists
  `inet 239.255.0.1 users 2`.
- **Packets actually arrive in the container:**
  `tcpdump -ni eth0 udp and host 192.168.1.3` shows `192.168.1.3.35551 > 239.255.0.1.7650`.

### Key diagnostic techniques
- **`tcpdump` to bisect robot-side vs link-side:** confirmed the robot IS transmitting RTPS
  and frames reach the NIC. (Note: `tcpdump` sees frames *before* multicast-group filtering,
  so "tcpdump sees it" ≠ "the app received it" — must also confirm the group join.)
- **`ip maddr show eth0`:** confirms whether CycloneDDS joined the discovery multicast group
  on the correct interface.
- **DDS port math:** multicast discovery port = `7400 + 250 × ROS_DOMAIN_ID`. Use it to verify
  both ends are on the same domain straight from a packet capture.
- **CycloneDDS tracing** (this is what cracked it):
  ```xml
  <Tracing><Verbosity>finest</Verbosity><OutputFile>stderr</OutputFile></Tracing>
  ```

### Root cause found
Turning on tracing surfaced a **config parse error**, not a network issue:
```
config: //CycloneDDS/Domain/Discovery: Participants: unknown element
        (/etc/standardbots/configuration/cyclonedds.xml line 18)
[ERROR] rmw_create_node: failed to create domain
```
`<Participants>` is **not a valid CycloneDDS element** — introduced while hand-editing the
"explicit peers" config. The parse failure meant CycloneDDS couldn't create the domain, so no
node could be created → zero topics and the `rmw handle is invalid` cascade. The correct
element is `<Peers>` with `<Peer>` children.

**Note:** this hard parse error is *distinct* from the earlier "SPDP arrives but 0
participants" state — it was introduced by a later edit. Expect to return to the discovery
question once the config parses.

---

## 9. ROS2 Bridge bring-up — finishing the saga

After §8 fixed the cyclonedds.xml parse error, `ros2 topic list` still showed only
`/parameter_events` and `/rosout` despite the robot's `domain_bridge` participant being
discovered cleanly via SPDP. Two more bugs were stacked on top of the config one.

### Bug A — ROS bridge was disabled at the robot
`sdk.ros.status.get_ros_control_state()` returned `Disabled`. The `domain_bridge` process
runs and is discoverable via SPDP **even when ROS control is off** — it just publishes the
ROS2 builtins (`ros_discovery_info`, `rt/rosout`, `rt/parameter_events`) and forwards
nothing else from the robot's internal ROS domain.

**Enable dance: brake state matters.** A bare `update_ros_control_state(Enabled)` returned
`500 cannot_change_ros_state` when the arm was braked. Order:

```python
sdk.movement.brakes.unbrake().ok()
sdk.ros.control.update_ros_control_state(
    models.ROSControlUpdateRequest(action=models.ROSControlStateEnum.Enabled)
).ok()
```

`write_poses.py` already does it in this order; ad-hoc scripts don't.

### Bug B — robot advertises a locator we have no route to
Once the bridge was enabled, the hardware topics still didn't appear in `ros2 topic list`.
**The §8 next-step #4 hypothesis was nearly right but slightly off:** CycloneDDS does NOT
discard the participant when the advertised locator is on a foreign subnet — it picks the
**first**-listed locator and silently sends everything there.

The robot announces both of its eth0 IPs in SPDP:
```
metatraffic_unicast_locator={udp/192.168.0.134:7660, udp/192.168.1.3:7660}
default_unicast_locator  ={udp/192.168.0.134:7661, udp/192.168.1.3:7661}
```

`192.168.0.134` is on the robot's internal/service VLAN; `192.168.1.3` is the IP we ping.
**Both live on the same physical NIC on the robot** — same MAC, check with `arp -n`:
```
192.168.1.3    ether   00:e0:4c:68:05:76   C   eth0
192.168.0.134  ether   00:e0:4c:68:05:76   C   eth0
```

`finest` trace, after the bridge was enabled:
```
ddsi_rebuild_writer_addrset(110c529:...:3c2): udp/192.168.0.134:7660@2
... (×10 endpoints, all 192.168.0.134, never 192.168.1.3)
nn_xpack_send to udp/192.168.0.134:7660@2 ... ×276 packets, vs ×5 to 192.168.1.3
```

With no route for `192.168.0.0/24`, the kernel sent those packets via the wlan0 default
gateway and they were black-holed. **Fix:** add a link-scope route via eth0 so the kernel
ARPs on the wire that actually carries the robot.

```bash
sudo ip route add 192.168.0.0/24 dev eth0           # one-shot
```

Persist via NetworkManager (the eth0 connection on this host is `netplan-eth0`):
```bash
nmcli connection modify netplan-eth0 +ipv4.routes "192.168.0.0/24"
nmcli connection up netplan-eth0
```

Verify: `ip route get 192.168.0.134` reports `dev eth0`, and `ping 192.168.0.134` works
without `-I eth0`.

### Two CycloneDDS code paths pick different locators — read the right log
- `setcover` (seen in SPDP routing) optimizes which addresses to send discovery beacons
  to. It picked `192.168.1.3` here, which is what made it look like the right address
  was already being used. **It's not what matters for data.**
- `ddsi_rebuild_writer_addrset` picks the address used for per-writer DATA / HEARTBEAT /
  ACKNACK. In this version it iterates the participant's locator list and takes the first.
  Always cross-check this when asking "where is my data actually going."

### End-to-end confirmation
```
$ ./run.sh python3 ./src/read_joint_states.py
Reading joint states from robot:  bot_0sapi_a0qbmeRWQdoY3PMVAq48
Spinning...
[1.363..., -0.209..., 1.788..., 2.273..., -4.961..., 3.142...]
```

---

## Next steps

1. **Daemon hygiene** when results look stale:
   ```bash
   ros2 daemon stop && ROS_DOMAIN_ID=1 ros2 topic list --no-daemon
   ```
2. Build the **Jacobian → joint-trajectory** velocity-control node (see §6).

## Standing reminders / credentials
- Token lives in `/home/max/Documents/.robot_token` (mode 600, gitignored). All scripts
  read it via `Path("/home/max/Documents/.robot_token").read_text().strip()`. On a 401
  "Invalid token", regenerate from RO1 web UI → API settings and overwrite that one file
  — no script edits needed. (Last working token stored there as of 2026-06-05.)
- Always run scripts with the venv interpreter: `.venv/bin/python <script>.py`.
- No trailing slash on the robot URL (see §1).
- The `192.168.0.0/24` route via eth0 (see §9) is required for ROS2 topics to flow. If
  topics suddenly stop after a reboot, `ip route` and re-add per §9. Make it persistent
  via the `nmcli connection modify netplan-eth0 +ipv4.routes …` form in §9.

## References
- RO1 Software Overview: https://help.standardbots.com/5-software-overview.html
- ROS2 Realtime API repo: https://github.com/standardbots/ros2-realtime-api
- RO1 User Manual: https://help.standardbots.com/

