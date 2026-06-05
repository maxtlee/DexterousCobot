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

## Next steps

1. **Fix `/etc/standardbots/configuration/cyclonedds.xml` line 18.** Replace the invalid
   `<Participants>` element. Target Discovery block:
   ```xml
   <Discovery>
     <ParticipantIndex>auto</ParticipantIndex>
     <Peers>
       <Peer address="192.168.1.3"/>
     </Peers>
   </Discovery>
   ```
   (Or just drop the bad element and keep `<ParticipantIndex>auto</ParticipantIndex>`.)
2. **Confirm the file parses cleanly** — CycloneDDS reports only the *first* bad element, so
   re-run until there are no `config:` lines:
   ```bash
   ros2 topic list 2>&1 | grep -i "config:"
   ```
   Keep tracing at `<Verbosity>config</Verbosity>` so the file still parses.
3. **Re-run `ros2 topic list`.** If the bot's topics appear → done; move on to streaming
   joint trajectories.
4. **If back to "packets arrive but 0 participants":** re-enable `finest` tracing and inspect
   the SPDP announcement's advertised **unicast locators**. If they're an internal/NAT address
   (`172.x`, `127.0.0.1`, foreign subnet) rather than `192.168.1.3`, CycloneDDS discards the
   participant as unreachable — that's a robot-side bridge config issue to raise with Standard
   Bots support (provide the captured locator). Grep the trace:
   ```bash
   grep -iE "SPDP|locator|new.*participant|ignor" <trace>
   ```
5. **Daemon hygiene** when results look stale:
   ```bash
   ros2 daemon stop && ROS_DOMAIN_ID=1 ros2 topic list --no-daemon
   ```
6. Once topics are live, build the **Jacobian → joint-trajectory** velocity-control node
   (see §6).

## Standing reminders / credentials
- Robot IP/token hardcoded across scripts. Token from RO1 web UI → API settings. A 401
  "Invalid token" means it was regenerated → update every script. (Current token in tree:
  `3citgsf7-gycosg-uy730cec-4pr51c`.)
- Always run scripts with the venv interpreter: `.venv/bin/python <script>.py`.
- No trailing slash on the robot URL (see §1).

## References
- RO1 Software Overview: https://help.standardbots.com/5-software-overview.html
- ROS2 Realtime API repo: https://github.com/standardbots/ros2-realtime-api
- RO1 User Manual: https://help.standardbots.com/
