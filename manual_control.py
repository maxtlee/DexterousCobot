import sys
import time
import termios
import tty
import select
import threading

from ah_wrapper import AHSerialClient

JOINT_NAMES = ["Index", "Middle", "Ring", "Pinky", "Thb Flex", "Thb Rot"]
FINGER_NAMES = ["Index", "Middle", "Ring", "Pinky", "Thumb"]
NUM_JOINTS = 6
NUM_FINGERS = 5
OPEN_POS = [30, 30, 30, 30, 30, -50]
MOVE_RATE = 50  # degrees per second

# Position limits per joint: (min, max)
POS_LIMITS = [
    (0, 100), (0, 100), (0, 100), (0, 100),  # index, middle, ring, pinky
    (0, 100),   # thumb flexor
    (-100, 0),  # thumb rotator
]

# Key -> (joint_index, direction)
# Bottom-row keys close, top-row keys open
# Left to right: qa=pinky, ws=ring, ed=middle, rf=index, uj=thb_flex, ik=thb_rot
KEY_MAP = {
    'a': (3, +1), 'q': (3, -1),   # pinky
    's': (2, +1), 'w': (2, -1),   # ring
    'd': (1, +1), 'e': (1, -1),   # middle
    'f': (0, +1), 'r': (0, -1),   # index
    'j': (4, +1), 'u': (4, -1),   # thumb flexor
    'k': (5, -1), 'i': (5, +1),   # thumb rotator
}

# Terminals don't emit key-release events. While a key is held the terminal
# auto-repeats it, so we treat a key as "pressed" until no repeat has arrived
# within KEY_TIMEOUT seconds, at which point it is considered released.
KEY_TIMEOUT = 0.15

RUNNING = True


class ManualController:
    """Holds target positions for all 6 joints and applies key-driven
    increments each control tick. Alternates reply modes 0 and 1 so that
    position, current, velocity, and FSR feedback are all available."""

    def __init__(self, client):
        self.client = client
        self.positions = list(OPEN_POS)
        self.keys_pressed = set()
        self._alternate = True

    def open_hand(self):
        self.positions = list(OPEN_POS)

    def update(self, dt):
        for key in list(self.keys_pressed):
            if key in KEY_MAP:
                idx, direction = KEY_MAP[key]
                lo, hi = POS_LIMITS[idx]
                self.positions[idx] = max(
                    lo, min(hi, self.positions[idx] + direction * MOVE_RATE * dt)
                )

        # Alternate: mode 0 = pos+cur+fsr, mode 1 = pos+vel+fsr
        reply_mode = 0 if self._alternate else 1
        self._alternate = not self._alternate
        self.client.set_position(
            positions=list(self.positions), reply_mode=reply_mode
        )


def control_thread(controller):
    dt = 1.0 / controller.client.rate_hz
    while RUNNING:
        controller.update(dt)
        controller.client.send_command()
        time.sleep(dt)


def keyboard_thread(controller):
    """Reads single keypresses from the terminal in raw mode and maintains
    the controller's set of currently-held keys. Relies on terminal key
    auto-repeat to keep a key 'held'; a key with no repeat within
    KEY_TIMEOUT is released. Space opens the hand, Esc/Ctrl-C quits."""
    global RUNNING
    last_seen = {}
    while RUNNING:
        # Wait for input with a short timeout so we can expire stale keys.
        ready, _, _ = select.select([sys.stdin], [], [], 0.02)
        now = time.time()
        if ready:
            ch = sys.stdin.read(1)
            if ch in ('\x1b', '\x03'):  # Esc or Ctrl-C
                RUNNING = False
                break
            elif ch == ' ':
                controller.open_hand()
            elif ch in KEY_MAP:
                last_seen[ch] = now
                controller.keys_pressed.add(ch)

        # Expire keys that haven't repeated recently (key released).
        for key in list(controller.keys_pressed):
            if now - last_seen.get(key, 0) > KEY_TIMEOUT:
                controller.keys_pressed.discard(key)


def print_help():
    print("PSYONIC Manual Control (CLI)")
    print("-" * 60)
    print("  [Q/A] Pinky    [W/S] Ring     [E/D] Middle")
    print("  [R/F] Index    [U/J] Thb Flex [I/K] Thb Rot")
    print("  Top row opens, bottom row closes.")
    print("  [Space] Open hand   [Esc / Ctrl-C] Quit")
    print("-" * 60)
    print("Hold a key to move the joint; release to stop.")
    print()


def status_loop(controller):
    """Continuously prints joint targets and live feedback to the terminal
    on a single refreshing line (no graphs, no pop-ups)."""
    client = controller.client
    while RUNNING:
        hand = client.hand
        pos = hand.get_position()
        fsr = hand.get_fsr()

        t = controller.positions
        targets = (
            f"Idx={t[0]:5.1f} Mid={t[1]:5.1f} Rng={t[2]:5.1f} "
            f"Pnk={t[3]:5.1f} ThF={t[4]:5.1f} ThR={t[5]:6.1f}"
        )

        if pos:
            # Negate thumb rotator for display (matches plotting version).
            disp = [(-pos[j] if j == 5 else pos[j]) for j in range(NUM_JOINTS)]
            actual = " ".join(f"{p:5.1f}" for p in disp)
        else:
            actual = "n/a"

        if fsr:
            forces = " ".join(
                f"{FINGER_NAMES[i][:3]}={sum(fsr[i * 6:i * 6 + 6]):4.1f}"
                for i in range(NUM_FINGERS)
            )
        else:
            forces = "n/a"

        line = f"Target: {targets} | Pos: {actual} | Force(N): {forces}"
        # \r refreshes the same line; pad to clear leftover characters.
        sys.stdout.write("\r" + line[:200].ljust(200))
        sys.stdout.flush()
        time.sleep(0.1)


def main():
    global RUNNING

    client = AHSerialClient(write_thread=False)
    controller = ManualController(client)

    ctrl_thread = threading.Thread(target=control_thread, args=(controller,))
    ctrl_thread.start()
    time.sleep(0.5)

    print_help()

    # Put the terminal into cbreak mode so keypresses are delivered
    # immediately, one character at a time, without echoing them.
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)

        kb_thread = threading.Thread(
            target=keyboard_thread, args=(controller,), daemon=True
        )
        kb_thread.start()

        # Run the status display in the main thread until quit.
        status_loop(controller)
    except KeyboardInterrupt:
        pass
    finally:
        RUNNING = False
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print()  # move off the status line
        client.close()


if __name__ == "__main__":
    main()
