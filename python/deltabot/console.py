"""Interactive terminal for the DeltaBot.

    python3 -m deltabot                # auto-detect the serial port
    python3 -m deltabot --port /dev/ttyUSB0
    python3 -m deltabot --port sim     # no hardware needed
"""

import argparse
import cmd
import shlex
import sys
import time
from typing import Optional

from .config import Geometry
from .controller import DeltaBot, DeltaBotError, list_ports
from .kinematics import common_mode


def _switch_line(states) -> str:
    return "  ".join(f"{i}:{'TRIPPED' if on else 'ok'}" for i, on in enumerate(states))


def watch_switches(bot: DeltaBot, seconds: Optional[float] = None,
                   poll: float = 0.02) -> list:
    """Print the limit switches live: one line per change, until Ctrl-C (or
    ``seconds``), then how many changes each switch made.  Nothing moves.

    Press and release each switch a few times.  With normally closed wiring a
    working switch reads ok when released and TRIPPED while pressed; one that
    reads TRIPPED throughout is not reaching GND."""
    states = bot.limit_switches()
    counts = [0, 0, 0]
    print("  watching switches (screw 0: D9, 1: D10, 2: D11) - press and release each;"
          + (" Ctrl-C to stop" if seconds is None else f" {seconds:g} s"))
    print(f"    0.00 s  {_switch_line(states)}")
    t0 = time.time()
    try:
        while seconds is None or time.time() - t0 < seconds:
            time.sleep(poll)
            now = bot.limit_switches()
            for i in range(3):
                if now[i] != states[i]:
                    counts[i] += 1
                    change = "pressed  -> TRIPPED" if now[i] else "released -> ok"
                    print(f"  {time.time() - t0:6.2f} s  switch {i} (D{9 + i}) {change:20s}"
                          f"  [{_switch_line(now)}]")
            states = now
    except KeyboardInterrupt:
        # A reply may still be on its way; drop it so the next command is clean.
        time.sleep(0.1)
        bot.serial.reset_input_buffer()
        print()
    for i in range(3):
        if counts[i]:
            verdict = f"{counts[i]} changes - working"
        elif states[i]:
            verdict = ("never changed, stuck TRIPPED - not reaching GND: check it is "
                       "connected, COM to GND, NC (not NO) to the pin")
        else:
            verdict = "never changed, stayed ok - not pressed, or its pin is shorted to GND"
        print(f"  switch {i} (D{9 + i}): {verdict}")
    return counts


# What each pin is for, as the firmware and the GRBL shield use them.
PIN_ROLES = {
    "D2": "output: dir, screw 2", "D3": "output: dir, screw 1", "D4": "output: dir, screw 0",
    "D5": "output: step, screw 2", "D6": "output: step, screw 1", "D7": "output: step, screw 0",
    "D8": "output: driver enable",
    "D9": "limit switch, screw 0", "D10": "limit switch, screw 1",
    "D11": "limit switch, screw 2",
    "D12": "spare (shield Z limit header on V3.51+)",
    "D13": "spare (Nano LED - may not read cleanly)",
    "A0": "spare (shield Abort header)", "A1": "spare (shield Hold header)",
    "A2": "spare (shield Resume header)", "A3": "spare (shield Coolant header)",
    "A4": "spare (SDA)", "A5": "spare (SCL)",
}
OUTPUT_PINS = {"D2", "D3", "D4", "D5", "D6", "D7", "D8"}


def watch_pins(bot: DeltaBot, seconds: Optional[float] = None,
               poll: float = 0.02) -> dict:
    """Print every input pin live - D9..D13 and A0..A5 - one line per change,
    until Ctrl-C (or ``seconds``), then which pins changed.  Nothing moves.

    Every input has the internal pull-up, so a pin with nothing on it reads 1.
    A normally closed switch to GND reads 0 at rest and 1 while pressed:
    press each switch and see which pin answers."""
    levels = bot.pin_levels()
    inputs = [p for p in levels if p not in OUTPUT_PINS]
    counts = {p: 0 for p in inputs}
    print("  pin levels now (1 = high; inputs are pulled up, so 0 means tied to GND):")
    for p in levels:
        print(f"    {p:>3}  {levels[p]}   {PIN_ROLES[p]}")
    grounded = [p for p in inputs if p not in ("D9", "D10", "D11") and not levels[p]]
    if grounded:
        print("  spare pins held at GND - a normally closed switch may be wired here: "
              + ", ".join(grounded))
    print("  watching inputs - press and release each switch;"
          + (" Ctrl-C to stop" if seconds is None else f" {seconds:g} s"))
    t0 = time.time()
    try:
        while seconds is None or time.time() - t0 < seconds:
            time.sleep(poll)
            now = bot.pin_levels()
            for p in inputs:
                if now[p] != levels[p]:
                    counts[p] += 1
                    print(f"  {time.time() - t0:6.2f} s  {p:>3}  {levels[p]} -> {now[p]}"
                          f"   {PIN_ROLES[p]}")
            levels = now
    except KeyboardInterrupt:
        time.sleep(0.1)
        bot.serial.reset_input_buffer()
        print()
    changed = [p for p in inputs if counts[p]]
    if changed:
        for p in changed:
            print(f"  {p:>3}: {counts[p]} changes   {PIN_ROLES[p]}")
    else:
        print("  no input pin changed")
    return counts


class DeltaBotConsole(cmd.Cmd):
    intro = "DeltaBot console.  'help' lists commands, 'status' shows the machine.\n"
    prompt = "delta> "

    def __init__(self, bot: DeltaBot):
        super().__init__()
        self.bot = bot

    # ------------------------------------------------------------- helpers
    def _floats(self, arg, count, name):
        parts = shlex.split(arg.replace(",", " "))
        if len(parts) != count:
            raise ValueError(f"{name} takes {count} value(s)")
        return [float(p) for p in parts]

    def _show(self):
        pose = self.bot.pose()
        extensions = self.bot.extensions()
        strain = common_mode(extensions)
        print(f"  {pose}   z~{self.bot.z():+.4f} mm")
        print("  screws  " + "  ".join(f"{i}:{h:+.4f} mm" for i, h in enumerate(extensions))
              + (f"   preload {strain:+.4f} mm" if abs(strain) > 1e-6 else ""))

    def onecmd(self, line):
        try:
            return super().onecmd(line)
        except (DeltaBotError, ValueError) as exc:
            print(f"error: {exc}")

    # ------------------------------------------------------------ commands
    def do_status(self, arg):
        """status - full machine state."""
        s = self.bot.status()
        print(f"  port {self.bot.conn.port}  {'MOVING' if s['moving'] else 'idle'}  "
              f"drivers {'on' if s['en'] else 'off'}")
        print(f"  steps   {s['pos'][0]} {s['pos'][1]} {s['pos'][2]}   "
              f"speed {s['speed']:.0f} steps/s   accel {s['accel']:.0f} steps/s^2")
        print(f"  limits  {s['limits'][0]} .. {s['limits'][1]} steps "
              f"(+/-{s['limits'][1] / self.bot.geometry.steps_per_mm:.2f} mm)")
        if "sw" in s:
            print(f"  switches {_switch_line(s['sw'])}")
        else:
            print("  switches  not reported - firmware older than DeltaBot 1.1")
        self._show()

    def do_pose(self, arg):
        """pose - where the platform is in the plane."""
        self._show()

    def do_xy(self, arg):
        """xy <x_mm> <y_mm> - move the platform to a position in the plane."""
        x, y = self._floats(arg, 2, "xy")
        self.bot.move_xy(x, y)
        self._show()

    def do_x(self, arg):
        """x <mm> - move to an absolute X, leaving Y where it is."""
        (value,) = self._floats(arg, 1, "x")
        self.bot.move_xy(x=value)
        self._show()

    def do_y(self, arg):
        """y <mm> - move to an absolute Y, leaving X where it is."""
        (value,) = self._floats(arg, 1, "y")
        self.bot.move_xy(y=value)
        self._show()

    def do_move(self, arg):
        """move <dx_mm> <dy_mm> - move by a relative distance in the plane."""
        dx, dy = self._floats(arg, 2, "move")
        self.bot.nudge_xy(dx, dy)
        self._show()

    def do_polar(self, arg):
        """polar <r_mm> <theta_deg> - move to a radius and direction from centre."""
        r, theta = self._floats(arg, 2, "polar")
        self.bot.move_polar(r, theta)
        self._show()

    def do_preload(self, arg):
        """preload <mm> - add common mode extension to all three screws.

        The over-constrained direction: this strains the legs against each
        other rather than moving the platform.  Keep it small."""
        (value,) = self._floats(arg, 1, "preload")
        self.bot.preload(value)
        self._show()

    def do_jog(self, arg):
        """jog <axis 0-2> <mm> - move one screw on its own.

        The stage is over-constrained, so this both moves and strains it.  Use
        it to check wiring and directions, not to position the platform."""
        axis, mm = self._floats(arg, 2, "jog")
        self.bot.jog_mm(int(axis), mm)
        self._show()

    def do_screws(self, arg):
        """screws <mm> <mm> <mm> - move all three screws to given extensions."""
        self.bot.move_extensions(self._floats(arg, 3, "screws"))
        self._show()

    def do_steps(self, arg):
        """steps <a> <b> <c> - move to absolute step positions."""
        a, b, c = self._floats(arg, 3, "steps")
        self.bot.move_steps(a, b, c)
        self._show()

    def do_home(self, arg):
        """home - send every screw back to zero, and the platform to centre."""
        self.bot.home()
        self._show()

    def do_switches(self, arg):
        """switches - watch the limit switches live until Ctrl-C.  Press and
        release each one; every change is printed.  Nothing moves."""
        watch_switches(self.bot)

    def do_pins(self, arg):
        """pins - watch every input pin live until Ctrl-C, to find which pin
        each switch is really wired to.  Nothing moves."""
        watch_pins(self.bot)

    def _axes(self, arg):
        # Also takes the IPython spelling, e.g. 'calibrate_home(2)'.
        arg = arg.strip().strip("()").replace(",", " ")
        return [int(a) for a in shlex.split(arg)] or [0, 1, 2]

    def do_calhome(self, arg):
        """calhome [screws] - from roughly neutral, home each screw in turn:
        a balanced move down onto its switch, then back up home_offset_mm
        (5 mm), which becomes that screw's zero.  'calhome 2' does screw 2
        only; the others keep their zero."""
        self.bot.find_home(self._axes(arg))
        self._show()

    do_findhome = do_calhome              # same routine, older name
    do_find_home = do_calhome             # the IPython names work here too
    do_calibrate_home = do_calhome

    def do_zero(self, arg):
        """zero - call the current position the origin."""
        self.bot.zero()
        print("  zeroed")

    def do_speed(self, arg):
        """speed <steps/s> - set the maximum step rate (600 is a safe start)."""
        (value,) = self._floats(arg, 1, "speed")
        self.bot.set_speed(value)
        print(f"  speed {value:.0f} steps/s "
              f"({value / self.bot.geometry.steps_per_mm:.4f} mm/s)")

    def do_accel(self, arg):
        """accel <steps/s^2> - set the acceleration."""
        (value,) = self._floats(arg, 1, "accel")
        self.bot.set_accel(value)
        print(f"  accel {value:.0f} steps/s^2")

    def do_limits(self, arg):
        """limits <mm> - set the firmware's per screw travel limit."""
        (value,) = self._floats(arg, 1, "limits")
        self.bot.apply_soft_limits(value)
        print(f"  soft limits +/-{value:.2f} mm of screw travel")

    def do_radius(self, arg):
        """radius <mm> - set the working radius in the plane."""
        (value,) = self._floats(arg, 1, "radius")
        self.bot.geometry.max_radius_mm = value
        print(f"  working radius {value:.3f} mm")

    def do_gain(self, arg):
        """gain [mm_per_mm] - show, set, or calibrate the lateral gain.

        With no argument it prints the current value.  With one it sets it.
        To calibrate: home, 'jog 0 1', measure how far the platform moved in
        the plane along screw 0's direction, then 'gain calibrate <measured>'."""
        parts = arg.split()
        if not parts:
            print(f"  lateral gain {self.bot.geometry.lateral_gain:+.4f} mm per mm of screw")
        elif parts[0] == "calibrate":
            measured = float(parts[1])
            screw_mm = float(parts[2]) if len(parts) > 2 else 1.0
            gain = self.bot.calibrate_gain(measured, screw_mm)
            print(f"  lateral gain {gain:+.4f} mm per mm of screw "
                  "(copy it into config.py to keep it)")
        else:
            self.bot.geometry.lateral_gain = float(parts[0])
            print(f"  lateral gain {self.bot.geometry.lateral_gain:+.4f}")

    def do_enable(self, arg):
        """enable - energise the motors."""
        self.bot.enable()
        print("  drivers on")

    def do_disable(self, arg):
        """disable - release the motors so the screws can be turned by hand."""
        self.bot.disable()
        print("  drivers off")

    def do_stop(self, arg):
        """stop - decelerate to a halt."""
        self.bot.stop()
        print("  stopped")

    def do_circle(self, arg):
        """circle [radius_mm] - walk the platform round a circle as a self test."""
        radius = self._floats(arg, 1, "circle")[0] if arg.strip() else 0.5
        self.bot.circle(radius)
        self._show()

    def do_raw(self, arg):
        """raw <command> - send a protocol line straight to the firmware."""
        for line in self.bot.command(arg):
            print("  " + line)
        print("  ok")

    def do_ports(self, arg):
        """ports - list the serial ports on this machine."""
        for device, description in list_ports():
            print(f"  {device}  {description}")

    def do_exit(self, arg):
        """exit - leave the console (the motors are released on the way out)."""
        return True

    do_quit = do_exit
    do_EOF = do_exit

    def emptyline(self):
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description="Interactive DeltaBot control terminal")
    parser.add_argument("--port", default="auto",
                        help="serial port, 'auto' to detect, or 'sim' for the simulator")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--gain", type=float, default=None,
                        help="lateral gain, mm of platform motion per mm of screw")
    parser.add_argument("--radius", type=float, default=None,
                        help="working radius in the plane, mm")
    parser.add_argument("--microsteps", type=int, default=None,
                        help="microstepping set by the driver jumpers")
    parser.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    parser.add_argument("--switches", action="store_true",
                        help="watch the limit switches until Ctrl-C, then exit")
    parser.add_argument("--pins", action="store_true",
                        help="watch every input pin until Ctrl-C, then exit")
    args = parser.parse_args(argv)

    if args.list_ports:
        for device, description in list_ports():
            print(f"{device}\t{description}")
        return 0

    geometry = Geometry()
    if args.gain is not None:
        geometry.lateral_gain = args.gain
    if args.radius is not None:
        geometry.max_radius_mm = args.radius
    if args.microsteps is not None:
        geometry.microsteps = args.microsteps

    try:
        bot = DeltaBot(port=args.port, baud=args.baud, geometry=geometry)
    except DeltaBotError as exc:
        print(f"could not connect: {exc}", file=sys.stderr)
        return 1

    print(f"connected to {bot.conn.port}: {bot.banner}")
    if args.switches or args.pins:
        try:
            (watch_pins if args.pins else watch_switches)(bot)
        except DeltaBotError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1
        finally:
            bot.close()
        return 0
    try:
        DeltaBotConsole(bot).cmdloop()
    except KeyboardInterrupt:
        print("\ninterrupted, stopping")
        try:
            bot.stop()
        except DeltaBotError:
            pass
    finally:
        bot.close()
    return 0
