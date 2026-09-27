"""A tiny stand-in for the firmware, so the host side can be exercised with
no hardware attached.  It speaks the same line protocol over an object that
looks enough like a ``serial.Serial`` for :class:`~deltabot.controller.DeltaBot`.

Motion is instantaneous: a move completes as soon as it is issued.

Each screw has a limit switch at a fixed mechanical position (``switches``,
in steps from where the simulator started, tripped at or above it, as
positive steps drive the nut down onto it).  A move
that reaches one stops every axis there and reports ``limit n``, as the
firmware does, so homing can be exercised too.
"""

import time


class SimSerial:
    def __init__(self, speed=600.0, accel=2000.0):
        self.is_open = True
        self.pos = [0, 0, 0]
        self.tgt = [0, 0, 0]
        self.enabled = True
        self.speed = speed
        self.accel = accel
        self.limits = [-400000, 400000]
        self.switches = [20480, 20480, 20480]   # 5 mm below where it started
        self.release = 1500                  # steps past the trip point before it releases
        self.stuck = [False, False, False]   # True: reads tripped whatever, like a broken wire
        self._tripped = [False, False, False]
        self._blocking = [False, False, False]   # tripped by a move, so that way is refused
        self.spare = [1] * 8                 # D12 D13 A0..A5 levels
        self.offset = [0, 0, 0]              # step counter minus mechanical position
        self._out = ["DeltaBot 1.2 (simulated) ready"]
        self._in = ""

    # -- serial.Serial surface -------------------------------------------
    def write(self, data):
        self._in += data.decode()
        while "\n" in self._in:
            line, self._in = self._in.split("\n", 1)
            self._handle(line.strip())
        return len(data)

    def readline(self):
        if not self._out:
            return b""
        return (self._out.pop(0) + "\n").encode()

    @property
    def in_waiting(self):
        return sum(len(s) + 1 for s in self._out)

    def reset_input_buffer(self):
        self._out.clear()

    def close(self):
        self.is_open = False

    # -- protocol ---------------------------------------------------------
    def _handle(self, line):
        if not line:
            return
        cmd, args = line[0].upper(), line[1:].replace(",", " ").split()
        try:
            nums = [int(float(a)) for a in args]
        except ValueError:
            self._out.append("err bad number")
            return

        if cmd in "MR":
            if len(nums) != 3:
                return self._out.append("err need 3 values")
            tgt = nums if cmd == "M" else [p + d for p, d in zip(self.pos, nums)]
            if any(not self._ok(t, p) for t, p in zip(tgt, self.pos)):
                return self._out.append("err soft limit")
            if self._blocked(tgt):
                return
            self._move(tgt)
        elif cmd == "J":
            if len(nums) != 2 or not 0 <= nums[0] <= 2:
                return self._out.append("err need axis and steps")
            tgt = list(self.pos)
            tgt[nums[0]] += nums[1]
            if not self._ok(tgt[nums[0]], self.pos[nums[0]]):
                return self._out.append("err soft limit")
            if self._blocked(tgt):
                return
            self._move(tgt)
        elif cmd == "P":
            self._out.append("pos %d %d %d" % tuple(self.pos))
            self._out.append("ok")
        elif cmd == "?":
            self._out.append(
                "status idle pos %d %d %d tgt %d %d %d en %d speed %.2f accel %.2f limits %d %d sw %d %d %d"
                % (*self.pos, *self.tgt, int(self.enabled), self.speed, self.accel, *self.limits,
                   *(int(self._closed(i, self.pos[i])) for i in range(3)))
            )
            self._out.append("ok")
        elif cmd == "V":
            self.speed = float(args[0]); self._out.append("ok")
        elif cmd == "A":
            self.accel = float(args[0]); self._out.append("ok")
        elif cmd == "E":
            self.enabled = bool(nums[0]); self._out.append("ok")
        elif cmd == "Z":
            new = list(nums) if len(nums) == 3 else [0, 0, 0]
            self.offset = [o + n - p for o, n, p in zip(self.offset, new, self.pos)]
            self.pos = new
            self.tgt = list(self.pos); self._out.append("ok")
        elif cmd == "B":
            self.limits = nums[:2]; self._out.append("ok")
        elif cmd in "SX":
            self.tgt = list(self.pos); self._out.append("ok")
        elif cmd == "I":
            # Outputs read low, spare inputs sit high on their pull-ups, and
            # D9..D11 follow the switches (1 = tripped).
            sw = "".join(str(int(self._closed(i, self.pos[i]))) for i in range(3))
            spare = "".join(map(str, self.spare))
            self._out.append("pins 0000000" + sw + spare[:2] + " " + spare[2:])
            self._out.append("ok")
        elif cmd == "$":
            self._out.append("simulated firmware"); self._out.append("ok")
        else:
            self._out.append("err unknown command")

    def _ok(self, target, current):
        lo, hi = self.limits
        if lo == hi or lo <= target <= hi:
            return True
        return target <= current if target > hi else target >= current

    def _closed(self, axis, steps):
        if self.stuck[axis]:
            return True
        edge = self.switches[axis] - (self.release if self._tripped[axis] else 0)
        return steps - self.offset[axis] >= edge

    def _blocked(self, tgt):
        for i in range(3):
            if self._blocking[i] and tgt[i] > self.pos[i]:
                self._out.append("err limit %d" % i)
                return True
        return False

    def _move(self, tgt):
        moved = tgt != self.pos
        # The first switch the move runs into, as a fraction of the way along.
        hit, frac = None, 1.0
        for i in range(3):
            edge = self.switches[i] + self.offset[i]
            if tgt[i] > self.pos[i] and not self._closed(i, self.pos[i]) and tgt[i] >= edge:
                f = (edge - self.pos[i]) / (tgt[i] - self.pos[i])
                if f < frac:
                    hit, frac = i, f
        if hit is not None:
            tgt = [round(p + frac * (t - p)) for p, t in zip(self.pos, tgt)]
            tgt[hit] = self.switches[hit] + self.offset[hit]
        self.pos = list(tgt)
        self.tgt = list(tgt)
        if hit is not None:
            self._tripped[hit] = self._blocking[hit] = True
        for i in range(3):
            self._tripped[i] = self._closed(i, self.pos[i])
            self._blocking[i] = self._blocking[i] and self._tripped[i]
        self._out.append("ok")
        if hit is not None:
            self._out.append("limit %d" % hit)
        elif moved:
            self._out.append("done")
