"""Standalone Python driver for a Xeryon rotation stage.

This is a *plain* driver: no instrument framework yet. It exists to be the first
real data point from which a shared base class is later extracted, so the shape
of its public surface matters as much as its behaviour.

It wraps the vendor library (``Xeryon.py``) rather than replacing it. The vendor
library already owns a background serial-read thread that parses controller
telemetry into a live per-axis dict, so reading state is a cheap, non-blocking
dict lookup. What this driver adds is the one thing the vendor library does badly
for an actor/control-loop world: **non-blocking, interruptible motion**.

Resolved design question -- how is a move issued and awaited without blocking a
future control loop?

    The vendor ``setDPOS``/``step`` block by busy-polling until the position is
    reached, which would starve any future actor loop (including its STOP path).
    We instead:

      1. Disable the vendor's blocking wait globally (``DISABLE_WAITING = True``)
         so ``setDPOS``/``step`` only *send* the command and return immediately.
      2. Run completion detection in ONE driver-owned monitor thread that reads
         the vendor's already-live telemetry -- no extra serial traffic, and the
         caller's thread is never the thing doing the polling.
      3. Expose motion as non-blocking (``move_absolute`` etc. return at once),
         with ``wait_until_idle()`` blocking the caller on a ``threading.Event``
         instead of busy-looping, and ``stop()`` cancelling via that same event
         plus a hardware ``STOP=0`` so STOP interrupts an in-progress move within
         one poll tick.
      4. Fire an ``on_move_complete`` callback with the terminal result. This is
         deliberately the seed of the framework's public/PUB status channel: the
         monitor loop is the seed of the actor's Process loop, and the callback
         is the seed of "status is pushed, not polled".

    This is both "non-blocking + completion detection" and "worker thread +
    interruptible STOP" at once: the detection lives off the caller's thread, so
    a later Process loop is never blocked, and STOP always wins.

Vendor rough edges wrapped here (not inherited):
  * ``OUTPUT_TO_CONSOLE`` -> silenced; we route everything through ``logging``.
  * ``settings_default.txt`` read from disk -> parameterised (``settings_file``).
  * Stateful units -> the framework boundary always carries explicit units; the
    underlying axis is pinned to degrees and conversions are explicit.

Requires: ``pip install pyserial`` and the vendor ``Xeryon.py`` on the path.
"""

from __future__ import annotations

import logging
import os
import sys
import threading
from dataclasses import dataclass, asdict
from enum import Enum
from typing import Callable, Optional

# --- Locate and import the vendor library ------------------------------------
# The vendor library folder name contains spaces, so it is not importable as a
# package; add it to sys.path and import the module directly.
_VENDOR_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "Xeryon Python-Matlab Library")
)
if _VENDOR_DIR not in sys.path:
    sys.path.insert(0, _VENDOR_DIR)

import Xeryon as _vendor  # noqa: E402  (import after sys.path manipulation)
from Xeryon import Stage, Units  # noqa: E402  re-exported for callers

log = logging.getLogger("xeryon_rotator")


class State(Enum):
    """Driver lifecycle / motion state. Guards are enforced at runtime."""

    DISCONNECTED = "disconnected"
    IDLE = "idle"
    HOMING = "homing"
    MOVING = "moving"
    STOPPED = "stopped"   # motion cancelled by stop(); ready for the next command
    ERROR = "error"       # latched hardware fault; needs reset()/ENBL=1 to clear


@dataclass(frozen=True)
class MoveResult:
    """Terminal outcome of a motion command. Passed to ``on_move_complete``."""

    success: bool
    reason: str            # "reached" | "homed" | "stopped" | "<fault>"
    epos_deg: float
    target_deg: Optional[float]


@dataclass(frozen=True)
class Snapshot:
    """A cheap, loggable state read. Seed of the framework's ``read()`` output."""

    state: str
    epos_deg: float
    dpos_deg: float
    speed: float                 # controller-reported (encoder units/ms)
    encoder_valid: bool
    position_reached: bool
    closed_loop: bool
    at_left_end: bool
    at_right_end: bool
    fault: Optional[str]         # first active fault flag, or None
    stat: Optional[int]          # raw STAT register


# Status-bit faults checked on every monitor tick, in priority order. Each entry
# is (human-readable reason, Axis predicate method name).
_FAULT_CHECKS = (
    ("error_limit", "isErrorLimit"),
    ("safety_timeout", "isSafetyTimeoutTriggered"),
    ("position_fail", "isPositionFailTriggered"),
    ("thermal_1", "isThermalProtection1"),
    ("thermal_2", "isThermalProtection2"),
)


class XeryonRotator:
    """Drives a single Xeryon rotation axis on one serial controller.

    The vendor model is one controller object owning N axes over one port. A
    rotation stage is one axis; this class wraps exactly that. Extending to a
    multi-axis controller (rotate + X/Y on one physical controller) means
    holding several axis handles and one monitor loop per axis -- the motion
    machinery below is written to make that a small step, not a rewrite.

    Typical use::

        rot = XeryonRotator(port="COM5", stage=Stage.XRTU_30_109)
        rot.connect()
        rot.home(wait=True)
        rot.move_absolute(90.0, wait=True)     # degrees
        rot.move_relative(-15.0)               # non-blocking
        rot.wait_until_idle(timeout=10)
        rot.disconnect()
    """

    # How many fresh telemetry frames to let pass after issuing a command before
    # trusting the status flags -- clears flags left over from the prior command
    # (mirrors the vendor's __waitForUpdate settle).
    _SETTLE_FRAMES = 3
    _MONITOR_PERIOD_S = 0.01
    _DEFAULT_TOL_ENC = 10  # fallback position tolerance if PTO2/PTOL unknown

    def __init__(
        self,
        port: Optional[str] = None,
        stage: Stage = Stage.XRTU_30_109,
        letter: str = "X",
        baudrate: int = 115200,
        settings_file: Optional[str] = None,
        on_move_complete: Optional[Callable[[MoveResult], None]] = None,
    ):
        """
        :param port: Serial COM port. ``None`` triggers the vendor auto-detect.
        :param stage: Stage enum -- MUST match the physical stage (see the
            ``config.txt`` shipped with the stage). The default is a common
            rotary model and is almost certainly not right for your unit.
        :param letter: Axis letter (single-axis controllers use ``"X"``).
        :param settings_file: Path to a ``settings_default.txt``. ``None`` means
            "don't load a file" (the vendor just warns and uses controller
            defaults).
        :param on_move_complete: Called once per motion command with its
            ``MoveResult``, from the monitor thread. Keep it fast and
            non-blocking; it is the seed of the public status broadcast.
        """
        self._port = port
        self._stage = stage
        self._letter = letter
        self._baud = baudrate
        self._settings_file = settings_file
        self._on_move_complete = on_move_complete

        self._xe: Optional[_vendor.Xeryon] = None
        self._axis = None  # vendor Axis

        # Motion state, guarded by _lock.
        self._lock = threading.RLock()
        self._state = State.DISCONNECTED
        self._target_enc: Optional[int] = None
        self._issue_update_nb = 0     # axis.update_nb captured at command issue
        self._cancel = False          # stop() requested
        self._last_result: Optional[MoveResult] = None

        # Caller-facing synchronisation: set whenever the driver is not in a
        # motion state. wait_until_idle() blocks on this instead of busy-looping.
        self._idle = threading.Event()
        self._idle.set()

        # Monitor thread lifecycle.
        self._monitor: Optional[threading.Thread] = None
        self._monitor_stop = threading.Event()

    # -- Lifecycle ------------------------------------------------------------

    def connect(self) -> None:
        """Open the port, start the vendor read thread, and start monitoring.

        Corresponds to Create+Start in the framework lifecycle. Legal only from
        DISCONNECTED.
        """
        with self._lock:
            if self._state is not State.DISCONNECTED:
                raise RuntimeError(f"connect() illegal from state {self._state.value}")

        # Silence the vendor's stdout chatter; we log ourselves.
        _vendor.OUTPUT_TO_CONSOLE = False
        # Make vendor motion calls non-blocking -- we do our own awaiting.
        _vendor.DISABLE_WAITING = True

        xe = _vendor.Xeryon(self._port, self._baud)
        axis = xe.addAxis(self._stage, self._letter)
        # start() opens the port, spawns the vendor serial-read thread, pushes
        # settings, enables the axis and queries limits/tolerances.
        xe.start(external_settings_default=self._settings_file)
        axis.setUnits(Units.deg)

        self._xe = xe
        self._axis = axis

        self._monitor_stop.clear()
        self._monitor = threading.Thread(
            target=self._monitor_loop, name="xeryon-motion-monitor", daemon=True
        )
        self._monitor.start()

        self._set_state(State.IDLE)
        log.info("Connected to Xeryon rotator on %s (stage=%s)", self._port, self._stage.name)

    def disconnect(self) -> None:
        """Stop motion, stop the monitor thread, close the port.

        Corresponds to Stop+Destroy. Idempotent.
        """
        if self._state is State.DISCONNECTED:
            return
        try:
            self.stop()
        except Exception:  # best-effort during teardown
            log.exception("stop() failed during disconnect")

        self._monitor_stop.set()
        if self._monitor is not None:
            self._monitor.join(timeout=2.0)
            self._monitor = None

        if self._xe is not None:
            try:
                self._xe.stop()  # sends STOP + closes the vendor comm thread
            except Exception:
                log.exception("vendor stop() failed during disconnect")
        self._xe = None
        self._axis = None
        self._set_state(State.DISCONNECTED)
        self._idle.set()
        log.info("Disconnected from Xeryon rotator")

    def __enter__(self) -> "XeryonRotator":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.disconnect()

    # -- Motion commands (non-blocking unless wait=True) ----------------------

    def home(self, wait: bool = False, timeout: Optional[float] = 30.0) -> Optional[MoveResult]:
        """Find the encoder index. Non-blocking unless ``wait=True``.

        Homing is required before closed-loop moves are meaningful.
        """
        self._require_ready("home")
        with self._lock:
            self._axis.findIndex()  # issues INDX=0; non-blocking (DISABLE_WAITING)
            self._begin_motion(State.HOMING, target_enc=None)
        log.info("Homing started")
        return self._maybe_wait(wait, timeout)

    def move_absolute(
        self, position_deg: float, wait: bool = False, timeout: Optional[float] = 30.0
    ) -> Optional[MoveResult]:
        """Move to an absolute angle (degrees). Non-blocking unless ``wait``."""
        self._require_ready("move_absolute")
        with self._lock:
            target_enc = int(self._axis.convertUnitsToEncoder(position_deg, Units.deg))
            self._axis.setDPOS(target_enc, Units.enc)  # send only; returns at once
            self._begin_motion(State.MOVING, target_enc=target_enc)
        log.info("move_absolute -> %.4f deg (%d enc)", position_deg, target_enc)
        return self._maybe_wait(wait, timeout)

    def move_relative(
        self, delta_deg: float, wait: bool = False, timeout: Optional[float] = 30.0
    ) -> Optional[MoveResult]:
        """Move by a relative angle (degrees), handling rotary wrap-around.

        The wrap normalisation to (-180, +180] is replicated from the vendor
        ``Axis.step`` so we own the target for completion detection rather than
        waiting for the controller to echo it back.
        """
        self._require_ready("move_relative")
        with self._lock:
            target_enc = self._relative_target_enc(delta_deg)
            self._axis.setDPOS(target_enc, Units.enc)
            self._begin_motion(State.MOVING, target_enc=target_enc)
        log.info("move_relative %+.4f deg -> target %d enc", delta_deg, target_enc)
        return self._maybe_wait(wait, timeout)

    def stop(self) -> None:
        """Interrupt any in-progress motion. Idempotent; safe when already idle.

        Sends a hardware ``STOP=0`` and signals the monitor to abandon the move.
        The monitor picks this up within one poll tick and transitions to
        STOPPED (or leaves a terminal state untouched).
        """
        with self._lock:
            if self._axis is not None:
                # STOP is a not-a-setting command; sendCommand routes it straight
                # through without being stored as a setting.
                self._axis.sendCommand("STOP=0")
                self._axis.was_valid_DPOS = False
            if self._state in (State.MOVING, State.HOMING):
                self._cancel = True
        # If nothing was moving, make sure callers aren't left blocked.
        if self._state not in (State.MOVING, State.HOMING):
            self._idle.set()

    def reset(self) -> None:
        """Clear a latched fault (ERROR) by re-enabling the axis.

        Faults like error-limit / thermal / safety-timeout latch until ``ENBL=1``
        or a reset. Legal from ERROR (or IDLE, as a no-op-ish refresh).
        """
        with self._lock:
            if self._axis is None:
                raise RuntimeError("reset() illegal while disconnected")
            self._axis.sendCommand("ENBL=1")
            self._cancel = False
            self._set_state(State.IDLE)
            self._idle.set()
        log.info("Fault cleared (ENBL=1)")

    # -- Awaiting -------------------------------------------------------------

    def wait_until_idle(self, timeout: Optional[float] = None) -> Optional[MoveResult]:
        """Block the *caller* until motion finishes. Never busy-loops.

        Returns the last :class:`MoveResult`, or ``None`` if it timed out.
        """
        if self._idle.wait(timeout):
            return self._last_result
        return None

    @property
    def state(self) -> State:
        return self._state

    @property
    def is_moving(self) -> bool:
        return self._state in (State.MOVING, State.HOMING)

    # -- State read -----------------------------------------------------------

    def snapshot(self) -> Snapshot:
        """Cheap, non-blocking read of live state. Seed of framework ``read()``."""
        ax = self._axis
        if ax is None:
            return Snapshot(
                state=self._state.value, epos_deg=0.0, dpos_deg=0.0, speed=0.0,
                encoder_valid=False, position_reached=False, closed_loop=False,
                at_left_end=False, at_right_end=False, fault=None, stat=None,
            )
        stat_raw = ax.getData("STAT")
        return Snapshot(
            state=self._state.value,
            epos_deg=float(ax.getEPOS()),
            dpos_deg=float(ax.getDPOS()),
            speed=float(ax.getData("SSPD") or 0.0),
            encoder_valid=ax.isEncoderValid(),
            position_reached=ax.isPositionReached(),
            closed_loop=ax.isClosedLoop(),
            at_left_end=ax.isAtLeftEnd(),
            at_right_end=ax.isAtRightEnd(),
            fault=self._active_fault(),
            stat=int(stat_raw) if stat_raw is not None else None,
        )

    # -- Internals ------------------------------------------------------------

    def _require_ready(self, what: str) -> None:
        st = self._state
        if st in (State.IDLE, State.STOPPED):
            return
        raise RuntimeError(f"{what}() illegal from state {st.value}")

    def _begin_motion(self, new_state: State, target_enc: Optional[int]) -> None:
        """Arm a new motion. Caller holds _lock."""
        self._target_enc = target_enc
        self._issue_update_nb = int(self._axis.update_nb)
        self._cancel = False
        self._last_result = None
        self._idle.clear()
        self._set_state(new_state)

    def _relative_target_enc(self, delta_deg: float) -> int:
        """Compute the absolute encoder target of a relative rotary move.

        Replicated from vendor ``Axis.step``: reference is DPOS if the last DPOS
        was valid, else EPOS; the sum is wrapped into one revolution centred on
        zero (-180 .. +180 in degrees).
        """
        ax = self._axis
        step = ax.convertUnitsToEncoder(delta_deg, Units.deg)
        if ax.was_valid_DPOS:
            base = int(ax.getData("DPOS"))
        else:
            base = int(ax.getData("EPOS"))
        new = base + step
        epr = ax.convertUnitsToEncoder(360, Units.deg)  # encoder units / revolution
        half = epr / 2
        new = -half * (new // half % 2) + (new % half)
        return int(new)

    def _tolerance_enc(self) -> int:
        ax = self._axis
        for tag in ("PTO2", "PTOL"):
            v = ax.getSetting(tag)
            if v is not None:
                try:
                    return int(v)
                except (TypeError, ValueError):
                    pass
        return self._DEFAULT_TOL_ENC

    def _active_fault(self) -> Optional[str]:
        ax = self._axis
        if ax is None:
            return None
        for reason, method in _FAULT_CHECKS:
            if getattr(ax, method)():
                return reason
        return None

    def _monitor_loop(self) -> None:
        """Owned thread: watch telemetry, drive motion to a terminal result.

        This is the seed of the future actor Process loop. It never issues
        serial traffic of its own -- it only reads the vendor's live dict.
        """
        while not self._monitor_stop.wait(self._MONITOR_PERIOD_S):
            with self._lock:
                st = self._state
                if st not in (State.MOVING, State.HOMING):
                    continue

                # STOP requested: abandon the move.
                if self._cancel:
                    self._finish(MoveResult(
                        success=False, reason="stopped",
                        epos_deg=float(self._axis.getEPOS()), target_deg=self._target_deg(),
                    ), State.STOPPED)
                    continue

                # Let stale flags from the previous command clear first.
                settled = (int(self._axis.update_nb) - self._issue_update_nb) >= self._SETTLE_FRAMES
                if not settled:
                    continue

                # Hardware faults take priority over completion.
                fault = self._active_fault()
                if fault is not None:
                    self._finish(MoveResult(
                        success=False, reason=fault,
                        epos_deg=float(self._axis.getEPOS()), target_deg=self._target_deg(),
                    ), State.ERROR)
                    continue

                if st is State.HOMING:
                    if self._axis.isEncoderValid():
                        self._finish(MoveResult(
                            success=True, reason="homed",
                            epos_deg=float(self._axis.getEPOS()), target_deg=None,
                        ), State.IDLE)
                    elif not self._axis.isSearchingIndex():
                        # Stopped searching without finding the index.
                        self._finish(MoveResult(
                            success=False, reason="index_not_found",
                            epos_deg=float(self._axis.getEPOS()), target_deg=None,
                        ), State.ERROR)
                    continue

                # st is MOVING: complete when the controller says position
                # reached AND EPOS is within tolerance of our target.
                if self._axis.isPositionReached() and self._within_tol():
                    self._finish(MoveResult(
                        success=True, reason="reached",
                        epos_deg=float(self._axis.getEPOS()), target_deg=self._target_deg(),
                    ), State.IDLE)

    def _within_tol(self) -> bool:
        if self._target_enc is None:
            return False
        epos = int(self._axis.getData("EPOS"))
        return abs(epos - self._target_enc) <= self._tolerance_enc()

    def _target_deg(self) -> Optional[float]:
        if self._target_enc is None:
            return None
        return float(self._axis.convertEncoderUnitsToUnits(self._target_enc, Units.deg))

    def _finish(self, result: MoveResult, new_state: State) -> None:
        """Record a terminal result and release waiters. Caller holds _lock."""
        self._last_result = result
        self._cancel = False
        self._set_state(new_state)
        self._idle.set()
        log.info("Motion finished: %s", result)
        if self._on_move_complete is not None:
            # Fire outside would be cleaner, but callers are told to keep this
            # fast; guard so a bad callback can't wedge the monitor.
            try:
                self._on_move_complete(result)
            except Exception:
                log.exception("on_move_complete callback raised")

    def _set_state(self, new: State) -> None:
        old = self._state
        if old is not new:
            log.debug("state %s -> %s", old.value, new.value)
        self._state = new

    def _maybe_wait(self, wait: bool, timeout: Optional[float]) -> Optional[MoveResult]:
        if wait:
            return self.wait_until_idle(timeout)
        return None


# --- Minimal manual demo ------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s"
    )

    # NOTE: set the correct port and stage for your hardware.
    def _report(r: MoveResult) -> None:
        print("  [event] move complete:", asdict(r))

    rot = XeryonRotator(port=None, stage=Stage.XRTU_30_109, on_move_complete=_report)
    rot.connect()
    try:
        print("homing...")
        print(" ", rot.home(wait=True))
        print("move to 90 deg (blocking)...")
        print(" ", rot.move_absolute(90.0, wait=True))

        print("relative -30 deg (non-blocking) + snapshot while moving...")
        rot.move_relative(-30.0)
        import time
        time.sleep(0.2)
        print("  mid-move:", rot.snapshot())
        print("  waited:", rot.wait_until_idle(timeout=15))
    finally:
        rot.disconnect()
