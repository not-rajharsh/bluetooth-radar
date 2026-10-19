#!/usr/bin/env python3
"""
BLE Proximity Radar
===================

A desktop radar that scans for nearby Bluetooth Low Energy (BLE) devices in
real time, smooths their RSSI readings, estimates distance, and plots each
device as a glowing blip on a rotating-sweep radar display.

DEPENDENCIES (Python 3.9+)
--------------------------
    pip install bleak PyQt6

Platform notes
    * Windows 10/11 : works out of the box (Bluetooth must be switched on).
    * Linux         : needs BlueZ + D-Bus (`sudo apt install bluez`), and the
                      bluetooth service running (`systemctl status bluetooth`).
    * macOS         : the first run triggers a Bluetooth permission prompt for
                      your terminal / Python app. If denied, enable it under
                      System Settings > Privacy & Security > Bluetooth.
                      (macOS reports a UUID instead of a MAC address.)

USAGE
-----
    python ble_proximity_radar.py              # live scan
    python ble_proximity_radar.py --demo       # simulated devices (no hardware)
    python ble_proximity_radar.py --range 20   # initial radar radius in metres

    Mouse wheel over the radar changes the radar range (2 - 50 m).

ARCHITECTURE / DESIGN DECISIONS
-------------------------------
1. UI toolkit: PyQt6 (QPainter) rather than PyGame.
   QPainter gives hardware-friendly anti-aliasing, radial/conical gradients
   (for the glow and the sweep trail), HiDPI support, a native event loop and
   a status bar for free. PyGame would need hand-rolled gradients and its own
   window/event plumbing.

2. Threading: bleak (asyncio) runs on its own QThread with its own event
   loop; the Qt GUI thread never awaits anything.
   - The GUI can never freeze on a slow Bluetooth stack call.
   - On Windows, bleak needs a COM multithreaded apartment, but Qt initialises
     the main thread as single-threaded (STA). A separate thread avoids that
     well-known conflict.
   - Advertisements arrive at 100s per second. The scanner thread coalesces
     them into one batch every 200 ms and sends it over a Qt signal (queued
     connection => thread-safe) so the GUI is never flooded.

3. Signal conditioning (two stages):
   a) Median of the last 5 samples  -> removes single-packet spikes/outliers.
   b) Exponential moving average    -> smooths the remaining jitter.
   The smoothed RSSI is converted to metres with the log-distance path-loss
   model:  d = 10 ** ((TxPower@1m - RSSI) / (10 * n)).
   The on-screen distance is additionally eased toward its target every frame
   so blips glide instead of jumping.

4. Limits worth knowing: RSSI-based distance is an *estimate* (walls, bodies
   and antenna orientation swing it by several metres), and BLE gives no
   direction. A blip's bearing is therefore derived from a hash of its
   address: stable per device, but arbitrary.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import random
import signal
import statistics
import sys
import threading
import time
import zlib
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

# --------------------------------------------------------------------------- #
# Dependency imports with friendly failure messages
# --------------------------------------------------------------------------- #
try:
    from bleak import BleakScanner
    from bleak.exc import BleakError
except ImportError:
    sys.exit("Missing dependency 'bleak'. Install it with:  pip install bleak")

try:
    from PyQt6.QtCore import QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
    from PyQt6.QtGui import (
        QBrush,
        QColor,
        QConicalGradient,
        QFont,
        QFontMetrics,
        QPainter,
        QPen,
        QPixmap,
        QRadialGradient,
    )
    from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget
except ImportError:
    sys.exit("Missing dependency 'PyQt6'. Install it with:  pip install PyQt6")


# --------------------------------------------------------------------------- #
# Tunable constants
# --------------------------------------------------------------------------- #
# Distance model
TX_POWER_AT_1M = -59.0        # typical RSSI (dBm) measured at 1 metre
PATH_LOSS_EXPONENT = 2.5      # 2.0 = free space, 2.5-4.0 = indoors
MIN_DISTANCE_M = 0.1

# Filtering
EMA_ALPHA = 0.25              # 0..1, lower = smoother but slower to react
MEDIAN_WINDOW = 5             # samples in the outlier-rejecting median stage

# Scanner thread
FLUSH_INTERVAL_S = 0.2        # how often batches are sent to the GUI
SILENCE_WARN_S = 8.0          # warn if no advertisements for this long
SILENCE_RESTART_S = 25.0      # restart the scan session after this much silence
RETRY_DELAY_S = 4.0           # wait before retrying after an error

# Radar display
FPS = 60
SWEEP_DEG_PER_S = 110.0
DEFAULT_RANGE_M = 10.0
RANGE_STEPS_M = (2.0, 5.0, 10.0, 20.0, 30.0, 50.0)
RING_COUNT = 4
STALE_FADE_START_S = 6.0      # device starts fading after this long unseen
STALE_REMOVE_S = 16.0         # device is dropped after this long unseen
EASE_RATE = 6.0               # blip glide speed (higher = snappier)


# --------------------------------------------------------------------------- #
# Signal processing
# --------------------------------------------------------------------------- #
class RssiFilter:
    """Median-of-N followed by an exponential moving average."""

    def __init__(self, alpha: float = EMA_ALPHA, window: int = MEDIAN_WINDOW) -> None:
        self.alpha = alpha
        self._window: Deque[float] = deque(maxlen=window)
        self.value: Optional[float] = None

    def update(self, rssi: float) -> float:
        self._window.append(rssi)
        med = statistics.median(self._window)          # stage 1: spike removal
        if self.value is None:
            self.value = med
        else:
            self.value += self.alpha * (med - self.value)  # stage 2: EMA
        return self.value


def rssi_to_distance(rssi: float) -> float:
    """Log-distance path-loss model -> estimated distance in metres."""
    d = 10.0 ** ((TX_POWER_AT_1M - rssi) / (10.0 * PATH_LOSS_EXPONENT))
    return max(MIN_DISTANCE_M, d)


def format_metres(value: float) -> str:
    """Compact metre formatting: 2.5 -> '2.5 m', 10.0 -> '10 m'."""
    text = f"{value:.1f}".rstrip("0").rstrip(".")
    return f"{text} m"


@dataclass
class DeviceTrack:
    """Everything the radar needs to know about one BLE device."""

    address: str
    name: str
    angle_deg: float                  # arbitrary but stable bearing (see docs)
    rssi_filter: RssiFilter = field(default_factory=RssiFilter)
    smoothed_rssi: float = -100.0
    target_dist: float = 0.0          # metres, from filtered RSSI
    display_dist: float = 0.0         # metres, eased for smooth animation
    last_seen: float = 0.0            # time.monotonic()
    initialised: bool = False

    def add_samples(self, name: str, rssis: List[float], now: float) -> None:
        if name:                      # names often appear only in some packets
            self.name = name
        for r in rssis:
            self.smoothed_rssi = self.rssi_filter.update(r)
        self.last_seen = now
        self.target_dist = rssi_to_distance(self.smoothed_rssi)
        if not self.initialised:      # first sighting: appear in place
            self.display_dist = self.target_dist
            self.initialised = True

    @property
    def label(self) -> str:
        return self.name or self.address


def stable_bearing(address: str) -> float:
    """Deterministic 0-360 degree bearing derived from the device address."""
    return (zlib.crc32(address.encode("utf-8")) % 3600) / 10.0


# --------------------------------------------------------------------------- #
# BLE scanner thread (asyncio + bleak)
# --------------------------------------------------------------------------- #
class _ScanStalled(Exception):
    """Raised internally when the adapter stops delivering advertisements."""


class BleScannerThread(QThread):
    """
    Runs an asyncio event loop with bleak's BleakScanner on a background
    thread and publishes batched sightings to the GUI via Qt signals.

    Signals
        batch_ready(list[(address, name, [rssi, ...])])
        status_changed(str)   informational text
        error_occurred(str)   human-readable hardware/permission problem
    """

    batch_ready = pyqtSignal(object)
    status_changed = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._lock = threading.Lock()
        self._pending: Dict[str, Tuple[str, List[float]]] = {}
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._stop_event: Optional[asyncio.Event] = None
        self._stop_requested = threading.Event()
        self._last_adv = time.monotonic()

    # ------------------------------------------------------------- lifecycle
    def run(self) -> None:  # executes on the worker thread
        try:
            asyncio.run(self._main())
        except Exception as exc:  # last-resort guard: never kill the app
            self.error_occurred.emit(f"Scanner thread crashed: {exc!r}")

    def request_stop(self) -> None:
        """Thread-safe: ask the scanner loop to finish."""
        self._stop_requested.set()
        with self._lock:
            loop, event = self._loop, self._stop_event
        if loop is not None and event is not None:
            try:
                loop.call_soon_threadsafe(event.set)
            except RuntimeError:      # loop already closed
                pass

    # ------------------------------------------------------------ main loop
    async def _main(self) -> None:
        stop_event = asyncio.Event()
        with self._lock:
            self._loop = asyncio.get_running_loop()
            self._stop_event = stop_event
        if self._stop_requested.is_set():
            stop_event.set()

        while not stop_event.is_set():
            try:
                await self._scan_session(stop_event)
            except asyncio.CancelledError:
                raise
            except _ScanStalled as exc:
                self.error_occurred.emit(str(exc))
            except Exception as exc:  # BleakError, PermissionError, OSError...
                self.error_occurred.emit(self._describe_error(exc))

            if stop_event.is_set():
                break
            self.status_changed.emit(f"Retrying in {RETRY_DELAY_S:.0f} s...")
            try:
                await asyncio.wait_for(stop_event.wait(), timeout=RETRY_DELAY_S)
            except asyncio.TimeoutError:
                pass

    async def _scan_session(self, stop_event: asyncio.Event) -> None:
        """One start-scan ... stop-scan cycle. Raises on hardware problems."""
        with self._lock:
            self._pending.clear()
        self._last_adv = time.monotonic()
        self.status_changed.emit("Starting BLE scan...")

        async with self._make_scanner():
            self.status_changed.emit("Scanning for BLE devices...")
            stop_task = asyncio.ensure_future(stop_event.wait())
            flush_task = asyncio.ensure_future(self._flush_loop())
            try:
                done, _ = await asyncio.wait(
                    {stop_task, flush_task}, return_when=asyncio.FIRST_COMPLETED
                )
                if flush_task in done:
                    flush_task.result()          # propagate _ScanStalled
            finally:
                for task in (stop_task, flush_task):
                    if not task.done():
                        task.cancel()
                await asyncio.gather(stop_task, flush_task, return_exceptions=True)

    def _make_scanner(self):
        """Factory hook (overridden by the demo thread)."""
        return BleakScanner(detection_callback=self._on_detect)

    async def _flush_loop(self) -> None:
        """Every FLUSH_INTERVAL_S: hand the GUI a batch; watch for silence."""
        warned = False
        while True:
            await asyncio.sleep(FLUSH_INTERVAL_S)
            batch = self._drain()
            if batch:
                self.batch_ready.emit(batch)
                if warned:
                    self.status_changed.emit("Scanning for BLE devices...")
                    warned = False

            silence = time.monotonic() - self._last_adv
            if silence > SILENCE_RESTART_S:
                raise _ScanStalled(
                    "No Bluetooth advertisements received for "
                    f"{SILENCE_RESTART_S:.0f} s - is Bluetooth switched on? "
                    "Restarting the scan."
                )
            if silence > SILENCE_WARN_S and not warned:
                warned = True
                self.status_changed.emit(
                    "Scanning... but no advertisements yet (check that Bluetooth is on)"
                )

    # ------------------------------------------------------ data collection
    def _on_detect(self, device, adv) -> None:
        """bleak callback: runs for every received advertisement."""
        rssi = adv.rssi
        if rssi is None or rssi >= 0 or rssi <= -127:   # sentinel / invalid
            return
        name = adv.local_name or device.name or ""
        self._ingest(device.address, name, float(rssi))

    def _ingest(self, address: str, name: str, rssi: float) -> None:
        with self._lock:
            entry = self._pending.get(address)
            if entry is None:
                self._pending[address] = (name, [rssi])
            else:
                samples = entry[1]
                samples.append(rssi)
                if name:
                    self._pending[address] = (name, samples)
            self._last_adv = time.monotonic()

    def _drain(self) -> List[Tuple[str, str, List[float]]]:
        with self._lock:
            if not self._pending:
                return []
            batch = [(a, n, s) for a, (n, s) in self._pending.items()]
            self._pending = {}
        return batch

    # ---------------------------------------------------------- diagnostics
    @staticmethod
    def _describe_error(exc: BaseException) -> str:
        """Translate low-level exceptions into actionable advice."""
        text = str(exc).strip() or exc.__class__.__name__
        low = text.lower()
        if isinstance(exc, PermissionError) or any(
            k in low for k in ("permission", "not authorized", "access denied", "denied")
        ):
            return (
                "Bluetooth access was denied - grant Bluetooth permission to this "
                f"app/terminal and retry. ({text})"
            )
        if any(
            k in low
            for k in ("no bluetooth adapter", "no adapter", "adapter", "powered",
                      "turned off", "not ready", "radio", "bluetooth is")
        ):
            return f"No usable Bluetooth adapter - plug one in / switch Bluetooth on. ({text})"
        if any(k in low for k in ("dbus", "d-bus", "bluez", "system_bus_socket")):
            return (
                "Cannot reach the BlueZ/D-Bus Bluetooth service - "
                f"is the 'bluetooth' service running? ({text})"
            )
        kind = "Bluetooth error" if isinstance(exc, BleakError) else exc.__class__.__name__
        return f"{kind}: {text}"


class _FakeScanner:
    """Async context manager that fabricates advertisements (for --demo)."""

    def __init__(self, ingest) -> None:
        self._ingest = ingest
        self._task: Optional[asyncio.Task] = None

    async def __aenter__(self):
        self._task = asyncio.ensure_future(self._produce())
        return self

    async def __aexit__(self, *exc_info) -> None:
        if self._task is not None:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)

    async def _produce(self) -> None:
        devices = [
            # address, name, mean RSSI, wobble amplitude, wobble speed
            ("DE:MO:00:00:00:01", "Earbuds", -48.0, 4.0, 0.7),
            ("DE:MO:00:00:00:02", "Smart Watch", -58.0, 6.0, 0.4),
            ("DE:MO:00:00:00:03", "", -66.0, 5.0, 0.9),
            ("DE:MO:00:00:00:04", "Fitness Band", -74.0, 7.0, 0.3),
            ("DE:MO:00:00:00:05", "Thermostat", -82.0, 4.0, 0.5),
            ("DE:MO:00:00:00:06", "", -88.0, 3.0, 0.6),
            ("DE:MO:00:00:00:07", "Beacon-7F", -93.0, 5.0, 0.2),
        ]
        t = 0.0
        while True:
            await asyncio.sleep(0.05)
            t += 0.05
            for addr, name, base, amp, speed in devices:
                if random.random() < 0.6:
                    rssi = base + amp * math.sin(t * speed) + random.gauss(0.0, 3.0)
                    if random.random() < 0.03:            # occasional spike
                        rssi += random.choice((-18.0, 18.0))
                    self._ingest(addr, name, max(-120.0, min(-30.0, rssi)))


class DemoScannerThread(BleScannerThread):
    """Same pipeline as the real scanner, fed by simulated devices."""

    def _make_scanner(self):
        return _FakeScanner(self._ingest)


# --------------------------------------------------------------------------- #
# Radar widget
# --------------------------------------------------------------------------- #
class RadarWidget(QWidget):
    """Custom-painted radar: rings, rotating sweep, glowing device blips."""

    BG = QColor(4, 14, 11)
    GRID = QColor(0, 255, 130)

    def __init__(self, max_range: float = DEFAULT_RANGE_M, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(560, 560)
        self.max_range = max_range
        self.tracks: Dict[str, DeviceTrack] = {}
        self._banner = ""
        self._banner_is_error = False

        self._sweep_deg = 0.0
        self._last_tick = time.monotonic()
        self._bg_cache: Optional[QPixmap] = None
        self._bg_key: Tuple = ()

        self._hud_font = QFont(self.font())
        self._hud_font.setPointSizeF(10.0)
        self._hud_font.setBold(True)
        self._small_font = QFont(self.font())
        self._small_font.setPointSizeF(8.5)

        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000 / FPS))

    # ------------------------------------------------------------ public API
    def update_devices(self, batch) -> None:
        """Slot: receives (address, name, [rssi...]) tuples from the scanner."""
        now = time.monotonic()
        for address, name, samples in batch:
            track = self.tracks.get(address)
            if track is None:
                track = DeviceTrack(address, name or "", stable_bearing(address))
                self.tracks[address] = track
            track.add_samples(name, samples, now)
        if batch:
            self.set_banner("")          # data is flowing => any old error is stale

    def set_banner(self, text: str, error: bool = False) -> None:
        self._banner = text
        self._banner_is_error = error

    # --------------------------------------------------------------- events
    def wheelEvent(self, event) -> None:
        dy = event.angleDelta().y()
        if dy == 0:
            return
        idx = min(range(len(RANGE_STEPS_M)), key=lambda i: abs(RANGE_STEPS_M[i] - self.max_range))
        idx = max(0, idx - 1) if dy > 0 else min(len(RANGE_STEPS_M) - 1, idx + 1)
        self.max_range = RANGE_STEPS_M[idx]
        event.accept()

    def _tick(self) -> None:
        """~60 Hz animation step: advance sweep, ease blips, expire devices."""
        now = time.monotonic()
        dt = min(0.1, now - self._last_tick)
        self._last_tick = now

        self._sweep_deg = (self._sweep_deg + SWEEP_DEG_PER_S * dt) % 360.0

        ease = 1.0 - math.exp(-EASE_RATE * dt)
        for address in list(self.tracks):
            track = self.tracks[address]
            if now - track.last_seen > STALE_REMOVE_S:
                del self.tracks[address]
                continue
            track.display_dist += (track.target_dist - track.display_dist) * ease
        self.update()

    # -------------------------------------------------------------- drawing
    def _geometry(self) -> Tuple[float, float, float]:
        w, h = self.width(), self.height()
        return w / 2.0, h / 2.0, max(10.0, min(w, h) / 2.0 - 34.0)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            painter.setRenderHint(QPainter.RenderHint.TextAntialiasing)

            key = (self.width(), self.height(), self.devicePixelRatioF(), self.max_range)
            if self._bg_cache is None or key != self._bg_key:
                self._bg_cache = self._build_background()
                self._bg_key = key
            painter.drawPixmap(0, 0, self._bg_cache)

            cx, cy, radius = self._geometry()
            self._draw_sweep(painter, cx, cy, radius)
            self._draw_devices(painter, cx, cy, radius, time.monotonic())
            self._draw_hud(painter)
        finally:
            painter.end()

    def _build_background(self) -> QPixmap:
        """Static layer (rings, crosshair, ticks) cached between frames."""
        dpr = self.devicePixelRatioF()
        pm = QPixmap(max(1, int(self.width() * dpr)), max(1, int(self.height() * dpr)))
        pm.setDevicePixelRatio(dpr)
        pm.fill(self.BG)

        p = QPainter(pm)
        try:
            p.setRenderHint(QPainter.RenderHint.Antialiasing)
            p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
            cx, cy, R = self._geometry()
            center = QPointF(cx, cy)

            # Disc background with a subtle inner glow
            disc = QRadialGradient(center, R)
            disc.setColorAt(0.0, QColor(0, 60, 40, 150))
            disc.setColorAt(1.0, QColor(0, 20, 14, 220))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(disc))
            p.drawEllipse(center, R, R)

            # Range rings + labels
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setFont(self._small_font)
            for k in range(1, RING_COUNT + 1):
                r = R * k / RING_COUNT
                outer = k == RING_COUNT
                p.setPen(QPen(QColor(0, 255, 130, 150 if outer else 60), 1.6 if outer else 1.0))
                p.drawEllipse(center, r, r)
                p.setPen(QColor(0, 255, 130, 130))
                p.drawText(QPointF(cx + 5, cy - r + 12), format_metres(self.max_range * k / RING_COUNT))

            # Crosshair
            p.setPen(QPen(QColor(0, 255, 130, 45), 1.0))
            p.drawLine(QPointF(cx - R, cy), QPointF(cx + R, cy))
            p.drawLine(QPointF(cx, cy - R), QPointF(cx, cy + R))

            # Bearing ticks every 10 degrees
            for deg in range(0, 360, 10):
                a = math.radians(deg)
                inner = R - (9 if deg % 30 == 0 else 5)
                p.setPen(QPen(QColor(0, 255, 130, 110 if deg % 30 == 0 else 60), 1.0))
                p.drawLine(
                    QPointF(cx + inner * math.cos(a), cy + inner * math.sin(a)),
                    QPointF(cx + R * math.cos(a), cy + R * math.sin(a)),
                )

            # Center marker ("you")
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(120, 255, 190, 230))
            p.drawEllipse(center, 3.5, 3.5)
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(QColor(120, 255, 190, 120), 1.0))
            p.drawEllipse(center, 8.0, 8.0)
        finally:
            p.end()
        return pm

    def _draw_sweep(self, p: QPainter, cx: float, cy: float, R: float) -> None:
        """Rotating beam with a fading trail behind it."""
        center = QPointF(cx, cy)
        # Qt's conical gradient runs counter-clockwise; our sweep runs
        # clockwise, so the trail (counter-clockwise of the beam) is correct.
        grad = QConicalGradient(center, (-self._sweep_deg) % 360.0)
        head = QColor(0, 255, 130, 120)
        tail = QColor(0, 255, 130, 0)
        grad.setColorAt(0.0, head)
        grad.setColorAt(0.20, tail)
        grad.setColorAt(1.0, tail)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(grad))
        p.drawEllipse(center, R, R)

        a = math.radians(self._sweep_deg)
        p.setPen(QPen(QColor(150, 255, 200, 235), 2.0))
        p.drawLine(center, QPointF(cx + R * math.cos(a), cy + R * math.sin(a)))

    def _draw_devices(self, p: QPainter, cx: float, cy: float, R: float, now: float) -> None:
        label_fm = QFontMetrics(self._small_font)
        p.setFont(self._small_font)
        placed_labels: List[QRectF] = []      # for simple label de-collision
        block_h = 26.0                        # height of a two-line label

        # Nearest devices first, so they get the best label positions
        for track in sorted(self.tracks.values(), key=lambda t: t.display_dist):
            # Fade out devices we haven't heard from recently
            age = now - track.last_seen
            if age <= STALE_FADE_START_S:
                fade = 1.0
            else:
                fade = max(0.0, 1.0 - (age - STALE_FADE_START_S) / (STALE_REMOVE_S - STALE_FADE_START_S))
            if fade <= 0.0:
                continue

            # Position: distance -> radius, hashed bearing -> angle
            frac = track.display_dist / self.max_range
            out_of_range = frac > 1.0
            r = R * (0.97 if out_of_range else frac)
            a = math.radians(track.angle_deg)
            x, y = cx + r * math.cos(a), cy + r * math.sin(a)

            # Classic radar "ping": blip flares when the sweep passes over it
            since_sweep = (self._sweep_deg - track.angle_deg) % 360.0
            pulse = math.exp(-since_sweep / 90.0)
            brightness = (0.35 + 0.65 * pulse) * fade * (0.6 if out_of_range else 1.0)

            hue = int(180 - 150 * min(1.0, frac))            # cyan (near) -> orange (far)
            color = QColor.fromHsv(max(0, min(359, hue)), 200, 255)

            # Glow
            glow_r = 11.0 + 14.0 * pulse
            glow = QRadialGradient(QPointF(x, y), glow_r)
            c_in = QColor(color)
            c_in.setAlpha(int(210 * brightness))
            c_out = QColor(color)
            c_out.setAlpha(0)
            glow.setColorAt(0.0, c_in)
            glow.setColorAt(1.0, c_out)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(glow))
            p.drawEllipse(QPointF(x, y), glow_r, glow_r)

            # Bright core
            core = QColor(255, 255, 255, int(255 * min(1.0, brightness + 0.1)))
            p.setBrush(core)
            p.drawEllipse(QPointF(x, y), 3.2, 3.2)

            # Labels (flip to the left side near the right edge)
            title = label_fm.elidedText(track.label, Qt.TextElideMode.ElideRight, 170)
            detail = f"{format_metres(track.display_dist)} | {track.smoothed_rssi:.0f} dBm"
            width = max(label_fm.horizontalAdvance(title), label_fm.horizontalAdvance(detail))
            tx = x + 11 if x + 11 + width < self.width() - 4 else x - 11 - width
            label_alpha = int(255 * min(1.0, (0.55 + 0.45 * pulse) * fade))

            # Nudge the label down until it no longer overlaps an earlier one
            rect = QRectF(tx, y - 13.0, float(width), block_h)
            for _ in range(10):
                if any(rect.intersects(other) for other in placed_labels):
                    rect.translate(0.0, block_h)
                else:
                    break
            placed_labels.append(rect)
            if abs(rect.top() - (y - 13.0)) > 4.0:        # thin leader line
                p.setPen(QPen(QColor(color.red(), color.green(), color.blue(), int(label_alpha * 0.45)), 1.0))
                p.drawLine(QPointF(x, y), QPointF(rect.left() - 2.0, rect.top() + 8.0))

            p.setPen(QColor(215, 255, 232, label_alpha))
            p.drawText(QPointF(rect.left(), rect.top() + 11.0), title)
            p.setPen(QColor(color.red(), color.green(), color.blue(), int(label_alpha * 0.85)))
            p.drawText(QPointF(rect.left(), rect.top() + 24.0), detail)

    def _draw_hud(self, p: QPainter) -> None:
        p.setFont(self._hud_font)
        p.setPen(QColor(120, 255, 170, 220))
        p.drawText(QPointF(16, 26), "BLE PROXIMITY RADAR")

        p.setFont(self._small_font)
        p.setPen(QColor(120, 255, 170, 160))
        p.drawText(QPointF(16, 45), f"Devices: {len(self.tracks)}")
        p.drawText(QPointF(16, 60), f"Range: {format_metres(self.max_range)}  (scroll to change)")
        p.setPen(QColor(120, 255, 170, 95))
        p.drawText(QPointF(16, self.height() - 12), "Bearing is arbitrary - BLE has no direction data")

        if self._banner:
            fm = QFontMetrics(self._small_font)
            text = fm.elidedText(self._banner, Qt.TextElideMode.ElideRight, self.width() - 40)
            p.setPen(QColor(255, 110, 110) if self._banner_is_error else QColor(120, 255, 170))
            p.drawText(QPointF((self.width() - fm.horizontalAdvance(text)) / 2, self.height() - 30), text)


# --------------------------------------------------------------------------- #
# Main window
# --------------------------------------------------------------------------- #
class MainWindow(QMainWindow):
    def __init__(self, demo: bool = False, max_range: float = DEFAULT_RANGE_M) -> None:
        super().__init__()
        self.setWindowTitle("BLE Proximity Radar" + ("  [DEMO]" if demo else ""))
        self.resize(820, 860)
        self.setStyleSheet(
            "QMainWindow { background: #04100c; }"
            "QStatusBar { background: #020a07; color: #8affb8; }"
        )

        self.radar = RadarWidget(max_range)
        self.setCentralWidget(self.radar)

        self._scanner: BleScannerThread = DemoScannerThread(self) if demo else BleScannerThread(self)
        self._scanner.batch_ready.connect(self.radar.update_devices)
        self._scanner.status_changed.connect(self._on_status)
        self._scanner.error_occurred.connect(self._on_error)
        self._scanner.start()

    def _on_status(self, text: str) -> None:
        self.statusBar().setStyleSheet("")
        self.statusBar().showMessage(text)

    def _on_error(self, text: str) -> None:
        self.statusBar().setStyleSheet("QStatusBar { background: #2a0808; color: #ff8a8a; }")
        self.statusBar().showMessage(text)
        self.radar.set_banner(text, error=True)

    def closeEvent(self, event) -> None:
        """Stop the scanner cleanly before the window disappears."""
        self._scanner.request_stop()
        if not self._scanner.wait(5000):          # BLE stack hung: last resort
            self._scanner.terminate()
            self._scanner.wait(1000)
        event.accept()


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #
def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Real-time BLE proximity radar")
    parser.add_argument("--demo", action="store_true", help="use simulated devices (no Bluetooth needed)")
    parser.add_argument("--range", type=float, default=DEFAULT_RANGE_M, dest="max_range",
                        help="initial radar radius in metres (default: %(default)s)")
    args, qt_args = parser.parse_known_args(argv)
    max_range = max(1.0, min(100.0, args.max_range))

    signal.signal(signal.SIGINT, signal.SIG_DFL)   # let Ctrl+C quit the Qt app
    app = QApplication([sys.argv[0], *qt_args])
    window = MainWindow(demo=args.demo, max_range=max_range)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
