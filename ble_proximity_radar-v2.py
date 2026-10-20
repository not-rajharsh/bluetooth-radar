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
    python ble_proximity_radar.py --names-file ~/my_names.json   # custom name store

    Mouse wheel over the radar changes the radar range (2 - 50 m).
    Double-click a blip (or right-click > Rename) to give it a custom name.
    The "Controls" panel (View menu, Ctrl+B) holds the smart filter and the
    live RSSI calibration sliders; both are off until you tick their boxes.

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

5. Smart filter (toggleable). Hides anonymous devices (no advertised name,
   or a "name" that is just the MAC address, which BlueZ fabricates) and
   devices whose smoothed RSSI is below a threshold. It is applied at *display*
   time on the GUI thread, never in the scanner: hidden devices keep being
   tracked, so un-ticking the filter brings them back instantly with their
   history intact. A 3 dB hysteresis band stops devices hovering at the
   threshold from flickering, and blips fade in/out instead of popping.

6. Live calibration (toggleable). Base power (RSSI @ 1 m) and the path-loss
   exponent are held in an immutable RadarSettings snapshot. The GUI swaps in
   a new snapshot on every slider move; distances are recomputed from the
   *smoothed RSSI* every frame, so changes take effect immediately and apply
   to every device, with no need to re-scan.

7. Custom names. DeviceNameStore keeps {address: name} in a JSON file.
   - All access to the dict is lock-protected.
   - Disk writes happen on a dedicated daemon writer thread that coalesces
     bursts of edits and writes atomically (temp file + os.replace), so a
     rename never blocks the GUI and a crash can't leave a half-written file.
   - The store is flushed synchronously on window close.
   Caveat: many phones/trackers use *rotating random* addresses; a name saved
   for such a device stops matching when the address rotates. Fixed-address
   hardware (beacons, sensors, most wearables/speakers) persists fine.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import random
import re
import signal
import statistics
import sys
import threading
import time
import zlib
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
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
    from PyQt6.QtCore import QObject, QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
    from PyQt6.QtGui import (
        QBrush,
        QColor,
        QConicalGradient,
        QFont,
        QFontMetrics,
        QKeySequence,
        QPainter,
        QPen,
        QPixmap,
        QRadialGradient,
    )
    from PyQt6.QtWidgets import (
        QApplication,
        QCheckBox,
        QDockWidget,
        QGroupBox,
        QHBoxLayout,
        QInputDialog,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMenu,
        QPushButton,
        QSlider,
        QVBoxLayout,
        QWidget,
    )
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

# Smart filter
DEFAULT_MIN_RSSI = -85        # dBm; weaker devices are hidden when the filter is on
FILTER_RSSI_RANGE = (-100, -40)
FILTER_HYSTERESIS_DB = 3.0    # a visible device is hidden only below (threshold - this)

# Calibration slider ranges
TX_POWER_RANGE = (-90, -30)   # dBm @ 1 m
PATH_LOSS_RANGE = (1.0, 6.0)  # exponent n (slider resolution 0.1)

# Custom names
NAMES_FILE_NAME = "ble_device_names.json"
DEMO_NAMES_FILE_NAME = "ble_device_names_demo.json"   # keeps fake demo MACs out of the real file
MAX_CUSTOM_NAME_LEN = 64
HIT_RADIUS_PX = 16.0          # click tolerance around a blip


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


def rssi_to_distance(
    rssi: float,
    tx_power: float = TX_POWER_AT_1M,
    path_loss: float = PATH_LOSS_EXPONENT,
) -> float:
    """Log-distance path-loss model -> estimated distance in metres."""
    path_loss = max(0.5, path_loss)                      # guard against /0
    d = 10.0 ** ((tx_power - rssi) / (10.0 * path_loss))
    return max(MIN_DISTANCE_M, d)


@dataclass(frozen=True)
class RadarSettings:
    """
    Immutable snapshot of the user-adjustable display settings.

    Immutability is what makes this thread-safe without locks: the GUI builds
    a new snapshot on every control change and *replaces* the reference
    (an atomic operation). Any reader holds a consistent, never-mutating view.
    """

    filter_enabled: bool = False
    hide_anonymous: bool = True
    min_rssi: float = float(DEFAULT_MIN_RSSI)
    calibration_enabled: bool = False
    tx_power: float = TX_POWER_AT_1M
    path_loss: float = PATH_LOSS_EXPONENT

    def distance_params(self) -> Tuple[float, float]:
        """(tx_power, path_loss) to use: user values only while calibration is on."""
        if self.calibration_enabled:
            return self.tx_power, self.path_loss
        return TX_POWER_AT_1M, PATH_LOSS_EXPONENT


_MAC_LIKE = re.compile(r"^([0-9A-Fa-f]{2}[:\-]){5}[0-9A-Fa-f]{2}$")


def looks_like_address(name: str) -> bool:
    """True for placeholder 'names' that are really a MAC (BlueZ does this)."""
    return bool(_MAC_LIKE.match(name.strip()))


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
    custom_name: str = ""             # user-assigned (persisted by DeviceNameStore)
    shown: bool = True                # passes the smart filter (with hysteresis)
    vis_alpha: float = 1.0            # eased 0..1 so filtering fades, not pops

    def add_samples(self, name: str, rssis: List[float], now: float) -> None:
        if name:                      # names often appear only in some packets
            self.name = name
        for r in rssis:
            self.smoothed_rssi = self.rssi_filter.update(r)
        self.last_seen = now

    def update_distance(self, tx_power: float, path_loss: float) -> None:
        """Recompute the distance from the smoothed RSSI (live calibration)."""
        self.target_dist = rssi_to_distance(self.smoothed_rssi, tx_power, path_loss)
        if not self.initialised:      # first sighting: appear in place
            self.display_dist = self.target_dist
            self.initialised = True

    @property
    def is_anonymous(self) -> bool:
        """No custom name and no real advertised name."""
        if self.custom_name:
            return False
        return not self.name or looks_like_address(self.name)

    @property
    def label(self) -> str:
        return self.custom_name or self.name or self.address


def stable_bearing(address: str) -> float:
    """Deterministic 0-360 degree bearing derived from the device address."""
    return (zlib.crc32(address.encode("utf-8")) % 3600) / 10.0


# --------------------------------------------------------------------------- #
# Persistent custom device names
# --------------------------------------------------------------------------- #
class DeviceNameStore(QObject):
    """
    Thread-safe {address: custom name} map persisted to a JSON file.

    * `_lock` guards the dict and the "save needed" flag.
    * Disk I/O runs only on a private daemon writer thread. Edits just flip a
      flag and wake it, so the GUI never waits on the filesystem, and a burst
      of edits collapses into one write.
    * Writes are atomic (temp file + fsync + os.replace).
    * close() performs a final synchronous flush; call it on app exit.

    Signals
        changed()            names were edited (emitted on the editing thread)
        save_failed(str)     writing the file failed (emitted from the writer
                             thread; Qt queues it onto the GUI thread)
    """

    changed = pyqtSignal()
    save_failed = pyqtSignal(str)

    def __init__(self, path: Path, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self.path = Path(path).expanduser()
        self.load_warning = ""
        self._lock = threading.Lock()
        self._names: Dict[str, str] = {}
        self._save_needed = False
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._load()
        self._writer = threading.Thread(
            target=self._writer_loop, name="device-name-writer", daemon=True
        )
        self._writer.start()

    # ------------------------------------------------------------ public API
    @staticmethod
    def _key(address: str) -> str:
        return address.strip().upper()

    @staticmethod
    def _clean(name: str) -> str:
        return " ".join(name.split())[:MAX_CUSTOM_NAME_LEN]

    def get(self, address: str) -> str:
        with self._lock:
            return self._names.get(self._key(address), "")

    def count(self) -> int:
        with self._lock:
            return len(self._names)

    def set(self, address: str, name: str) -> None:
        """Assign a name; an empty/blank name removes the entry."""
        key, clean = self._key(address), self._clean(name)
        with self._lock:
            if clean:
                if self._names.get(key) == clean:
                    return
                self._names[key] = clean
            elif self._names.pop(key, None) is None:
                return
            self._save_needed = True
        self._wake.set()
        self.changed.emit()

    def remove(self, address: str) -> None:
        self.set(address, "")

    def close(self) -> None:
        """Flush pending edits to disk and stop the writer (blocking, short)."""
        self._stop.set()
        self._wake.set()
        self._writer.join(timeout=3.0)

    # ------------------------------------------------------------ persistence
    def _load(self) -> None:
        try:
            raw = self.path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return
        except OSError as exc:
            self.load_warning = f"Could not read {self.path.name}: {exc}"
            return
        try:
            data = json.loads(raw)
            if not isinstance(data, dict):
                raise ValueError("top level is not an object")
            entries = data["names"] if isinstance(data.get("names"), dict) else data
            loaded = {
                self._key(k): self._clean(v)
                for k, v in entries.items()
                if isinstance(k, str) and isinstance(v, str) and self._clean(v)
            }
        except (ValueError, AttributeError) as exc:
            # Don't silently overwrite a file we could not understand: keep a copy.
            backup = self.path.with_suffix(self.path.suffix + ".corrupt")
            try:
                os.replace(self.path, backup)
                self.load_warning = f"{self.path.name} was unreadable ({exc}); saved as {backup.name}"
            except OSError:
                self.load_warning = f"{self.path.name} is unreadable ({exc}); it will be overwritten"
            return
        with self._lock:
            self._names = loaded

    def _writer_loop(self) -> None:
        while True:
            self._wake.wait()
            self._wake.clear()
            while True:                                   # drain until nothing is pending
                with self._lock:
                    if not self._save_needed:
                        break
                    self._save_needed = False
                    snapshot = dict(self._names)
                try:
                    self._write(snapshot)
                except OSError as exc:
                    self.save_failed.emit(f"Could not save custom names to {self.path}: {exc}")
            if self._stop.is_set():
                return

    def _write(self, snapshot: Dict[str, str]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump({"version": 1, "names": snapshot}, fh, indent=2, ensure_ascii=False, sort_keys=True)
            fh.write("\n")
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, self.path)


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

    def __init__(
        self,
        max_range: float = DEFAULT_RANGE_M,
        names: Optional[DeviceNameStore] = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.setMinimumSize(560, 560)
        self.max_range = max_range
        self.tracks: Dict[str, DeviceTrack] = {}
        self.names = names
        self._settings = RadarSettings()          # immutable; replaced, never mutated
        self._hidden_count = 0
        self._hit_targets: List[Tuple[float, float, str]] = []   # rebuilt every paint
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

        if self.names is not None:
            self.names.changed.connect(self._refresh_custom_names)

    # ------------------------------------------------------------ public API
    def update_devices(self, batch) -> None:
        """Slot: receives (address, name, [rssi...]) tuples from the scanner."""
        now = time.monotonic()
        settings = self._settings
        tx, n = settings.distance_params()
        for address, name, samples in batch:
            track = self.tracks.get(address)
            if track is None:
                track = DeviceTrack(address, name or "", stable_bearing(address))
                if self.names is not None:
                    track.custom_name = self.names.get(address)
                track.shown = False      # so the first check uses the strict threshold
                self.tracks[address] = track
                is_new = True
            else:
                is_new = False
            track.add_samples(name, samples, now)
            if is_new:                   # appear already filtered, not fading out
                track.update_distance(tx, n)
                track.shown = self._passes_filter(track, settings)
                track.vis_alpha = 1.0 if track.shown else 0.0
        if batch:
            self.set_banner("")          # data is flowing => any old error is stale

    def set_settings(self, settings: RadarSettings) -> None:
        """Slot: swap in a new immutable settings snapshot (filter / calibration)."""
        self._settings = settings

    @staticmethod
    def _passes_filter(track: DeviceTrack, s: RadarSettings) -> bool:
        """Smart filter with hysteresis (a visible device needs to drop further to hide)."""
        if not s.filter_enabled:
            return True
        if s.hide_anonymous and track.is_anonymous:
            return False
        floor = s.min_rssi - (FILTER_HYSTERESIS_DB if track.shown else 0.0)
        return track.smoothed_rssi >= floor

    def _refresh_custom_names(self) -> None:
        """Slot: names were edited; re-read them for all live tracks."""
        if self.names is None:
            return
        for address, track in self.tracks.items():
            track.custom_name = self.names.get(address)

    def rename_device(self, address: str) -> None:
        """Ask the user for a custom name (empty input clears it)."""
        if self.names is None:
            return
        track = self.tracks.get(address)
        advertised = track.name if track and track.name and not looks_like_address(track.name) else ""
        text, accepted = QInputDialog.getText(
            self,
            "Rename device",
            f"Custom name for {address}\n"
            f"Advertised name: {advertised or '(none)'}\n"
            "Leave empty to remove the custom name.",
            QLineEdit.EchoMode.Normal,
            self.names.get(address),
        )
        if accepted:
            self.names.set(address, text)

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

    def _hit_test(self, pos) -> Optional[str]:
        """Address of the blip nearest to `pos` (within HIT_RADIUS_PX), if any."""
        best, best_d2 = None, HIT_RADIUS_PX ** 2
        for x, y, address in self._hit_targets:
            d2 = (x - pos.x()) ** 2 + (y - pos.y()) ** 2
            if d2 <= best_d2:
                best, best_d2 = address, d2
        return best

    def mouseDoubleClickEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            address = self._hit_test(event.position())
            if address is not None:
                self.rename_device(address)
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def contextMenuEvent(self, event) -> None:
        address = self._hit_test(event.pos())
        if address is None or self.names is None:
            return
        menu = QMenu(self)
        rename = menu.addAction("Rename...")
        forget = menu.addAction("Remove custom name")
        forget.setEnabled(bool(self.names.get(address)))
        chosen = menu.exec(event.globalPos())
        if chosen is rename:
            self.rename_device(address)
        elif chosen is forget:
            self.names.remove(address)

    def _tick(self) -> None:
        """~60 Hz animation step: advance sweep, ease blips, expire devices."""
        now = time.monotonic()
        dt = min(0.1, now - self._last_tick)
        self._last_tick = now

        self._sweep_deg = (self._sweep_deg + SWEEP_DEG_PER_S * dt) % 360.0

        ease = 1.0 - math.exp(-EASE_RATE * dt)
        settings = self._settings                    # one consistent snapshot per frame
        tx, n = settings.distance_params()
        hidden = 0
        for address in list(self.tracks):
            track = self.tracks[address]
            if now - track.last_seen > STALE_REMOVE_S:
                del self.tracks[address]
                continue
            track.update_distance(tx, n)             # live calibration
            track.display_dist += (track.target_dist - track.display_dist) * ease
            track.shown = self._passes_filter(track, settings)
            target_alpha = 1.0 if track.shown else 0.0
            track.vis_alpha += (target_alpha - track.vis_alpha) * ease
            if not track.shown:
                hidden += 1
        self._hidden_count = hidden
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
        self._hit_targets = []

        # Nearest devices first, so they get the best label positions
        for track in sorted(self.tracks.values(), key=lambda t: t.display_dist):
            # Fade out devices we haven't heard from recently
            age = now - track.last_seen
            if age <= STALE_FADE_START_S:
                fade = 1.0
            else:
                fade = max(0.0, 1.0 - (age - STALE_FADE_START_S) / (STALE_REMOVE_S - STALE_FADE_START_S))
            fade *= track.vis_alpha                # smart-filter fade in/out
            if fade <= 0.02:
                continue

            # Position: distance -> radius, hashed bearing -> angle
            frac = track.display_dist / self.max_range
            out_of_range = frac > 1.0
            r = R * (0.97 if out_of_range else frac)
            a = math.radians(track.angle_deg)
            x, y = cx + r * math.cos(a), cy + r * math.sin(a)
            if track.shown:                        # fading-out blips aren't clickable
                self._hit_targets.append((x, y, track.address))

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

            title_color = QColor(255, 226, 140) if track.custom_name else QColor(215, 255, 232)
            title_color.setAlpha(label_alpha)       # custom names are tinted gold
            p.setPen(title_color)
            p.drawText(QPointF(rect.left(), rect.top() + 11.0), title)
            p.setPen(QColor(color.red(), color.green(), color.blue(), int(label_alpha * 0.85)))
            p.drawText(QPointF(rect.left(), rect.top() + 24.0), detail)

    def _draw_hud(self, p: QPainter) -> None:
        p.setFont(self._hud_font)
        p.setPen(QColor(120, 255, 170, 220))
        p.drawText(QPointF(16, 26), "BLE PROXIMITY RADAR")

        p.setFont(self._small_font)
        p.setPen(QColor(120, 255, 170, 160))
        visible = len(self.tracks) - self._hidden_count
        devices = f"Devices: {visible}" + (f"  ({self._hidden_count} filtered out)" if self._hidden_count else "")
        p.drawText(QPointF(16, 45), devices)
        p.drawText(QPointF(16, 60), f"Range: {format_metres(self.max_range)}  (scroll to change)")
        s = self._settings
        if s.calibration_enabled:
            p.drawText(QPointF(16, 75), f"Calibration: {s.tx_power:.0f} dBm @ 1 m,  n = {s.path_loss:.1f}")
        p.setPen(QColor(120, 255, 170, 95))
        p.drawText(
            QPointF(16, self.height() - 12),
            "Bearing is arbitrary - BLE has no direction data  |  double-click a blip to rename",
        )

        if self._banner:
            fm = QFontMetrics(self._small_font)
            text = fm.elidedText(self._banner, Qt.TextElideMode.ElideRight, self.width() - 40)
            p.setPen(QColor(255, 110, 110) if self._banner_is_error else QColor(120, 255, 170))
            p.drawText(QPointF((self.width() - fm.horizontalAdvance(text)) / 2, self.height() - 30), text)


# --------------------------------------------------------------------------- #
# Control panel (smart filter, calibration, custom-name info)
# --------------------------------------------------------------------------- #
class ControlPanel(QWidget):
    """
    Side panel. Each feature sits in a checkable group box: the tick is the
    on/off toggle and un-ticking greys out the controls. Every change emits an
    immutable RadarSettings snapshot via `settings_changed`.
    """

    settings_changed = pyqtSignal(object)

    STYLE = """
        QWidget { background: #04100c; color: #8affb8; }
        QGroupBox { border: 1px solid #0b3a28; border-radius: 6px; margin-top: 12px;
                    padding: 10px 8px 8px 8px; font-weight: bold; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
        QCheckBox:disabled, QLabel:disabled { color: #3f6b55; }
        QSlider::groove:horizontal { height: 4px; background: #0b3a28; border-radius: 2px; }
        QSlider::handle:horizontal { background: #3dffa0; width: 12px; margin: -5px 0;
                                     border-radius: 6px; }
        QSlider::handle:horizontal:disabled { background: #2c5a46; }
        QPushButton { background: #0b3a28; border: 1px solid #14633f; border-radius: 4px;
                      padding: 4px 10px; }
        QPushButton:hover { background: #14633f; }
        QPushButton:disabled { color: #3f6b55; border-color: #0b3a28; }
    """

    def __init__(self, names: Optional[DeviceNameStore] = None, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumWidth(280)
        self.setStyleSheet(self.STYLE)
        root = QVBoxLayout(self)

        # ---- 1. smart filter -------------------------------------------------
        self.filter_group = QGroupBox("Smart filter")
        self.filter_group.setCheckable(True)
        self.filter_group.setChecked(False)
        box = QVBoxLayout(self.filter_group)
        self.anon_check = QCheckBox("Hide anonymous devices")
        self.anon_check.setChecked(True)
        self.anon_check.setToolTip("Devices that advertise no name (and have no custom name)")
        box.addWidget(self.anon_check)
        self.rssi_slider, self.rssi_value = self._add_slider(
            box, "Hide signals weaker than", *FILTER_RSSI_RANGE, DEFAULT_MIN_RSSI)
        root.addWidget(self.filter_group)

        # ---- 2. live calibration --------------------------------------------
        self.calib_group = QGroupBox("RSSI calibration")
        self.calib_group.setCheckable(True)
        self.calib_group.setChecked(False)
        box = QVBoxLayout(self.calib_group)
        self.tx_slider, self.tx_value = self._add_slider(
            box, "Base power (RSSI @ 1 m)", *TX_POWER_RANGE, int(TX_POWER_AT_1M))
        self.n_slider, self.n_value = self._add_slider(
            box, "Path-loss exponent (n)",
            int(PATH_LOSS_RANGE[0] * 10), int(PATH_LOSS_RANGE[1] * 10), int(PATH_LOSS_EXPONENT * 10))
        hint = QLabel("Stand a known device 1 m away and set base power to its reading. "
                      "n: ~2 open air, 2.5-4 indoors.")
        hint.setWordWrap(True)
        box.addWidget(hint)
        reset = QPushButton("Reset to defaults")
        reset.clicked.connect(self._reset_calibration)
        box.addWidget(reset)
        root.addWidget(self.calib_group)

        # ---- 3. custom names (info only; editing happens on the radar) -------
        names_group = QGroupBox("Custom names")
        box = QVBoxLayout(names_group)
        self.names_label = QLabel()
        box.addWidget(self.names_label)
        tip = QLabel("Double-click a blip, or right-click it > Rename. "
                     "Names are saved automatically.")
        tip.setWordWrap(True)
        box.addWidget(tip)
        if names is not None:
            names_group.setToolTip(f"Stored in {names.path}")
            self._names = names
            names.changed.connect(self._refresh_names)
        else:
            self._names = None
        root.addWidget(names_group)
        root.addStretch(1)

        # ---- wiring ----------------------------------------------------------
        self.filter_group.toggled.connect(self._on_change)
        self.calib_group.toggled.connect(self._on_change)
        self.anon_check.toggled.connect(self._on_change)
        for slider in (self.rssi_slider, self.tx_slider, self.n_slider):
            slider.valueChanged.connect(self._on_change)
        self._refresh_names()
        self._update_labels()

    @staticmethod
    def _add_slider(layout: QVBoxLayout, title: str, lo: int, hi: int, value: int):
        row = QHBoxLayout()
        row.addWidget(QLabel(title))
        row.addStretch(1)
        value_label = QLabel()
        row.addWidget(value_label)
        layout.addLayout(row)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(lo, hi)
        slider.setValue(value)
        layout.addWidget(slider)
        return slider, value_label

    def current_settings(self) -> RadarSettings:
        return RadarSettings(
            filter_enabled=self.filter_group.isChecked(),
            hide_anonymous=self.anon_check.isChecked(),
            min_rssi=float(self.rssi_slider.value()),
            calibration_enabled=self.calib_group.isChecked(),
            tx_power=float(self.tx_slider.value()),
            path_loss=self.n_slider.value() / 10.0,
        )

    def _update_labels(self) -> None:
        self.rssi_value.setText(f"{self.rssi_slider.value()} dBm")
        self.tx_value.setText(f"{self.tx_slider.value()} dBm")
        self.n_value.setText(f"{self.n_slider.value() / 10.0:.1f}")

    def _on_change(self, *_args) -> None:
        self._update_labels()
        self.settings_changed.emit(self.current_settings())

    def _reset_calibration(self) -> None:
        self.tx_slider.setValue(int(TX_POWER_AT_1M))
        self.n_slider.setValue(int(PATH_LOSS_EXPONENT * 10))

    def _refresh_names(self) -> None:
        count = self._names.count() if self._names is not None else 0
        self.names_label.setText(f"{count} saved name{'s' if count != 1 else ''}")


# --------------------------------------------------------------------------- #
# Main window
# --------------------------------------------------------------------------- #
class MainWindow(QMainWindow):
    def __init__(
        self,
        demo: bool = False,
        max_range: float = DEFAULT_RANGE_M,
        names_path: Optional[Path] = None,
    ) -> None:
        super().__init__()
        self.setWindowTitle("BLE Proximity Radar" + ("  [DEMO]" if demo else ""))
        self.resize(1130, 860)
        self.setStyleSheet(
            "QMainWindow { background: #04100c; }"
            "QStatusBar { background: #020a07; color: #8affb8; }"
            "QMenuBar { background: #020a07; color: #8affb8; }"
            "QMenuBar::item:selected, QMenu::item:selected { background: #0b3a28; }"
            "QMenu { background: #06150f; color: #8affb8; border: 1px solid #0b3a28; }"
            "QMenu::item:disabled { color: #3f6b55; }"
            "QDockWidget { color: #8affb8; }"
            "QDockWidget::title { background: #06150f; padding: 4px; }"
        )

        if names_path is None:
            default_name = DEMO_NAMES_FILE_NAME if demo else NAMES_FILE_NAME
            names_path = Path(__file__).resolve().parent / default_name
        self.names = DeviceNameStore(names_path, self)
        self.names.save_failed.connect(self._on_error)       # queued from writer thread

        self.radar = RadarWidget(max_range, names=self.names)
        self.setCentralWidget(self.radar)

        self.panel = ControlPanel(self.names)
        self.panel.settings_changed.connect(self.radar.set_settings)
        self.radar.set_settings(self.panel.current_settings())
        dock = QDockWidget("Controls", self)
        dock.setWidget(self.panel)
        dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetClosable
            | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)
        toggle = dock.toggleViewAction()
        toggle.setText("Show &controls")
        toggle.setShortcut(QKeySequence("Ctrl+B"))
        self.menuBar().addMenu("&View").addAction(toggle)

        self._scanner: BleScannerThread = DemoScannerThread(self) if demo else BleScannerThread(self)
        self._scanner.batch_ready.connect(self.radar.update_devices)
        self._scanner.status_changed.connect(self._on_status)
        self._scanner.error_occurred.connect(self._on_error)
        self._scanner.start()

        if self.names.load_warning:
            print(self.names.load_warning, file=sys.stderr)
            self._on_error(self.names.load_warning)

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
        self.names.close()                        # flush any pending name edits
        event.accept()


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #
def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Real-time BLE proximity radar")
    parser.add_argument("--demo", action="store_true", help="use simulated devices (no Bluetooth needed)")
    parser.add_argument("--range", type=float, default=DEFAULT_RANGE_M, dest="max_range",
                        help="initial radar radius in metres (default: %(default)s)")
    parser.add_argument("--names-file", type=Path, default=None, metavar="PATH",
                        help=f"JSON file for custom device names (default: {NAMES_FILE_NAME} "
                             f"next to this script; {DEMO_NAMES_FILE_NAME} with --demo)")
    args, qt_args = parser.parse_known_args(argv)
    max_range = max(1.0, min(100.0, args.max_range))

    signal.signal(signal.SIGINT, signal.SIG_DFL)   # let Ctrl+C quit the Qt app
    app = QApplication([sys.argv[0], *qt_args])
    window = MainWindow(demo=args.demo, max_range=max_range, names_path=args.names_file)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
