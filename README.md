# BLE Proximity Radar

A desktop radar that scans for nearby Bluetooth Low Energy (BLE) devices in real time, smooths their signal strength, estimates how far away they are, and plots each one as a glowing blip on a rotating-sweep radar display.

Built with **PyQt6** (rendering) and **bleak** (Bluetooth scanning). It runs as a single Python file.

> **Heads up:** distance is estimated from signal strength (RSSI), so it is approximate. Walls, bodies and antenna orientation can swing it by several metres. BLE also carries no direction information, so a blip's bearing is arbitrary (see [Limitations](#limitations)).

## Features

- **Live radar** with range rings, a rotating sweep, and blips that flare as the sweep passes over them
- **Two-stage signal smoothing** (median of the last 5 samples, then an exponential moving average) so blips glide instead of jumping
- **Smart filter** (toggleable): hide anonymous devices and weak signals
- **Live RSSI calibration** (toggleable): adjust base power and path-loss exponent while the radar runs
- **Persistent custom names**: rename any device; names are saved to a JSON file and restored on restart
- **Demo mode** that simulates devices, so you can try it with no Bluetooth hardware
- **Resilient scanning**: detects a silent or stalled adapter, restarts the scan, and shows plain-language error messages (permissions, adapter off, BlueZ/D-Bus problems)

## Requirements

- Python 3.9 or newer
- A Bluetooth adapter (not needed for `--demo`)
- Dependencies:

```bash
pip install bleak PyQt6
```

### Platform notes

| Platform | Notes |
|----------|-------|
| **Windows 10/11** | Works out of the box; Bluetooth must be switched on. |
| **Linux** | Needs BlueZ and D-Bus: `sudo apt install bluez`, and the `bluetooth` service running (`systemctl status bluetooth`). |
| **macOS** | The first run triggers a Bluetooth permission prompt for your terminal or Python app. If denied, enable it under *System Settings > Privacy & Security > Bluetooth*. macOS reports a UUID instead of a MAC address. |

## Quick start

```bash
# Try it with simulated devices (no hardware needed)
python ble_proximity_radar.py --demo

# Scan for real devices
python ble_proximity_radar.py
```

## Command-line options

| Option | Description | Default |
|--------|-------------|---------|
| `--demo` | Use simulated devices instead of real Bluetooth | off |
| `--range METRES` | Initial radar radius in metres (clamped to 1-100) | `10` |
| `--names-file PATH` | JSON file used to store custom device names | `ble_device_names.json` next to the script (`ble_device_names_demo.json` with `--demo`) |

Any other arguments are passed through to Qt.

```bash
python ble_proximity_radar.py --range 20
python ble_proximity_radar.py --names-file ~/my_ble_names.json
```

## Using the radar

| Action | Result |
|--------|--------|
| **Mouse wheel** over the radar | Change the range (steps: 2, 5, 10, 20, 30, 50 m) |
| **Double-click** a blip | Rename the device |
| **Right-click** a blip | Menu: *Rename...* / *Remove custom name* |
| **Ctrl+B** (or *View > Show controls*) | Show or hide the Controls panel |
| **Ctrl+C** in the terminal | Quit |

Blips are coloured from cyan (near) to orange (far). Devices that stop advertising fade after 6 seconds and are removed after 16 seconds. Devices beyond the current range are drawn dimmed at the outer ring.

## The Controls panel

Each feature lives in a checkable group box. The tick is the on/off toggle, and unticking it greys out the controls. Both features are **off by default**, so the app behaves like a plain radar until you enable them.

### Smart filter

Declutters the display.

- **Hide anonymous devices**: hides devices that advertise no name. A "name" that is really just the device's MAC address (which BlueZ on Linux fabricates) also counts as anonymous. Devices you have given a custom name are never treated as anonymous.
- **Hide signals weaker than**: hides devices whose smoothed RSSI is below the threshold (range -100 to -40 dBm, default -85 dBm).

Filtering happens at display time, so hidden devices keep being tracked in the background. Unticking the filter brings them back instantly. A 3 dB hysteresis band stops devices sitting right at the threshold from flickering, and blips fade in and out rather than popping. The HUD shows how many devices are currently filtered out.

### RSSI calibration

Distance is estimated with the log-distance path-loss model:

```
distance = 10 ^ ((base_power - RSSI) / (10 * n))
```

| Control | Meaning | Range | Default |
|---------|---------|-------|---------|
| **Base power (RSSI @ 1 m)** | The signal strength you would measure at exactly 1 metre | -90 to -30 dBm | -59 |
| **Path-loss exponent (n)** | How quickly the signal fades with distance | 1.0 to 6.0 | 2.5 |

Changes apply immediately to every device, with no re-scan. Unticking the group returns to the default values; the sliders remember your settings, and *Reset to defaults* restores them.

**Calibrating:** place a known device 1 m from your adapter and set *Base power* to the RSSI it shows. For *n*, use about 2 in open air and 2.5-4 indoors, then adjust until a device at a known distance lands on the right ring.

### Custom names

Give any device a friendly name. Names are saved automatically to a JSON file and shown in gold on the radar, so they are still there after you restart.

The file looks like this:

```json
{
  "version": 1,
  "names": {
    "AA:BB:CC:DD:EE:01": "Living room TV",
    "AA:BB:CC:DD:EE:02": "Desk speaker"
  }
}
```

- Addresses are stored upper-case; names are whitespace-collapsed and limited to 64 characters.
- An empty name removes the entry.
- You can edit the file by hand while the app is closed. A plain `{"address": "name"}` object is also accepted.
- If the file is corrupt, it is backed up with a `.corrupt` suffix instead of being overwritten, and a warning is shown.

## How it works

```
┌──────────────────────┐  batches every 200 ms   ┌────────────────────────────┐
│ Scanner QThread      │ ──────────────────────▶ │ GUI thread                 │
│  asyncio + bleak     │   (Qt queued signal)    │  RadarWidget (QPainter)    │
│  coalesces adverts   │                         │  filter / calibration /    │
└──────────────────────┘                         │  smoothing / animation     │
                                                 └─────────────┬──────────────┘
                                                               │ edits
                                                 ┌─────────────▼──────────────┐
                                                 │ DeviceNameStore            │
                                                 │  lock-guarded dict +       │
                                                 │  background writer thread  │
                                                 └────────────────────────────┘
```

- **Why PyQt6 rather than PyGame:** QPainter provides anti-aliasing, radial and conical gradients (for the glow and sweep trail), HiDPI support and a native event loop out of the box.
- **Threading:** bleak's asyncio loop runs on its own `QThread`, so the GUI never awaits Bluetooth calls and can't freeze. On Windows this also avoids the conflict between bleak's need for a COM multithreaded apartment and Qt's single-threaded one. Advertisements arrive at hundreds per second, so they are coalesced into one batch every 200 ms.
- **Signal conditioning:** median of the last 5 samples (removes spikes), then an exponential moving average (smooths jitter). The on-screen distance is also eased toward its target every frame.
- **Thread safety of the newer features:**
  - Filter and calibration settings are an *immutable* `RadarSettings` snapshot. The GUI replaces the reference on every change, so readers always see a consistent view without locks.
  - `DeviceNameStore` guards its data with a lock. All disk writes run on a single daemon writer thread that coalesces bursts of edits and writes atomically (temp file, `fsync`, `os.replace`). It is flushed synchronously on window close, so renaming never blocks the GUI.

## Configuration

The tunable constants are near the top of `ble_proximity_radar.py`, including:

| Constant | Purpose |
|----------|---------|
| `TX_POWER_AT_1M`, `PATH_LOSS_EXPONENT` | Default distance model |
| `EMA_ALPHA`, `MEDIAN_WINDOW` | Smoothing strength |
| `FLUSH_INTERVAL_S` | How often batches reach the GUI |
| `SILENCE_WARN_S`, `SILENCE_RESTART_S`, `RETRY_DELAY_S` | Stalled-scan handling |
| `STALE_FADE_START_S`, `STALE_REMOVE_S` | When devices fade and disappear |
| `DEFAULT_MIN_RSSI`, `FILTER_HYSTERESIS_DB` | Smart filter defaults |
| `FPS`, `SWEEP_DEG_PER_S`, `RANGE_STEPS_M` | Display behaviour |

## Troubleshooting

| Symptom | Likely cause and fix |
|---------|----------------------|
| *"Bluetooth access was denied"* | Grant Bluetooth permission to your terminal or Python app (macOS: System Settings > Privacy & Security > Bluetooth). |
| *"No usable Bluetooth adapter"* | Plug in an adapter or switch Bluetooth on. |
| *"Cannot reach the BlueZ/D-Bus Bluetooth service"* (Linux) | Install BlueZ and start the service: `sudo systemctl start bluetooth`. |
| *"No advertisements yet"* / scan keeps restarting | Bluetooth is probably off, or nothing nearby is advertising. The scan restarts automatically after 25 s of silence. |
| Radar looks empty with the filter on | The threshold may be too strict, or most nearby devices are anonymous. Lower the threshold, untick *Hide anonymous*, or check the "filtered out" count in the HUD. |
| A custom name disappeared for a phone or tracker | See [Limitations](#limitations): rotating random addresses. |
| Names aren't saved | Check the status bar for a write error; make sure the names file's folder is writable, or point `--names-file` somewhere else. |

## Limitations

- **Distance is an estimate.** RSSI is affected by walls, people, interference and antenna orientation. Use calibration for your environment, but expect errors of several metres.
- **No direction.** BLE advertisements don't say where a device is, so each blip's bearing is derived from a hash of its address: stable per device, but arbitrary.
- **Rotating addresses.** Many phones and trackers use random addresses that change periodically. A custom name saved for such a device stops matching when its address rotates. Fixed-address hardware (beacons, sensors, most wearables and speakers) persists fine.
- **macOS identifiers.** macOS reports per-app UUIDs rather than MAC addresses, so saved names are tied to that identifier.
- **Settings not persisted.** Filter and calibration settings reset on restart; only custom names are saved.
- **Single writer.** If you run several instances with the same names file, the last one to save wins.

## Project layout

```
ble_proximity_radar.py          # the whole application
ble_device_names.json           # created on first rename (custom names)
ble_device_names_demo.json      # same, but used only in --demo mode
README.md
```

## License

No license has been specified for this project. Add a `LICENSE` file if you plan to share or distribute it.
