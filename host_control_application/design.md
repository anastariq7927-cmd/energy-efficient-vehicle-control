# Design Document

## Overview

The ESP32 RC Car Controller is a data-collection platform for energy-optimization research (reinforcement learning / dynamic programming). A two-motor DC car equipped with an ESP32 NodeMCU-32S traverses a track; the space is discretized into fixed-length windows of dx mm. At the end of each window the ESP32 emits a binary observation packet to the Python host application running in Docker. The host reacts with spatially-indexed motor commands on an independent UDP channel. All energy, timing, and control data are logged per-window into CSV files for offline algorithm training.

The system deliberately separates the two UDP directions so that network jitter on one channel does not block the other. Safety is layered: a spatial stop rule (no command for the current window), a temporal watchdog (no valid command packet for 3 s), an overcurrent protection (>500 mA), and an emergency stop command each independently cut motor power and set a flag bit in the observation stream.

### Design Goals

- Exact integer round-trip: all physical quantities encoded as integers (duty in per-mille, current in mA, voltage in mV, energy in µJ, duration in µs) — no floating-point in packets.
- Reproducibility: a shared LCG pseudo-random generator and a Docker-packaged host application allow full reproduction on macOS, Linux, and Windows.
- Replaceability: the Controller interface decouples the control algorithm from transport and logging; researchers swap algorithms by changing a config entry.
- Safety-first: motors are off by default; any missing command or lost connection triggers an immediate stop.

---

## Architecture

### System Component Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                        ESP32 (NodeMCU-32S)                       │
│                                                                  │
│  ┌────────────┐  5 ms   ┌──────────────────────────────────────┐ │
│  │   AS5600   │────────▶│  Window Detection FSM                │ │
│  │  I2C 0x36  │         │  (IDLE / EPISODE_ACTIVE /            │ │
│  └────────────┘         │   STOPPED_*)                         │ │
│                         └──────────────┬─────────────────────┬─┘ │
│  ┌────────────┐  ~10ms  ┌──────────────▼──────────────────┐   │  │
│  │   INA226   │────────▶│  Per-Window Integrator          │   │  │
│  │  I2C 0x40  │         │  (current, voltage, energy)     │   │  │
│  └────────────┘         └──────────────┬────────────────┬─┘   │  │
│                                        │                │      │  │
│                         ┌──────────────▼─────────────┐  │      │  │
│                         │  Table_Commandes (16 slots) │  │      │  │
│                         └──────────────┬─────────────-┘  │      │  │
│                                        │                  │      │  │
│  ┌─────────────────────────────────────▼──────────────┐   │      │  │
│  │               UDP Tasks                             │   │      │  │
│  │  TX: Paquet_Observation → host:8888                 │◀──┘      │  │
│  │  RX: Paquet_Commande  ← host (ESP32:8889)           │          │  │
│  └─────────────────────────────────────────────────────┘          │  │
│                                                                  │
│  ┌──────────────────┐                                            │
│  │   TB6612FNG      │◀── Safety State Machine (duty clamp)      │
│  │  (dual DC motors)│                                            │
│  └──────────────────┘                                            │
└──────────────────────────────────────────────────────────────────┘
              │ UDP :8888              │ UDP :8889
              ▼                        │
┌─────────────────────────────────────▼──────────────────────────┐
│                  Application_Hôte (Docker Compose)             │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    Processus_UDP                         │  │
│  │   RX Paquet_Observation  ──▶  Flux_Observations (Redis)  │  │
│  │   TX Paquet_Commande     ◀──  Flux_Commandes (Redis)     │  │
│  │   Heartbeat type-4 every 1 s                             │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    Bus_Partagé (Redis)                   │  │
│  │   Stream: "observations"   Stream: "commandes"           │  │
│  │   Key:    "derniere_observation"                         │  │
│  │   Key:    "derniere_commande"                            │  │
│  │   Key:    "episode_id_counter"                           │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                 │
│  ┌────────────────────┐  ┌──────────────────┐                  │
│  │  Processus_Contrôle│  │  Journal_Episodes│                  │
│  │  (Controller impl) │  │  (CSV + JSON)    │                  │
│  └────────────────────┘  └──────────────────┘                  │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              Dashboard (Plotly Dash)                     │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │            Simulateur (optional profile)                 │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
```

### High-Level Data Flow

```
ESP32 window end
       │
       ▼
Enregistrement_Fenêtre (N, duree_us, courant_moy_mA, tension_moy_mV, energie_fenetre_uJ)
       │  packed into
       ▼
Paquet_Observation (type=0, up to 3 records, CRC16)
       │  UDP :8888
       ▼
Processus_UDP ──validates CRC, deduplicates by (episode_id, N)──▶ Flux_Observations (Redis)
                                                                        │
                          ┌─────────────────────────────────────────────┤
                          │                                             │
                          ▼                                             ▼
                 Processus_Contrôle                           Journal_Episodes
                 (blocking XREAD)                             (separate consumer group)
                 Controller.on_observation()                  writes CSV row
                          │
                          ▼
                 (N_cmd, duty_pm) list ──▶ Flux_Commandes (Redis)
                                                   │
                                                   ▼
                                          Processus_UDP
                                          packs Paquet_Commande (type=0)
                                                   │  UDP :8889
                                                   ▼
                                                ESP32
                                          stores in Table_Commandes
                                          applies duty at window N entry
```

---

## Components and Interfaces

### ESP32 Firmware Components

#### 1. Main Loop and Task Structure

The firmware runs FreeRTOS (via Arduino/ESP-IDF). Three concurrent tasks plus the main loop:

| Task | Core | Period | Responsibility |
|------|------|--------|----------------|
| `task_as5600` | 1 | 5 ms | Read AS5600, accumulate forward distance |
| `task_ina226` | 1 | ~10 ms (~100 Hz) | Read INA226, accumulate current/voltage samples |
| `task_udp_rx` | 0 | event-driven | Receive Paquet_Commande, update Table_Commandes |
| `task_udp_tx` | 0 | event-driven | Send Paquet_Observation when window closes or 250 ms heartbeat |

A shared mutex protects the window state (current window N, integration accumulators, Table_Commandes). The safety state machine runs inline within `task_as5600` at every window boundary and within `task_udp_rx` at every packet reception.

#### 2. Window Detection FSM

```
States: IDLE | EPISODE_ACTIVE | STOPPED_WINDOW_RULE | STOPPED_WATCHDOG_TEMPORAL
        | STOPPED_OVERCURRENT | STOPPED_EMERGENCY | STOPPED_DISTANCE

IDLE ──[type-1 received]──▶ EPISODE_ACTIVE
  • reset N=0, clear Table_Commandes, record dx_mm, distance_max_mm, episode_id
  • init motors STBY=LOW, PWM=0

EPISODE_ACTIVE:
  • On every 5 ms AS5600 read:
      accumulated_mm += forward_delta (zero-crossing handled, hysteresis applied)
      if accumulated_mm >= dx_mm:
          [check Table_Commandes for N]
          if entry[N] present:
              apply duty_pm → TB6612FNG (clamped to duty_max)
              update derniere_commande_utilisee
              emit Enregistrement_Fenêtre, transition N → N+1
              remove entry[N] from Table_Commandes
          else:
              → STOPPED_WINDOW_RULE (set bit2=1, STBY=LOW, PWM=0)

  • On entry of window N (after advancing N):
      if cumulative_distance_mm >= distance_max_mm:
          → STOPPED_DISTANCE (set bit7=1, STBY=LOW, PWM=0)

  • Watchdog check (runs on timer, independent of AS5600 task):
      if now - last_valid_cmd_time > watchdog_temporel_s:
          → STOPPED_WATCHDOG_TEMPORAL (set bit3=1, STBY=LOW, PWM=0)

  • INA226 overcurrent (inline in task_ina226):
      if current_mA > overcurrent_threshold_mA:
          → STOPPED_OVERCURRENT (set bit4=1, STBY=LOW, PWM=0)

  • type-2 received → IDLE (stop motors, cease emitting observations)
  • type-3 received → STOPPED_EMERGENCY (set STBY=LOW, PWM=0)

STOPPED_WINDOW_RULE:
  • type-1 received → EPISODE_ACTIVE (only this resets the stop)
  • type-0 alone → remain in STOPPED_WINDOW_RULE
  • emits Paquet_Vie_ESP32 every 250 ms with bit2=1

STOPPED_WATCHDOG_TEMPORAL:
  • type-0 received for current or future window → EPISODE_ACTIVE (clear bit3)
  • type-1 received → EPISODE_ACTIVE
  • emits Paquet_Vie_ESP32 every 250 ms with bit3=1

STOPPED_OVERCURRENT:
  • type-1 received → EPISODE_ACTIVE
  • emits Paquet_Vie_ESP32 every 250 ms with bit4=1

STOPPED_EMERGENCY:
  • type-1 received → EPISODE_ACTIVE
  • emits Paquet_Vie_ESP32 every 250 ms

STOPPED_DISTANCE:
  • type-1 received → EPISODE_ACTIVE (starts new episode)
  • emits Paquet_Vie_ESP32 every 250 ms with bit7=1
```

#### 3. AS5600 Reader

- Read raw 12-bit angle over I2C every 5 ms.
- Zero-crossing: detect wrap-around (delta > +2048 → subtract 4096; delta < -2048 → add 4096).
- Hysteresis: a window boundary at position `P` is only crossed when the accumulated distance reaches `P + hysteresis_ticks` on the forward side, preventing oscillation.
- I2C error: hold last valid position, set bit0=0 (AS5600_invalide) in flags_obs.
- Magnet out-of-range (AGC register check): set bit5=1 (aimant_hors_plage).
- Backward delta (negative): set bit6=1 (recul_détecté), do not add to forward accumulator.

#### 4. INA226 Reader

- Read current (shunt) and bus voltage registers over I2C at ~100 Hz.
- Accumulate sum_current_mA and sum_voltage_mV within the current window.
- Track n (expected samples) and k (failed reads).
- I2C error: hold last valid values (hold strategy), increment k.
- At window close:
  - if k < 0.3 × n: courant_moy = sum_current / n, tension_moy = sum_voltage / n, bit1=1
  - if k ≥ 0.3 × n: bit1=0 (INA226_invalide)
- Overcurrent check: if any single sample exceeds `overcurrent_threshold_mA` (config.h, default 500 mA), trigger safety state machine → STOPPED_OVERCURRENT.
- energie_fenetre_uJ = Σ (current_i_mA × voltage_i_mV × dt_us) / 1_000_000 (accumulated in µJ using integer arithmetic).

#### 5. Table_Commandes

- Fixed-size array of 16 slots (configurable in config.h), each slot: `{ uint16_t N; uint16_t duty_pm; bool occupied; }`.
- On Paquet_Commande type-0 reception: for each (N_cmd, duty_pm) entry:
  - reject if N_cmd < fenetre_courante (stale)
  - reject if N_cmd > fenetre_courante + Horizon_Max (too far ahead)
  - reject if table full and no existing slot for N_cmd (log warning)
  - store or replace existing slot for N_cmd
- On window boundary N → N+1: remove slot for N (if still present).
- Lookup at window N entry: scan array for slot with N matching current window.

#### 6. UDP Communication

**TX (Paquet_Observation):**
- Triggered at window close: pack Enregistrement_Fenêtre ring buffer (last 3 records, most recent first), fill envelope fields, compute CRC16/CCITT-FALSE over all preceding bytes, send UDP to host_ip:host_port.
- Heartbeat: if 250 ms elapsed since last observation, send Paquet_Vie_ESP32 (type=1).
- Sequence number `seq` incremented per packet.

**RX (Paquet_Commande):**
- Receive on ESP32_port (default 8889).
- Validate: version, packet size (per type), CRC16.
- Validate: episode_id matches current episode (reject otherwise).
- Dispatch by type: 0 → Table_Commandes update; 1 → episode start; 2 → episode end; 3 → emergency stop; 4 → vie hôte (no-op beyond updating last-received-host-heartbeat timestamp).
- Update `last_valid_cmd_time` for types 0, 1, 2, 3 only (not type 4).
- Learn host IP from first Paquet_Commande if not yet known (fallback: host_ip from config.h is always set at compile time, so the ESP32 always knows where to send observations).

---

### Host Application Components

#### Processus_UDP

Single Python process (asyncio or threading). Two UDP sockets:
- **RX socket** bound to `0.0.0.0:8888` — receives Paquet_Observation from ESP32.
- **TX socket** — sends Paquet_Commande to learned ESP32 IP:8889.

Responsibilities:
1. Parse and validate incoming Paquet_Observation (version, size, CRC16).
2. Reject packets with bad episode_id, unknown version, wrong size, bad CRC → increment rejected counter.
3. Extract up to 3 Enregistrements_Fenêtre; deduplicate each by Clé_Fenêtre = (episode_id, N); publish new records to `observations` Redis Stream.
4. Update `derniere_observation` Redis key with full deserialized observation + host timestamp.
5. Consume `commandes` Redis Stream; pack and send Paquet_Commande type-0 to ESP32.
6. Send Paquet_Commande type-4 (vie hôte) at least once per second even when no command is queued.
7. Log statistics to stdout every 5 s (windows/s, recovered, lost, rejected, commands sent, stale commands).
8. Do not send any Paquet_Commande until at least one Paquet_Observation has been received in the current session (ESP32 IP discovery).

**Sliding redundancy on receive:** When a Paquet_Observation arrives carrying records at positions 1, 2, 3 (indices 0–2), each record is checked against the deduplication set. Records recovered from position 2 or 3 (not the first slot) are marked with `source=redundancy` so the Journal_Episodes can set `valide=2`.

#### Processus_Contrôle

Single Python process. No internal clock — event-driven.

```python
while True:
    entry = redis.xread({"observations": last_id}, block=0)[0]  # blocking
    obs = deserialize(entry)
    commands = controller.on_observation(obs)  # list[(N, duty_pm)]
    for (N, duty_pm) in commands:
        redis.xadd("commandes", {"N": N, "duty_pm": duty_pm})
    redis.set("derniere_commande", serialize_last_command(commands))
    last_id = entry.id
```

Crash recovery: supervised by Docker `restart: unless-stopped`; the Redis Stream retains all unread entries so no observation is permanently lost.

Pre-loading at episode start: The Processus_Contrôle detects a type-1 episode start signal (special entry in `observations` stream or a Redis pub/sub channel) and calls `controller.on_episode_start()` which may publish commands for windows 0 through M (Paramètre_M, default 3) before any observation arrives.

#### Controller Interface

```python
from abc import ABC, abstractmethod

class Controller(ABC):
    @abstractmethod
    def on_observation(self, obs: dict) -> list[tuple[int, int]]:
        """
        Called for each completed window observation.

        obs keys (minimum):
            N               int   – window number
            duree_us        int   – window duration in µs
            courant_moy_mA  int   – mean current in mA
            tension_moy_mV  int   – mean voltage in mV
            energie_fenetre_uJ int – window energy in µJ
            advance_margin  int   – current N_cmd - N_esp

        Returns:
            List of (N_target, duty_pm) pairs to be sent as commands.
            duty_pm is in per-mille [0, 1000].
        """

    def on_episode_start(self, episode_id: int, dx_mm: int,
                         distance_max_mm: int) -> list[tuple[int, int]]:
        """
        Optional hook called at episode start for pre-loading commands.
        Default: returns [].
        """
        return []
```

**Provided implementations:**

| Class | Behavior |
|-------|----------|
| `ManuelController` | Returns constant `(N+M, duty_pm)` for configurable `duty_pm` and `M`. |
| `EchelonRampeController` | Executes a programmed profile: list of `(N_start, duty_pm)` breakpoints read from config. |
| `PolitiqueController` | Loads a callable from an external Python file (`policy.py`) and delegates to it. |

Selection is done via a config entry (`CONTROLLER_CLASS` environment variable or `.env`); Processus_Contrôle imports the chosen class at startup.

#### Journal_Episodes

Separate Python process. Consumes `observations` Redis Stream via consumer group `journal`.

- Creates one CSV file per episode at episode start: `episodes/{date}_{episode_name}.csv`.
- Writes one row per window N (type-0 records only; type-1 vie packets are ignored).
- Tracks pending rows: some windows may be recovered via redundancy before the direct record arrives.
- At episode end: closes CSV, writes `{date}_{episode_name}.json` with metadata.

**Derived column formulas:**

| Column | Formula |
|--------|---------|
| `x_mm` | `N * dx_mm` |
| `vitesse_moy_m_s` | `(dx_mm * 1000) / duree_us` (integer µm/µs = m/s) |
| `puissance_moy_mW` | `(energie_fenetre_uJ * 1000) / duree_us` |
| `energie_cumulee_uJ` | Running sum of `energie_fenetre_uJ` since episode start |
| `temps_cumule_us` | Running sum of `duree_us` since episode start |

Safety-flag override: if any of bit2, bit3, bit4 set, or bit0=0, or bit1=0 → set `valide=0`.

#### Dashboard

Plotly Dash application. Reads `derniere_observation` and `derniere_commande` Redis keys for current state; reads CSV files from the mounted volume for history.

Live charts update on each new Redis key change (interval callback, ~500 ms). Selectable x-axis: time (`temps_cumule_us`) or position (`x_mm`).

Exposed controls via Redis pub/sub channel `dashboard_commands`:
- Start episode → publish type-1 episode start parameters.
- Stop episode → publish type-2 finish command.
- Emergency stop → publish type-3 command.

---

## Data Models

### Binary Packet Layouts

All packets are little-endian packed binary. CRC-16/CCITT-FALSE is computed over all bytes preceding the CRC field.

#### Paquet_Observation type=0 (fenêtre)

```
Offset  Size  Type    Field
──────  ────  ──────  ─────────────────────────────────────────
0       1     uint8   version  (must be 1)
1       1     uint8   type     (= 0)
2       2     uint16  episode_id
4       2     uint16  seq
6       4     uint32  t_esp_us
10      1     uint8   nb_enregistrements  (0–3)
11      2     uint16  derniere_commande_acceptee_N
13      2     uint16  flags_obs
15      …     —       nb_enregistrements × Enregistrement_Fenêtre
…       2     uint16  crc16
```

Total size (nb=0): 17 bytes. Per extra record: +14 bytes.
- nb=1: 31 bytes
- nb=2: 45 bytes
- nb=3: 59 bytes

#### Enregistrement_Fenêtre (embedded, 14 bytes)

```
Offset  Size  Type    Field
──────  ────  ──────  ─────────────────────────────────────────
+0      2     uint16  N
+2      4     uint32  duree_us
+6      2     uint16  courant_moy_mA
+8      2     uint16  tension_moy_mV
+10     4     uint32  energie_fenetre_uJ
```

#### Paquet_Observation type=1 (vie / heartbeat)

```
Offset  Size  Type    Field
──────  ────  ──────  ─────────────────────────────────────────
0       1     uint8   version  (= 1)
1       1     uint8   type     (= 1)
2       2     uint16  episode_id
4       2     uint16  seq
6       4     uint32  t_esp_us
10      1     uint8   nb_enregistrements  (= 0, always)
11      2     uint16  fenetre_courante_N
13      4     uint32  duree_ecoulee_us
17      2     uint16  flags_obs
19      2     uint16  crc16
```

Total size: 21 bytes.

#### Paquet_Commande type=0 (commande moteur)

```
Offset  Size  Type    Field
──────  ────  ──────  ─────────────────────────────────────────
0       1     uint8   version  (= 1)
1       1     uint8   type     (= 0)
2       2     uint16  episode_id
4       2     uint16  seq
6       1     uint8   nb_commandes  (1–3)
7       …     —       nb_commandes × (N: uint16, duty_pm: uint16)
…       2     uint16  crc16
```

Total size: nb=1: 13 bytes, nb=2: 17 bytes, nb=3: 21 bytes.

#### Paquet_Commande type=1 (démarrage d'épisode)

```
Offset  Size  Type    Field
──────  ────  ──────  ─────────────────────────────────────────
0       1     uint8   version  (= 1)
1       1     uint8   type     (= 1)
2       2     uint16  episode_id
4       2     uint16  seq
6       2     uint16  dx_mm
8       4     uint32  distance_max_mm
12      2     uint16  crc16
```

Total size: 14 bytes.

#### Paquet_Commande type=2 (fin d'épisode)

```
Offset  Size  Type    Field
──────  ────  ──────  ─────────────────────────────────────────
0       1     uint8   version  (= 1)
1       1     uint8   type     (= 2)
2       2     uint16  episode_id
4       2     uint16  seq
6       2     uint16  crc16
```

Total size: 8 bytes.

#### Paquet_Commande type=3 (arrêt d'urgence)

```
Offset  Size  Type    Field
──────  ────  ──────  ─────────────────────────────────────────
0       1     uint8   version  (= 1)
1       1     uint8   type     (= 3)
2       2     uint16  episode_id
4       2     uint16  seq
6       2     uint16  crc16
```

Total size: 8 bytes.

#### Paquet_Commande type=4 (vie hôte)

```
Offset  Size  Type    Field
──────  ────  ──────  ─────────────────────────────────────────
0       1     uint8   version  (= 1)
1       1     uint8   type     (= 4)
2       2     uint16  episode_id
4       2     uint16  seq
6       2     uint16  crc16
```

Total size: 8 bytes.

### Shared Schema Implementation

A single source-of-truth schema is defined in the C++ header `firmware/include/protocol.h` using packed structs and a Python module `host/protocol.py` using `struct` format strings. A coherence test (`tests/test_schema_coherence.py`) verifies that field names, types, sizes, and order are identical between the two implementations. This test runs in CI before any integration test.

### Redis Data Model

#### Streams

| Stream name | Producer | Consumer groups | Entry fields |
|-------------|----------|----------------|-------------|
| `observations` | Processus_UDP | `controle`, `journal` | `episode_id`, `N`, `duree_us`, `courant_moy_mA`, `tension_moy_mV`, `energie_fenetre_uJ`, `flags`, `t_esp_us`, `seq`, `source` (`direct`\|`redundancy`), `host_ts_us` |
| `commandes` | Processus_Contrôle | `udp` | `episode_id`, `N`, `duty_pm` |

#### State Keys

| Key | Type | Content | Writer | Reader |
|-----|------|---------|--------|--------|
| `derniere_observation` | STRING (JSON) | Full deserialized Paquet_Observation + `host_ts_us` | Processus_UDP | Dashboard |
| `derniere_commande` | STRING (JSON) | `{episode_id, N, duty_pm, sent_ts_us}` | Processus_UDP | Dashboard |
| `episode_id_counter` | STRING (INT) | Auto-incrementing episode counter | Application_Hôte (startup) | Processus_Contrôle, Dashboard |

#### Consumer Groups

- `controle`: used exclusively by Processus_Contrôle on `observations`.
- `journal`: used exclusively by Journal_Episodes on `observations`.
- `udp`: used exclusively by Processus_UDP on `commandes`.

### CSV File Schema

File path: `episodes/YYYYMMDD_HHMMSS_{episode_name}.csv`

| Column | Type | Source / Formula |
|--------|------|-----------------|
| `episode_id` | INT | From observation |
| `N` | INT | Window number |
| `x_mm` | INT | `N * dx_mm` |
| `duree_us` | INT | From observation |
| `vitesse_moy_m_s` | FLOAT | `(dx_mm * 1000) / duree_us` (µm/µs = m/s) |
| `courant_moy_mA` | INT | From observation |
| `tension_moy_mV` | INT | From observation |
| `puissance_moy_mW` | FLOAT | `(energie_fenetre_uJ * 1000) / duree_us` |
| `energie_fenetre_uJ` | INT | From observation |
| `energie_cumulee_uJ` | INT | Running sum of `energie_fenetre_uJ` |
| `temps_cumule_us` | INT | Running sum of `duree_us` |
| `duty_commande_pm` | INT | From command entry (last command applied for this window, per ESP32 echo in `derniere_commande_acceptee_N`) |
| `flags` | INT | `flags_obs` from observation |
| `valide` | INT | 0 = missing/invalid, 1 = direct reception, 2 = recovered via redundancy |

**Valide override rules (applied in order):**
1. If window was never received → `valide=0`.
2. If window was recovered via sliding redundancy → `valide=2` (provisional).
3. If the direct record is later received → `valide=1` (or 0 if safety flags active).
4. If any safety flag is active (bit2=1 OR bit3=1 OR bit4=1 OR bit0=0 OR bit1=0) → `valide=0`.

### Shared Pseudo-Random Generator (LCG)

The same LCG is implemented identically in C++ and Python:

```
state_next = (1664525 * state + 1013904223) mod 2^32
```

**C++ (firmware/include/prng.h):**
```cpp
uint32_t lcg_state;
inline uint32_t lcg_next() {
    lcg_state = 1664525u * lcg_state + 1013904223u;  // wraps at 2^32
    return lcg_state;
}
```

**Python (host/prng.py):**
```python
MASK32 = 0xFFFFFFFF
def lcg_next(state: int) -> tuple[int, int]:
    state = (1664525 * state + 1013904223) & MASK32
    return state, state
```

The coherence test fixture (`tests/fixtures/lcg_reference.json`) contains the first 1000 values generated from seed 42 by the C++ implementation (via a standalone test binary). The Python test verifies the Python implementation produces the same 1000 values.

---

## Correctness Properties

*A property is a characteristic or behavior that should hold true across all valid executions of a system — essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.*

### Property 1: Packet round-trip identity

*For any* valid `Enregistrement_Fenêtre` (N ∈ [0, 65535], duree_us ∈ [1, 2^32-1], courant_moy_mA ∈ [0, 65535], tension_moy_mV ∈ [0, 65535], energie_fenetre_uJ ∈ [0, 2^32-1]), serializing it into a `Paquet_Observation` binary frame and then deserializing that frame SHALL produce a record whose fields are identical to the original.

**Validates: Requirements 2.10, 2.1, 2.2, 13.1**

### Property 2: CRC rejection

*For any* valid binary packet, flipping any single bit in the payload (i.e., any byte before the CRC field) SHALL cause the receiver to reject the packet (computed CRC ≠ received CRC).

**Validates: Requirements 2.13, 1.9**

### Property 3: Window-stop invariant

*For any* Table_Commandes state and window number N, if no entry for N is present in the Table_Commandes at the moment the ESP32 enters window N, the safety state machine SHALL be in state STOPPED_WINDOW_RULE (bit2=1, motors off) immediately after crossing the window boundary, regardless of the presence of entries for windows N+k (k > 0).

**Validates: Requirements 4.6, 4.9, 13.3**

### Property 4: Energy no-double-counting

*For any* episode with a sequence of windows 0..K, the `energie_cumulee_uJ` value in the last CSV row SHALL equal the sum of all `energie_fenetre_uJ` values across rows 0..K with `valide ≠ 0`, and SHALL equal this sum regardless of the order in which window records are received (direct vs. recovered via redundancy), as long as each Clé_Fenêtre (episode_id, N) is counted at most once.

**Validates: Requirements 7.1, 7.9**

### Property 5: LCG determinism

*For any* seed value in [0, 2^32-1] and any number of steps n ≥ 1, the Python `lcg_next` implementation and the C++ `lcg_next` implementation SHALL produce identical sequences of n values when started from the same seed.

**Validates: Requirements 12.3, 12.10, 13.7**

### Property 6: Horizon and staleness rejection

*For any* current window number N_current and received command entry N_cmd, the Table_Commandes SHALL accept the entry if and only if `N_current ≤ N_cmd ≤ N_current + Horizon_Max`; entries outside this range SHALL be discarded without affecting the current motor state.

**Validates: Requirements 4.2, 4.5**

### Property 7: INA226 validity threshold

*For any* window with n total INA226 sample slots and k failed reads, the window's INA226_valide bit (bit1) SHALL be 1 (valid) if and only if k < 0.3 × n; it SHALL be 0 (invalid) if k ≥ 0.3 × n.

**Validates: Requirements 3.7, 3.11**

### Property 8: Watchdog temporal non-reset by type-4

*For any* sequence of Paquet_Commande type-4 packets received after the last type-0/1/2/3 packet, the Watchdog_Temporel timer SHALL NOT be reset; the elapsed time since the last valid command SHALL continue to increase, and the watchdog SHALL fire after `watchdog_temporel_s` seconds regardless of how many type-4 packets arrive in the interim.

**Validates: Requirements 4.8, 4.18, 13.9**

---

## Error Handling

### ESP32 Firmware

| Condition | Detection | Action |
|-----------|-----------|--------|
| AS5600 I2C error | read returns error code | Set bit0=0, hold last position, continue |
| AS5600 magnet out of range | AGC register | Set bit5=1 in flags |
| INA226 I2C error | read returns error code | Hold last values, increment k |
| INA226 k ≥ 0.3×n | at window close | Set bit1=0 for window |
| Overcurrent > threshold | each INA226 sample | Set bit4=1, STBY=LOW, PWM=0 → STOPPED_OVERCURRENT |
| No command at window N | at window entry | Set bit2=1, STBY=LOW, PWM=0 → STOPPED_WINDOW_RULE |
| No valid command ≥ 3 s | watchdog timer | Set bit3=1, STBY=LOW, PWM=0 → STOPPED_WATCHDOG_TEMPORAL |
| Emergency stop type-3 | UDP packet | Set STBY=LOW, PWM=0 → STOPPED_EMERGENCY |
| Distance limit reached | cumulative mm | Set bit7=1, STBY=LOW, PWM=0 → STOPPED_DISTANCE |
| Bad packet (version/size/CRC) | UDP RX | Increment rejected counter, discard |
| Table_Commandes full | at command insert | Discard new entry if no slot available for N |
| Wi-Fi disconnect | WiFi events | Log warning; UDP send will fail silently; watchdog will eventually fire |

### Host Application

| Condition | Detection | Action |
|-----------|-----------|--------|
| Redis unavailable at startup | connection error | Log error, abort service (Docker restart policy handles retry) |
| Redis write failure during operation | exception | Log error with traceback, retry once, then abort (restart policy) |
| Paquet_Observation bad CRC | validation | Increment rejected counter, discard, log |
| Paquet_Observation wrong episode_id | validation | Increment rejected counter, discard |
| Processus_Contrôle unhandled exception | Python exception | Log full stack trace, exit (Docker restarts within 1 s) |
| Marge_Avance < 1 window | comparison | Log WARNING with current N_esp and N_cmd |
| UDP socket error | send/recv | Log with timestamp, error type, source IP |

---

## Testing Strategy

### Unit Tests (`tests/unit/`)

**Packet codec (`test_codec.py`):**
- Test serialization of each packet type against known byte sequences.
- Test deserialization of those byte sequences back to original values.
- Property-based: round-trip test (see Property 1) using Hypothesis.
- Test CRC rejection (see Property 2) using Hypothesis.
- Test rejection of packets with bad version, wrong size, wrong CRC16.

**Table_Commandes logic (`test_table_commandes.py`):**
- Test insertion, replacement, staleness rejection, horizon rejection.
- Property-based: horizon invariant (see Property 6) using Hypothesis.

**INA226 validity rule (`test_ina226_validity.py`):**
- Test boundary: k=2, n=9 (< 30%) → bit1=1.
- Test boundary: k=3, n=9 (≥ 30%) → bit1=0.
- Property-based: validity threshold (see Property 7) using Hypothesis.

**LCG coherence (`test_lcg.py`):**
- Property-based: determinism across seeds (see Property 5) using Hypothesis.
- Load reference fixture (`lcg_reference.json`) and verify 1000 values against Python implementation.

**Journal energy computation (`test_journal_energy.py`):**
- Property-based: no-double-counting (see Property 4) using Hypothesis.
- Test valide override rules with all flag combinations.

**Schema coherence (`test_schema_coherence.py`):**
- Parse `firmware/include/protocol.h` and `host/protocol.py`.
- Assert identical field names, types, sizes, and order for each packet type.
- Assert identical Enregistrement_Fenêtre layout.

### Integration Tests (`tests/integration/`)

All integration tests use the Python Simulateur in place of real hardware.

**Stop rule (`test_stop_rule.py`):**
- Configure Simulateur with `Table_Commandes = {9: 500, 13: 300}`.
- Run episode; verify Simulateur stops at window 10 entry (bit2=1); verify window 13 command does not prevent stop. (Req 13.3)

**Watchdog temporal (`test_watchdog.py`):**
- Start episode, send commands for windows 0–2, then silence.
- Verify stop after 3 s (bit3=1). (Req 13.4)
- Verify type-4 packets during silence do not reset watchdog. (Req 13.9)

**Type-4 no resume (`test_type4.py`):**
- Put Simulateur into STOPPED_WINDOW_RULE state.
- Send Paquet_Commande type-4 only; verify no resume.
- Send Paquet_Commande type-0 only; verify no resume.
- Send Paquet_Commande type-1; verify resume.

**Sliding redundancy (`test_redundancy.py`):**
- Simulate Paquet_Observation with window N only in position 2 (slot index 1); verify Processus_UDP recovers N and marks `source=redundancy`. (Req 13.5)

**Règle_Arrêt resume rules (`test_resume.py`):**
- After STOPPED_WINDOW_RULE: verify type-0 does NOT resume, type-1 does resume. (Req 13.10)

**INA226 partial validity (`test_ina226_integration.py`):**
- Simulate window with k=3/n=9: verify bit1=0, valide=0 in CSV. (Req 13.11)
- Simulate window with k=2/n=9: verify bit1=1, valide=1 in CSV.

### End-to-End Tests (`tests/e2e/`)

**Full episode (`test_full_episode.py`):**
- Start Simulateur, Processus_UDP, Processus_Contrôle (ManuelController, duty=600), Journal_Episodes.
- Send type-1 start command.
- Wait for N=5 observations.
- Send type-2 stop command.
- Verify: CSV file exists with 5 rows, valide ∈ {1, 2}, energie_cumulee_uJ equals sum of per-row energie_fenetre_uJ, JSON metadata file exists. (Req 13.6)

### Property-Based Testing Configuration

- Framework: **Hypothesis** (Python).
- Minimum 100 examples per property test (configured via `@settings(max_examples=100)`).
- Each property test tagged with a comment: `# Feature: esp32-rc-car-controller, Property N: <text>`.
- Separate test target for codec properties (fast, pure Python) and integration properties (use Simulateur with in-process transport).

### Test Pyramid Summary

```
         ┌──────────────┐
         │  E2E Tests   │  ← 1 full-episode test
         └──────┬───────┘
        ┌───────┴────────┐
        │ Integration    │  ← Stop rules, watchdog, redundancy, INA226 rules
        └───────┬────────┘
       ┌────────┴──────────┐
       │   Unit + PBT      │  ← Codec, Table_Commandes, LCG, energy, schema
       └───────────────────┘
```

---

## Docker Compose Service Topology

```yaml
# docker-compose.yml (abbreviated structure)
services:
  redis:
    image: redis:7-alpine
    restart: unless-stopped

  udp:
    build: ./host
    command: python -m udp_process
    restart: unless-stopped
    depends_on: [redis]
    ports:
      - "8888:8888/udp"       # ESP32 → host observations
    volumes:
      - ./config.env:/app/.env:ro
    environment:
      - REDIS_URL=redis://redis:6379

  controle:
    build: ./host
    command: python -m control_process
    restart: unless-stopped
    depends_on: [redis]
    volumes:
      - ./config.env:/app/.env:ro
      - ./policy.py:/app/policy.py:ro   # optional, for PolitiqueController
    environment:
      - REDIS_URL=redis://redis:6379

  dashboard:
    build: ./host
    command: python -m dashboard
    restart: unless-stopped
    depends_on: [redis]
    ports:
      - "8050:8050/tcp"
    volumes:
      - ./config.env:/app/.env:ro
      - episodes:/data/episodes:ro
    environment:
      - REDIS_URL=redis://redis:6379

  simulateur:
    build: ./host
    command: python -m simulateur
    restart: unless-stopped
    depends_on: [redis]
    profiles: ["simulateur"]
    ports:
      - "8889:8889/udp"       # host → simulateur commands
    volumes:
      - ./config.env:/app/.env:ro
    environment:
      - REDIS_URL=redis://redis:6379

volumes:
  episodes:
```

**Network notes:**
- macOS and Windows: `network_mode: host` is not supported → explicit port mappings are mandatory. The ESP32 must have the host machine's IP (reachable from the LAN).
- Linux: `network_mode: host` is optional but not required.
- The `udp` service binds port 8888/UDP; on macOS/Windows, Docker Desktop's NAT forwards incoming UDP from the LAN. The host machine's LAN IP must be fixed (DHCP reservation on router).

---

## Key Configuration Parameters

### Firmware (`firmware/include/config.h`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `WIFI_SSID` | string | — | Wi-Fi network name |
| `WIFI_PASSWORD` | string | — | Wi-Fi password |
| `HOST_IP` | string | — | Host machine LAN IP (fixed) |
| `HOST_UDP_PORT` | uint16 | 8888 | Host UDP RX port for observations |
| `ESP32_UDP_PORT` | uint16 | 8889 | ESP32 UDP RX port for commands |
| `DX_DEFAULT_MM` | uint16 | 250 | Default window length in mm |
| `HORIZON_MAX` | uint8 | 15 | Max windows ahead in Table_Commandes |
| `WATCHDOG_TEMPOREL_S` | uint8 | 3 | Watchdog timeout in seconds |
| `DUTY_MAX` | uint16 | 1000 | Max duty cycle in per-mille |
| `OVERCURRENT_THRESHOLD_MA` | uint16 | 500 | Overcurrent stop threshold |
| `WHEEL_DIAMETER_MM` | float | — | Wheel diameter for distance calibration |
| `TABLE_COMMANDES_SIZE` | uint8 | 16 | Maximum entries in Table_Commandes |
| `PWMA`, `AIN1`, `AIN2` | uint8 | 25, 26, 27 | Motor A pins |
| `PWMB`, `BIN1`, `BIN2` | uint8 | 33, 14, 13 | Motor B pins |
| `STBY` | uint8 | 32 | TB6612FNG standby pin |
| `SDA`, `SCL` | uint8 | 21, 22 | I2C pins |
| `K_SPEED` | float | — | m/s per full-scale duty (Mode_Test_Capteurs) |
| `PRN_SEED` | uint32 | — | LCG seed (Mode_Test_Capteurs and Simulateur) |

A compile-time `static_assert` verifies that `HOST_IP` is syntactically valid and that all port values are in [1024, 65535].

### Host Application (`.env` / Docker environment)

| Variable | Default | Description |
|----------|---------|-------------|
| `REDIS_URL` | `redis://redis:6379` | Redis connection string |
| `UDP_LISTEN_PORT` | `8888` | Host UDP RX port |
| `ESP32_UDP_PORT` | `8889` | ESP32 UDP TX port |
| `CONTROLLER_CLASS` | `ManuelController` | Controller implementation to use |
| `CONTROLLER_DUTY_PM` | `500` | Duty cycle for ManuelController |
| `PARAMETRE_M` | `3` | Command advance margin |
| `DASHBOARD_PORT` | `8050` | Plotly Dash HTTP port |
| `EPISODES_DIR` | `/data/episodes` | Mounted volume for CSV output |
| `LOG_STATS_INTERVAL_S` | `5` | Statistics logging interval |
| `SIMULATOR_LOSS_RATE` | `0.0` | Packet loss probability [0.0–1.0] for Simulateur |
| `SIMULATOR_K` | — | k parameter (m/s per full duty) for Simulateur |
| `PRN_SEED` | `42` | LCG seed for Simulateur |

---

## Simulateur Design

The Simulateur is a Python service that replaces the physical ESP32 for testing. It listens for Paquet_Commande on port 8889 and sends Paquet_Observation to the Processus_UDP on port 8888.

### Simulateur Internal State Machine

Identical to the ESP32 firmware FSM (IDLE, EPISODE_ACTIVE, STOPPED_*). Same Table_Commandes logic, same Horizon_Max, same Règle_Arrêt_Fenêtres, same Watchdog_Temporel.

### Window Duration Formula

```
if duty_pm == 0:
    no window ends; emit Paquet_Vie_ESP32 every 250 ms
else:
    vitesse_m_s = k * duty_pm / 1000
    duree_us = (dx_mm * 1_000_000) / vitesse_m_s
           = (dx_mm * 1_000_000_000) / (k * duty_pm)  [integer arithmetic]
```

### LCG Usage

At each window close, the Simulateur draws two consecutive LCG values:
- `courant_moy_mA = lcg_next() % 1000`  (0–999 mA range for simulation)
- `tension_moy_mV = 7200 + lcg_next() % 400`  (7200–7599 mV range, LiPo 2S nominal)

These must match the values the firmware would generate in MODE_TEST_CAPTEURS with the same seed and the same sequence of window completions.

### Packet Loss Simulation

The Simulateur drops each outgoing Paquet_Observation with probability `SIMULATOR_LOSS_RATE`. Sliding redundancy (carrying the last 3 records) mitigates the effect. The test suite uses a loss rate of ~0.2 to exercise redundancy recovery.

---

## Mode_Test_Capteurs (Firmware)

Activated by the compile flag `-DMODE_TEST_CAPTEURS` in `platformio.ini`.

**Sensor replacement:**
- AS5600 is not read. Window duration is computed from duty_pm using the kinematic formula: `duree_us = (dx_mm × 1_000_000) / (k × duty_pm / 1000)`. If duty_pm = 0, no window ends and Paquet_Vie_ESP32 is emitted every 250 ms.
- INA226 is not read. Values are generated by the LCG.

**PWM output for multimeter verification:**
- PWMA / PWMB outputs a mean voltage equal to `(duty_pm / 1000) × 3.3 V`.
- With k set low (e.g., 0.1 m/s), each window lasts several seconds at duty=500 → readable with a 1 Hz multimeter.
- Expected readings at duty_pm=500: PWMA ≈ 1.65 V mean, STBY = 3.3 V (HIGH), AIN1 = 3.3 V / AIN2 = 0 V (or reverse depending on direction config).
- On watchdog stop: STBY → 0 V.

**Step-by-step multimeter test procedure:**
1. Flash firmware with `-DMODE_TEST_CAPTEURS` and `K_SPEED=0.1`.
2. Power ESP32 via USB. Observe serial log: "Mode Test Capteurs actif".
3. Connect multimeter (DC voltage, 10 V range) to PWMA and GND.
4. From Dashboard, send episode start (type-1) with duty_pm=500 for window 0.
5. Expected: STBY pin = 3.3 V. PWMA ≈ 1.65 V (mean, needle oscillates on slow meter).
6. Wait 6+ s without sending any further commands.
7. Expected: Watchdog fires. STBY pin drops to 0 V. Serial log: "Watchdog temporel".
8. Send episode start (type-1). Motors resume.

