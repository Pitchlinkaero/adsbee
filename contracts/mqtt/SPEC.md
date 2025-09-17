### ADSBee MQTT Contract v1

This document defines the MQTT contract (topics, payloads, QoS, pacing, and operational behavior) for ADSBee 1090/UAT publishing. It is the single source of truth for implementers and integrators. All payload JSON is defined by the protobuf in `contracts/mqtt/mqtt_messages.proto` and follows the proto3 JSON mapping.

Low-latency objective: deliver aircraft status updates to the broker with minimal delay at a fixed, predictable cadence.

#### Scope
- Aircraft state (status) over JSON and compact binary
- Device telemetry over JSON and compact binary
- GPS position over JSON and compact binary
- Per-device topic namespace and dual-band routing (1090 MHz, 978 MHz/UAT)

#### Topic Namespace
- Device-scoped topics require a 16-hex-char `device_id` derived from the receiver ID.

- JSON topics:
  - `{device_id}/adsb/{ICAO}/status` for 1090 MHz
  - `{device_id}/uat/{ICAO}/status` for 978 MHz (UAT)
  - `{device_id}/system/telemetry`
  - `{device_id}/system/gps`

- Binary topics (short form):
  - `{device_id}/a/{ICAO}/s` (1090 MHz status)
  - `{device_id}/u/{ICAO}/s` (UAT status)
  - `{device_id}/sys/t` (telemetry)
  - `{device_id}/sys/g` (gps)

Constraints:
- `ICAO` is uppercase hex, 6 chars.
- Topic length must be < 64 bytes.

#### QoS, Retain, LWT
- Aircraft status: QoS 0, retain=false
- Raw frames (if enabled): QoS 0, retain=false
- Telemetry: QoS 1, retain=false (broker may redeliver)
- GPS: QoS 0, retain=false
- Last Will (LWT): `{device_id}/system/online` retained with payload `0` on disconnect; publisher sets `1` retained on connect.

#### Message Schemas (JSON)
Defined in `mqtt_messages.proto` and serialized using proto3 JSON mapping:
- `AircraftStatus` (key fields: `icao`, `band`, `call`, `cat`, `lat`, `lon`, `alt_ft`, `hdg_deg`, `spd_kts`, `vr_fpm`, `sqk`, `on_ground`)
- `Telemetry` (key fields: `uptime_sec`, `msgs_rx`, `msgs_tx`, `cpu_temp_c`, `mem_free_kb`, `noise_floor_dbm`, `rx_1090`, `rx_978`, `wifi`, `mqtt`, `fw_version`, optional `mps_total`, `mps_feeds`)
- `GPS` (key fields: `lat`, `lon`, `alt_m`, `fix`, `sats`, `hdop`, `ts`)
- `RawPacket` (optional; only when `report_mode` includes RAW): `t_ms`, `icao`, `band`, `df`, `rssi_dbm`, `bytes`

Field semantics:
- `band` ∈ {`BAND_1090`, `BAND_UAT`}
- `cat` is the ADS-B emitter category code like `A3`, `B1`, etc.
- `on_ground` is boolean. `alt_ft` is baro altitude.

#### Binary Messages (compact)
- Status: fixed-size 31 bytes containing type, band, icao, rssi, timestamp, lat/lon (1e-5 deg), alt/25ft, hdg, spd, vr, flags, category, callsign[8].
- Telemetry: 16 bytes
- GPS: 15 bytes

Binary layout is defined in the implementation and MUST be packed manually (no C bitfields). Exact byte offsets are covered by unit tests that round-trip messages and by a Python reference decoder used in CI.

#### Pacing and Backpressure
- Aircraft status cadence: 1 Hz per aircraft by default (configurable 1–2 Hz). Publisher MUST prioritize immediate delivery (no batching) and drop intermediate updates to preserve cadence under load. End-to-end publish latency target: < 100 ms from ingest to broker when network is healthy.
- Raw frames (JSON/binary): disabled by default. If enabled, binary is strongly preferred; JSON raw MUST be rate-limited (global cap, e.g., 50 msgs/sec per device).
- Telemetry: every 60 seconds (configurable, ≥10 seconds)
- GPS: every 60 seconds (configurable, ≥5 seconds)

#### Connection Policy
- URI: `mqtt://host:port` or `mqtts://host:8883`
- Auth: optional username/password; recommended for cloud brokers
- TLS: CA bundle verification required when using `mqtts://`; client cert optional
- Reconnect: exponential backoff with jitter (e.g., base 1s, factor 2, max 60s, ±20% jitter). On successful connect, the publisher should immediately resume 1 Hz cadence.
- Error classification: log auth, DNS, TLS, TCP separately

#### Band Routing
- Publishers MUST set `band` correctly from the ingest source.
- Topic prefixes MUST reflect band (`adsb/` vs `uat/`; `a/` vs `u/`).

#### Settings and Safety
- Global feature gate: `ENABLE_MQTT` must be true to initialize any MQTT clients.
- Per-feed settings: protocol (NONE|BEAST|BEAST_RAW|MQTT), format (JSON|BINARY), report_mode (STATUS|RAW|BOTH).
- Settings version handshake across RP2040/ESP: on mismatch, do not apply; run safe defaults and surface error until reset/migrate.

#### Acceptance Tests
- Schema compliance: sample JSON fixtures in CI must validate against the protobuf JSON mapping.
- Binary compliance: pack/unpack round-trip tests for all message types; Python and C++ decoders must agree.
- Broker smoke test: containerized Mosquitto, device publishes test frames, test client verifies topics, QoS, payloads, and pacing.
- Fail-safe behavior: with bad credentials or TLS failure, device must not impact non-MQTT functions and must back off retries.

#### Versioning
- Contract version: v1. Backwards-compatible changes add optional fields. Breaking changes bump major and require coordinated rollout.


