### MQTT Requirements and Acceptance Criteria (v1)

#### Functional
1. Topics
   - MUST publish JSON to `{device_id}/adsb|uat/{ICAO}/status`, `{device_id}/system/telemetry`, `{device_id}/system/gps`.
   - MAY publish raw to `{device_id}/adsb|uat/{ICAO}/raw` only if `report_mode` enables RAW.
   - MUST publish binary to short topics when `format=BINARY`.

2. Payloads
   - MUST conform to protobuf `mqtt_messages.proto` and proto3 JSON mapping.
   - `band` MUST match the source (1090 vs UAT) and topic prefix.
   - `ICAO` MUST be uppercase hex, 6 chars.

3. QoS/Retain/LWT
   - Status and raw: QoS 0, retain=false.
   - Telemetry: QoS 1, retain=false.
   - LWT: `{device_id}/system/online` retained: `1` on connect, `0` on broker-triggered LWT.

4. Pacing
   - Status: 1 Hz per aircraft by default (configurable 1–2 Hz). MUST prioritize low-latency delivery; drop intermediate updates rather than buffer.
   - Raw: disabled by default; if enabled, global cap (configurable; default ≤ 50 msgs/sec).
   - Telemetry: default 60s; configurable ≥10s.
   - GPS: default 60s; configurable ≥5s.

5. Settings and gating
   - Global `ENABLE_MQTT` must be true before any MQTT code initializes.
   - Per-feed protocol (NONE|BEAST|BEAST_RAW|MQTT), format (JSON|BINARY), report_mode (STATUS|RAW|BOTH).

6. Connectivity
   - Support `mqtt://` and `mqtts://` with CA verification.
   - Optional username/password.
   - Reconnect with exponential backoff (base 1s, factor 2, max 60s, ±20% jitter).

#### Non-Functional
1. Stability: MQTT failures MUST NOT degrade Beast/GDL90 or core decoding. Publish path MUST remain non-blocking.
2. Safety: On settings version mismatch across MCUs, MUST fail-safe with defaults and log; MUST NOT apply incompatible layouts.
3. Determinism: Binary packing MUST be manual (no C bitfields) and platform-independent.
4. Observability: MUST expose counters for publishes, drops, reconnects, last error; visible via telemetry and logs.
5. Resource bounds: No unbounded queues for MQTT; enforce pacing and drops under load.

#### Acceptance Tests
1. Contract fixtures: JSON samples validate against proto mapping; binary messages round-trip in C++ and Python; status cadence verified at 1 Hz in integration test.
2. Broker smoke test: With Mosquitto, verify topics, QoS, retain, pacing, and band routing.
3. Failure modes: Bad creds/TLS/DNS simulate proper backoff and no impact to Beast/GDL90.
4. Settings compatibility: Version mismatch path exercised; MQTT remains disabled until reset/migrate.


