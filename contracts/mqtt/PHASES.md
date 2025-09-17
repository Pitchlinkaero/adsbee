### MQTT v1 Refactor – Phased Delivery

Branch: `mqtt-v1-refactor`

#### Phase 0 – Guardrails (no functional changes)
- Add global `ENABLE_MQTT` feature gate.
- Enforce settings version handshake (fail-safe on mismatch).
- Add structured logs and counters for publishes/drops/reconnects (surfaced via telemetry later).

#### Phase 1 – Contract-core (JSON status @ 1 Hz)
- Implement `firmware/common/comms/mqtt/mqtt_protocol.{hh,cpp}`:
  - JSON packing per `mqtt_messages.proto` (proto3 JSON mapping semantics).
  - Manual binary packing helpers (no bitfields) – not yet wired.
  - Topic builder per `SPEC.md` with device_id + band-aware prefixes.
- Add pacing helpers for 1 Hz status cadence (drop intermediate updates; no buffers).

#### Phase 2 – Binary parity
- Implement binary status/telemetry/GPS packing with explicit byte layout.
- Add Python reference decoder and C++ round-trip tests (CI).

#### Phase 3 – Telemetry and GPS
- JSON telemetry and GPS at configured cadence, QoS per `SPEC.md`.
- LWT retained online flag.

#### Phase 4 – Connectivity hardening
- `mqtt://` and `mqtts://` with CA verification and optional username/password.
- Exponential backoff with jitter and error classification.

#### Phase 5 – Integration tests
- Mosquitto containerized smoke test verifying topics, QoS, retain, cadence, and band routing.
- Failure modes: bad creds/TLS/DNS backoff, no impact to Beast/GDL90.

Rollout: Keep MQTT disabled by default until CI green and single-device soak passes. Then enable per-device via AT.


