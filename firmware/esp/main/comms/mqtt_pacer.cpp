#include "mqtt_pacer.hh"

bool MQTTPacer::ShouldPublish(uint32_t icao24, uint32_t now_ms, uint32_t interval_ms) {
    // Linear search is fine for kMaxEntries size
    for (uint16_t i = 0; i < size_; i++) {
        if (entries_[i].icao == icao24) {
            if (now_ms - entries_[i].last_ms >= interval_ms) {
                entries_[i].last_ms = now_ms;
                return true;
            }
            return false;
        }
    }
    // Not found, add if capacity
    if (size_ < kMaxEntries) {
        entries_[size_].icao = icao24;
        entries_[size_].last_ms = now_ms;
        size_++;
        return true;
    }
    // Out of capacity: publish conservatively without pacing control for this ICAO
    return true;
}


