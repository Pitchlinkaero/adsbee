#ifndef ADSBEE_MQTT_PACER_HH_
#define ADSBEE_MQTT_PACER_HH_

#include <stdint.h>

// Simple fixed-size pacer for per-aircraft 1 Hz publish cadence.
class MQTTPacer {
public:
    MQTTPacer() : size_(0) {}

    // Returns true if we should publish for this ICAO at time now_ms given interval_ms
    bool ShouldPublish(uint32_t icao24, uint32_t now_ms, uint32_t interval_ms);

    // Clear all state
    void Reset() { size_ = 0; }

private:
    static const uint16_t kMaxEntries = 256;  // Enough for typical active aircraft set
    struct Entry { uint32_t icao; uint32_t last_ms; };
    Entry entries_[kMaxEntries];
    uint16_t size_;
};

#endif  // ADSBEE_MQTT_PACER_HH_


