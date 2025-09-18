#!/usr/bin/env python3
"""
ADSBee MQTT OTA Publisher Example
Publishes firmware updates to ADSBee devices via MQTT

Usage:
    python mqtt_ota_publisher.py --broker broker.example.com --device device123 firmware.ota
"""

import argparse
import hashlib
import json
import os
import struct
import sys
import time
import uuid
from pathlib import Path
from typing import Optional, List

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("Please install paho-mqtt: pip install paho-mqtt")
    sys.exit(1)


class ADSBeeOTAPublisher:
    """Handles OTA firmware publishing via MQTT"""

    def __init__(self, broker: str, port: int = 1883,
                 username: Optional[str] = None,
                 password: Optional[str] = None,
                 use_tls: bool = False):
        """Initialize OTA publisher

        Args:
            broker: MQTT broker hostname
            port: MQTT broker port
            username: Optional MQTT username
            password: Optional MQTT password
            use_tls: Enable TLS/SSL
        """
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.use_tls = use_tls

        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_publish = self._on_publish

        self.device_id = None
        self.session_id = str(uuid.uuid4())
        self.chunk_size = 4096
        self.firmware_data = None
        self.firmware_size = 0
        self.total_chunks = 0

        # Track ACKs
        self.acked_chunks = set()
        self.failed_chunks = set()
        self.last_progress = {}

        # State
        self.connected = False
        self.ota_state = "IDLE"

    def connect(self) -> bool:
        """Connect to MQTT broker"""
        if self.username and self.password:
            self.client.username_pw_set(self.username, self.password)

        if self.use_tls:
            import ssl
            self.client.tls_set(cert_reqs=ssl.CERT_REQUIRED)

        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()

            # Wait for connection
            timeout = 10
            while not self.connected and timeout > 0:
                time.sleep(0.5)
                timeout -= 0.5

            if not self.connected:
                print(f"Failed to connect to {self.broker}:{self.port}")
                return False

            return True

        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def disconnect(self):
        """Disconnect from broker"""
        self.client.loop_stop()
        self.client.disconnect()

    def _on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            print(f"Connected to {self.broker}:{self.port}")
            self.connected = True
        else:
            print(f"Connection failed with code {rc}")

    def _on_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        topic = msg.topic

        # Handle status updates
        if topic.endswith("/ota/status/state"):
            try:
                status = json.loads(msg.payload.decode())
                self.ota_state = status.get("state", "UNKNOWN")
                print(f"Device state: {self.ota_state}")

                if status.get("error"):
                    print(f"Device error: {status['error']}")

            except json.JSONDecodeError:
                pass

        # Handle progress updates
        elif topic.endswith("/ota/status/progress"):
            try:
                progress = json.loads(msg.payload.decode())
                self.last_progress = progress

                percent = progress.get("percent", 0)
                chunks_received = progress.get("chunks_received", 0)
                total_chunks = progress.get("total_chunks", 0)

                print(f"Progress: {percent:.1f}% ({chunks_received}/{total_chunks} chunks)")

            except json.JSONDecodeError:
                pass

        # Handle chunk ACKs
        elif "/ota/status/ack/" in topic:
            chunk_index = int(topic.split("/")[-1])
            success = msg.payload.decode() == "1"

            if success:
                self.acked_chunks.add(chunk_index)
            else:
                self.failed_chunks.add(chunk_index)
                print(f"Chunk {chunk_index} failed, will retry")

    def _on_publish(self, client, userdata, mid):
        """MQTT publish callback"""
        pass

    def subscribe_to_device(self, device_id: str):
        """Subscribe to device OTA topics"""
        self.device_id = device_id

        # Subscribe to status topics
        topics = [
            f"{device_id}/ota/status/state",
            f"{device_id}/ota/status/progress",
            f"{device_id}/ota/status/ack/+"
        ]

        for topic in topics:
            self.client.subscribe(topic, 1)
            print(f"Subscribed to {topic}")

    def load_firmware(self, firmware_path: str) -> bool:
        """Load firmware file

        Args:
            firmware_path: Path to .ota firmware file

        Returns:
            True if loaded successfully
        """
        try:
            firmware_path = Path(firmware_path)
            if not firmware_path.exists():
                print(f"Firmware file not found: {firmware_path}")
                return False

            self.firmware_data = firmware_path.read_bytes()
            self.firmware_size = len(self.firmware_data)
            self.total_chunks = (self.firmware_size + self.chunk_size - 1) // self.chunk_size

            print(f"Loaded firmware: {firmware_path.name}")
            print(f"  Size: {self.firmware_size:,} bytes")
            print(f"  Chunks: {self.total_chunks} x {self.chunk_size} bytes")

            return True

        except Exception as e:
            print(f"Failed to load firmware: {e}")
            return False

    def publish_manifest(self, version: str) -> bool:
        """Publish OTA manifest

        Args:
            version: Firmware version string

        Returns:
            True if published successfully
        """
        if not self.firmware_data:
            print("No firmware loaded")
            return False

        # Calculate SHA256
        sha256 = hashlib.sha256(self.firmware_data).hexdigest()

        # Create manifest
        manifest = {
            "version": version,
            "size": self.firmware_size,
            "chunks": self.total_chunks,
            "chunk_size": self.chunk_size,
            "sha256": sha256,
            "session_id": self.session_id,
            "build_date": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        }

        topic = f"{self.device_id}/ota/control/manifest"
        payload = json.dumps(manifest)

        print(f"Publishing manifest for version {version}")
        self.client.publish(topic, payload, qos=1, retain=True)

        # Wait for device to receive manifest
        time.sleep(2)

        return True

    def send_command(self, command: str) -> bool:
        """Send control command to device

        Args:
            command: START, PAUSE, RESUME, ABORT, VERIFY, or BOOT

        Returns:
            True if sent successfully
        """
        cmd = {
            "command": command,
            "session_id": self.session_id,
            "timestamp": time.time()
        }

        topic = f"{self.device_id}/ota/control/command"
        payload = json.dumps(cmd)

        print(f"Sending command: {command}")
        self.client.publish(topic, payload, qos=1)

        return True

    def publish_chunk(self, chunk_index: int) -> bool:
        """Publish a firmware chunk

        Args:
            chunk_index: Index of chunk to publish

        Returns:
            True if published successfully
        """
        if chunk_index >= self.total_chunks:
            return False

        # Calculate chunk data
        offset = chunk_index * self.chunk_size
        chunk_data = self.firmware_data[offset:offset + self.chunk_size]
        chunk_len = len(chunk_data)

        # Calculate CRC32
        crc32 = self._crc32(chunk_data)

        # Create chunk header
        header = struct.pack(">IIHHI",
            int(self.session_id[:8], 16) & 0xFFFFFFFF,  # Session ID (first 8 chars as hex)
            chunk_index,
            chunk_len,
            crc32,
            0  # Flags
        )

        # Combine header and data
        payload = header + chunk_data

        # Publish
        topic = f"{self.device_id}/ota/data/chunk/{chunk_index}"
        self.client.publish(topic, payload, qos=1)

        return True

    def publish_all_chunks(self, retry_failed: bool = True) -> bool:
        """Publish all firmware chunks

        Args:
            retry_failed: Whether to retry failed chunks

        Returns:
            True if all chunks sent successfully
        """
        print(f"Publishing {self.total_chunks} chunks...")

        # Reset tracking
        self.acked_chunks.clear()
        self.failed_chunks.clear()

        # Send all chunks
        for i in range(self.total_chunks):
            if self.publish_chunk(i):
                # Rate limiting
                if i % 10 == 0:
                    print(f"Sent chunk {i}/{self.total_chunks}")
                    time.sleep(0.1)  # Small delay every 10 chunks
            else:
                print(f"Failed to send chunk {i}")
                self.failed_chunks.add(i)

        # Wait for ACKs
        print("Waiting for acknowledgments...")
        timeout = 60  # 60 seconds timeout
        while len(self.acked_chunks) < self.total_chunks and timeout > 0:
            time.sleep(1)
            timeout -= 1

            # Check for missing chunks
            missing = set(range(self.total_chunks)) - self.acked_chunks
            if missing and timeout % 10 == 0:
                print(f"Missing ACKs for {len(missing)} chunks")

        # Retry failed chunks
        if retry_failed and self.failed_chunks:
            print(f"Retrying {len(self.failed_chunks)} failed chunks...")
            for chunk_index in self.failed_chunks:
                self.publish_chunk(chunk_index)
                time.sleep(0.1)

        # Final check
        if len(self.acked_chunks) == self.total_chunks:
            print("All chunks acknowledged!")
            return True
        else:
            missing = self.total_chunks - len(self.acked_chunks)
            print(f"Warning: {missing} chunks not acknowledged")
            return False

    def perform_ota_update(self, device_id: str, firmware_path: str,
                           version: str) -> bool:
        """Perform complete OTA update

        Args:
            device_id: Target device ID
            firmware_path: Path to firmware file
            version: Firmware version string

        Returns:
            True if update successful
        """
        # Subscribe to device
        self.subscribe_to_device(device_id)
        time.sleep(2)

        # Load firmware
        if not self.load_firmware(firmware_path):
            return False

        # Publish manifest
        if not self.publish_manifest(version):
            return False

        # Wait for device to process manifest
        print("Waiting for device to process manifest...")
        time.sleep(3)

        # Send START command
        self.send_command("START")

        # Wait for device to enter DOWNLOADING state
        print("Waiting for device to start download...")
        timeout = 30
        while self.ota_state != "DOWNLOADING" and timeout > 0:
            time.sleep(1)
            timeout -= 1

        if self.ota_state != "DOWNLOADING":
            print(f"Device not ready for download (state: {self.ota_state})")
            return False

        # Publish all chunks
        if not self.publish_all_chunks():
            print("Failed to publish all chunks")
            self.send_command("ABORT")
            return False

        # Wait for verification
        print("Waiting for device to verify firmware...")
        timeout = 60
        while self.ota_state not in ["READY_TO_BOOT", "ERROR"] and timeout > 0:
            time.sleep(1)
            timeout -= 1

        if self.ota_state == "READY_TO_BOOT":
            print("Firmware verified successfully!")

            # Optional: Send BOOT command
            response = input("Send BOOT command to reboot device? (y/n): ")
            if response.lower() == 'y':
                self.send_command("BOOT")
                print("Device rebooting with new firmware...")

            return True

        else:
            print(f"Update failed (state: {self.ota_state})")
            return False

    def _crc32(self, data: bytes) -> int:
        """Calculate CRC32 checksum

        Args:
            data: Data to checksum

        Returns:
            CRC32 value
        """
        crc = 0xFFFFFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1))
        return ~crc & 0xFFFFFFFF


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="ADSBee MQTT OTA Firmware Publisher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic OTA update
  %(prog)s --broker test.mosquitto.org --device abc123 firmware.ota

  # With authentication
  %(prog)s --broker broker.hivemq.com --username user --password pass --device abc123 firmware.ota

  # With TLS
  %(prog)s --broker broker.example.com --port 8883 --tls --device abc123 firmware.ota
        """
    )

    parser.add_argument("firmware", help="Path to firmware .ota file")
    parser.add_argument("--broker", required=True, help="MQTT broker hostname")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port (default: 1883)")
    parser.add_argument("--device", required=True, help="Target device ID")
    parser.add_argument("--version", default="0.8.3", help="Firmware version (default: 0.8.3)")
    parser.add_argument("--username", help="MQTT username")
    parser.add_argument("--password", help="MQTT password")
    parser.add_argument("--tls", action="store_true", help="Use TLS/SSL")
    parser.add_argument("--chunk-size", type=int, default=4096,
                       help="Chunk size in bytes (default: 4096)")

    args = parser.parse_args()

    # Validate firmware file
    if not os.path.exists(args.firmware):
        print(f"Error: Firmware file not found: {args.firmware}")
        return 1

    # Create publisher
    publisher = ADSBeeOTAPublisher(
        broker=args.broker,
        port=args.port,
        username=args.username,
        password=args.password,
        use_tls=args.tls
    )

    publisher.chunk_size = args.chunk_size

    # Connect to broker
    print(f"Connecting to {args.broker}:{args.port}...")
    if not publisher.connect():
        return 1

    try:
        # Perform OTA update
        success = publisher.perform_ota_update(
            device_id=args.device,
            firmware_path=args.firmware,
            version=args.version
        )

        if success:
            print("\n✓ OTA update completed successfully!")
            return 0
        else:
            print("\n✗ OTA update failed!")
            return 1

    except KeyboardInterrupt:
        print("\n\nUpdate cancelled by user")
        publisher.send_command("ABORT")
        return 1

    finally:
        publisher.disconnect()


if __name__ == "__main__":
    sys.exit(main())