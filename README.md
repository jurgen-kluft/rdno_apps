# rdno_apps

- Air Quality Monitoring
- Human Presence Detection
- Bed Occupancy Detection

Uses the following `rdno` libraries:

- rdno_core
- rdno_network
- rdno_wifi
- rdno_sensors

## Silent Mode

During a specific time window (e.g., 10 PM to 6 AM), the device can enter a silent mode where it minimizes data transmission to conserve power and reduce noise. In this mode, the device can either stop sending data altogether or send data at a much lower frequency (e.g., every hour instead of every minute).

info = 8 bytes, 7 sensor values = 24 bytes, total = 32 bytes

e.g. one packet per second, size of one packet = 32 bytes:
- per second = 32 bytes
- per minute = 1.92 KB
- per hour = 115.2 KB
- 8 hours = 921.6 KB = 0.92 MB
- per day = 2.76 MB
