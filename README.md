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

info = 4 bytes, 7 sensor values = 17 bytes, total = 21 bytes, rounded up to 24 bytes (multiple of 4).

e.g. one packet per second, size of one packet = 24 bytes:
- per second = 24 bytes
- per minute = 1.44 KB
- per hour = 86.4 KB
- 8 hours = 691.2 KB
- per day = 2.06 MB

