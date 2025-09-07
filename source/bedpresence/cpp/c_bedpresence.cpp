
#include "bedpresence/c_bedpresence.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_dio.h"
#include "rdno_core/c_adc.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_sensor_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"

#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_remote.h"
#include "rdno_wifi/c_node.h"

#include "common/c_common.h"

using namespace ncore;

static ncore::linear_alloc_t gAllocator;  // Linear allocator for memory management
ncore::nvstore::config_t     gConfig;     // Configuration structure for non-volatile storage

#define SENSOR_LOCATION (nsensor::DeviceLocation::Bedroom | nsensor::DeviceLocation::Location1)
static u64                   gLastSensorReadTimeInMillis = 0;

// TODO; we need a way to provide a 'logger', so that in the final build we can
//       disable the serial output, and use a different method to log messages.

// Initialize the system
void setup()
{
    nserial::begin();  // Initialize serial communication at 115200 baud

    const u32 alloc_size = 1024 * 8;
    byte*     alloc_mem  = nsystem::malloc(alloc_size);  // Allocate memory for the linear allocator
    gAllocator.setup(alloc_mem, alloc_size);             // Set up the linear allocator with the allocated memory

    // Initialize the WiFi node
    if (!nvstore::load(&gConfig))  // Load configuration from non-volatile storage
    {
        setup_default_config(&gConfig);  // Set up default configuration values
        nvstore::save(&gConfig);         // Save the default configuration to non-volatile storage
    }

    // Initialize the WiFi node
    nwifi::node_setup(&gConfig, ncore::key_to_index);

    // This is where you would set up your hardware, peripherals, etc.
    npin::set_pinmode(2, ncore::npin::ModeOutput);  // Set the LED pin as output

    gLastSensorReadTimeInMillis = ntimer::millis();

    nserial::println("Setup done...");
}

// Constants
const s32 R_BITS    = 10;                          // ADC resolution (bits)
const f32 ADC_STEPS = 1.0f / ((1 << R_BITS) - 1);  // Number of steps (2^R_BITS - 1)

const u8 kLeftSideGPIO  = 10;  // Pin number for the left side sensor
const u8 kRightSideGPIO = 12;  // Pin number for the right side sensor

// Convert ADC value to a value between 0 and 65535
// Formula: presence = (adc_value / ADC_STEPS)
inline s32 ToPresence(s32 adc_value) { return (static_cast<float>(adc_value) * ADC_STEPS) * 65535.0f; }

static nsensor::SensorPacket_t gSensorPacket;           // Sensor packet for sending data
static u16                     gSequence          = 0;  // Sequence number for the packet
static const u8                kVersion           = 1;  // Version number for the packet
static s32                     gLeftSidePresence  = 0;  // Presence value for the left side sensor
static s32                     gRightSidePresence = 0;  // Presence value for the right side

// Main loop of the application
void loop()
{
    if (nwifi::node_loop(&gConfig, ncore::key_to_index))
    {
        const u64 currentTimeInMillis = ntimer::millis();
        if (currentTimeInMillis - gLastSensorReadTimeInMillis >= 100)
        {
            gLastSensorReadTimeInMillis = currentTimeInMillis;

            const s32 left_side          = nadc::analog_read(kLeftSideGPIO);  // Read the left side sensor value
            const s32 left_side_presence = ToPresence(left_side);             // Convert the sensor value to voltage

            const s32 right_side          = nadc::analog_read(kRightSideGPIO);  // Read the right side sensor value
            const s32 right_side_presence = ToPresence(right_side);             // Convert the sensor value to voltage

            // Write a custom (binary-format) network message
            gSensorPacket.begin(gSequence++, kVersion, SENSOR_LOCATION);
            if (left_side_presence != gLeftSidePresence)
            {
                gLeftSidePresence = left_side_presence;
                gSensorPacket.write_sensor_value(nsensor::SensorType::Presence, left_side_presence);  // Write the left side sensor value
            }
            if (right_side_presence != gRightSidePresence)
            {
                gRightSidePresence = right_side_presence;
                gSensorPacket.write_sensor_value(nsensor::SensorType::Presence, right_side_presence);  // Write the left side sensor value
            }
            if (gSensorPacket.finalize() > 0)
            {
                nremote::write(gSensorPacket.Data, gSensorPacket.Size);  // Send the sensor packet to the server
            }
        }
    }
}

#ifndef TARGET_ESP32

    // We need a 'void main()' function to run the code on non-ESP32 platforms
    #include "rdno_core/c_setup_loop.h"

int main()
{
    setup();  // Call the setup function to initialize the system

    while (true)
    {
        loop();  // Call the loop function continuously
    }

    return 0;  // Return success
}

#endif