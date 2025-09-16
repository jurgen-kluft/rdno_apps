#include "humanpresence/c_humanpresence.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_remote.h"
#include "rdno_wifi/c_node.h"
#include "rdno_core/c_nvstore.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_sensor_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"

#include "common/c_common.h"

#include "rdno_sensors/c_hmmd.h"

using namespace ncore;

ncore::linear_alloc_t    gAllocator;  // Linear allocator for memory management
ncore::nvstore::config_t gConfig;     // Configuration structure for non-volatile storage

u64                     gLastSensorReadTimeInMillis = 0;
nsensor::SensorPacket_t gSensorPacket;                      // Sensor packet for sending data
u16                     gSequence                 = 0;      // Sequence number for the packet
const u8                kVersion                  = 1;      // Version number for the packet
s16                     gLastDistanceInCm         = 32768;  // Last distance value read from the sensor
s8                      gLastPresence             = 0;      // Last presence value read from the sensor

void setup()
{
    nserial::begin();  // Initialize serial communication at 115200 baud

    if (nsystem::init_psram())  // Initialize PSRAM if available
    {
        nserial::println("PSRAM is available and initialized.");
        
        nserial::print("PSRAM size: ");
        const s32 free_heap_size = (s32)nsystem::total_psram();
        nserial::print(free_heap_size);
        nserial::println("");

        nserial::print("PSRAM free: ");
        const s32 free_psram_size = (s32)nsystem::free_psram();
        nserial::print(free_psram_size);
        nserial::println("");
    }

    const u32 alloc_size = 1024 * 8;
    byte*     alloc_mem  = nsystem::malloc(alloc_size);  // Allocate memory for the linear allocator
    gAllocator.setup(alloc_mem, alloc_size);             // Set up the linear allocator with the allocated memory

    // Initialize the sensors
    const u8 rx = 15;            // RX pin for HMMD
    const u8 tx = 16;            // TX pin for HMMD
    nsensors::initHMMD(rx, tx);  // Initialize the HMMD sensor

    // Initialize the WiFi node
    nserial::println("Load config from non-volatile memory.");
    if (!nvstore::load(&gConfig))  // Load configuration from non-volatile storage
    {
        nserial::println("Initializing default config.");
        setup_default_config(&gConfig);  // Set up default configuration values
        nserial::println("Saving config to non-volatile memory.");
        nvstore::save(&gConfig);  // Save the default configuration to non-volatile storage
    }

    nserial::println("Node setup started...");
    nwifi::node_setup(&gConfig, ncore::key_to_index);  // Set up the WiFi node with the configuration

    // This is where you would set up your hardware, peripherals, etc.
    // npin::SetPinMode(2, ncore::npin::ModeOutput);  // Set the LED pin as output

    gLastSensorReadTimeInMillis = ntimer::millis();

    nserial::println("Setup done...");
}

// Main loop of the application
void loop()
{
    if (nwifi::node_loop(&gConfig, ncore::key_to_index))
    {
        const u64 currentTimeInMillis = ntimer::millis();
        if (currentTimeInMillis - gLastSensorReadTimeInMillis >= 100)  // 10 times per second
        {
            gLastSensorReadTimeInMillis = currentTimeInMillis;

            // Read the HMMD sensor data
            s8  presence     = gLastPresence;
            u16 distanceInCm = gLastDistanceInCm;
            s32 count        = 0;
            while (nsensors::readHMMD2(&presence, &distanceInCm))
            {
#if 0
                nserial::print("Read Presence: ");
                nserial::println(presence == 1 ? "On" : "Off");
                nserial::print("Read Distance: ");
                char  distanceStrBuffer[16];
                str_t distanceStr = str_mutable(distanceStrBuffer, 16);
                to_str(distanceStr, (s32)distanceInCm, 10);
                nserial::print(distanceStr.m_const);
                nserial::println(" cm");
                nserial::print("Count = ");
                nserial::print(count);
                nserial::println("");
                count++;
#endif
            }

            // Write a custom (binary-format) network message
            gSensorPacket.begin(gSequence++, kVersion);
            if (presence != gLastPresence)
            {
                gLastPresence = presence;
                gSensorPacket.write_sensor_value(nsensor::SensorType::Presence, presence);
            }
            if ((distanceInCm > 0 && distanceInCm < 2000) && distanceInCm != gLastDistanceInCm)
            {
                gLastDistanceInCm = distanceInCm;
                gSensorPacket.write_sensor_value(nsensor::SensorType::Distance, distanceInCm);
            }

            if (gSensorPacket.finalize() > 0)
            {
#ifdef TARGET_DEBUG
                nserial::print("Sending presence=");
                nserial::print(presence == 1 ? "On" : "Off");
                nserial::print(", distance=");
                char  distanceStrBuffer[16];
                str_t distanceStr = str_mutable(distanceStrBuffer, 16);
                to_str(distanceStr, (s32)distanceInCm, 10);
                nserial::print(distanceStr.m_const);
                nserial::println(" cm");
#endif
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