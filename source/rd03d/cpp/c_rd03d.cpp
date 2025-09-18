#include "rd03d/c_rd03d.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_gpio.h"
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

using namespace ncore;

ncore::nvstore::config_t gConfig;                    // Configuration structure for non-volatile storage
u64                      gLastReadTimeInMillis = 0;  // Last time the sensor was read

void setup()
{
    nserial::begin();  // Initialize serial communication at 115200 baud

    // Initialize the WiFi node
    //    if (!nvstore::load(&gConfig))  // Load configuration from non-volatile storage
    {
        setup_default_config(&gConfig);  // Set up default configuration values
        nvstore::save(&gConfig);         // Save the default configuration to non-volatile storage
    }

    // Initialize the WiFi node
    nserial::println("Node setup started...");
    nwifi::node_setup(&gConfig, ncore::key_to_index);  // Set up the WiFi node with the configuration

    // This is where you would set up your hardware, peripherals, etc.

    nserial::println("Setup done...");
}

s32 lastMagnetValue = 0;

// Main loop of the application
void loop()
{
    if (nwifi::node_loop(&gConfig, ncore::key_to_index))
    {
        const u64 currentTimeInMillis = ntimer::millis();
        if (currentTimeInMillis - gLastReadTimeInMillis >= 100)  // 10 times per second
        {
            gLastReadTimeInMillis = currentTimeInMillis;

            
        }
        ntimer::delay(80);
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