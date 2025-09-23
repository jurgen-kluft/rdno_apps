#include "rd03d/c_rd03d.h"
#include "rdno_sensors/c_rd03d.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_gpio.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_remote.h"
#include "rdno_wifi/c_node.h"
#include "rdno_core/c_nvstore.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
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
    nsensors::nrd03d::begin(20, 21);  // Initialize RD03D sensor with rx and tx pin

    nserial::println("Setup done...");
}

// Main loop of the application
void loop()
{
    if (nwifi::node_loop(&gConfig, ncore::key_to_index))
    {
        const u64 currentTimeInMillis = ntimer::millis();
        if (currentTimeInMillis - gLastReadTimeInMillis >= 100)  // 10 times per second
        {
            if (nsensors::nrd03d::update())
            {
                gLastReadTimeInMillis = currentTimeInMillis;

                nsensors::nrd03d::target_t tgt[3];
                for (s8 i = 0; i < 3; ++i)
                {
                    nsensors::nrd03d::getTarget(i, tgt[i]);
                }

                nserial::println("-------------------------");
                nserial::print("X: ");
                nserial::print((s32)tgt[0].x);
                nserial::print(", ");
                nserial::print((s32)tgt[1].x);
                nserial::print(", ");
                nserial::print((s32)tgt[2].x);
                nserial::println("");

                nserial::print("Y: ");
                nserial::print((s32)tgt[0].y);
                nserial::print(", ");
                nserial::print((s32)tgt[1].y);
                nserial::print(", ");
                nserial::print((s32)tgt[2].y);
                nserial::println("");

                nserial::print("S: ");
                nserial::print((s32)tgt[0].distance);
                nserial::print(", ");
                nserial::print((s32)tgt[1].distance);
                nserial::print(", ");
                nserial::print((s32)tgt[2].distance);
                nserial::println("");

                nserial::print("V: ");
                nserial::print((s32)tgt[0].speed);
                nserial::print(", ");
                nserial::print((s32)tgt[1].speed);
                nserial::print(", ");
                nserial::print((s32)tgt[2].speed);
                nserial::println("");
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
