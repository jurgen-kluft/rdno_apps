#include "magnet/c_magnet.h"

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

#include "rdno_sensors/c_ys312.h"

#include "common/c_common.h"

using namespace ncore;

ncore::nvstore::config_t gConfig;                    // Configuration structure for non-volatile storage
u64                      gLastReadTimeInMillis = 0;  // Last time the sensor was read

//#define USE_MAGNET_SENSOR
#define USE_PIR_SENSOR

#ifdef USE_MAGNET_SENSOR
const s8  gInputPin            = 2;  // GPIO pin connected to the magnet sensor
const s32 gMagnetThreshold = 100;  // Threshold value for magnet detection, < 100 means magnet is closed, >= 100 means magnet is open
s32 lastMagnetValue = -1;          // Last read value of the magnet sensor
#endif

#ifdef USE_PIR_SENSOR
const s8  gInputPin            = 2;  // GPIO pin connected to the magnet sensor
const s32 gPIRThreshold = 500;  // Threshold value for PIR detection, < 500 means no motion, >= 500 means motion detected
s32 lastPIRValue = -1;          // Last read value of the PIR sensor
#endif

void setup()
{
    nserial::begin();  // Initialize serial communication at 115200 baud

    nserial::println("Load config ...");
    // Initialize the WiFi node
    //    if (!nvstore::load(&gConfig))  // Load configuration from non-volatile storage
    {
        setup_default_config(&gConfig);  // Set up default configuration values
        nvstore::save(&gConfig);         // Save the default configuration to non-volatile storage
    }

    // Initialize the WiFi node
    nserial::println("Node setup ...");
    nwifi::node_setup(&gConfig, ncore::key_to_index);  // Set up the WiFi node with the configuration

    // This is where you would set up your hardware, peripherals, etc.
    ngpio::set_pinmode(gInputPin, ncore::ngpio::ModeInput);  // Set D0 pin as input

    nserial::println("Setup done ...");
}

// Main loop of the application
void loop()
{
    if (nwifi::node_loop(&gConfig, ncore::key_to_index))
    {
        const u64 currentTimeInMillis = ntimer::millis();
        //if (currentTimeInMillis - gLastReadTimeInMillis >= 100)  // 10 times per second
        {
            gLastReadTimeInMillis = currentTimeInMillis;

#ifdef USE_MAGNET_SENSOR
            const s32 magnetValue = ngpio::read_analog(gInputPin) < gMagnetThreshold ? 1 : 0;  // Read the magnet sensor
            if (magnetValue != lastMagnetValue)
            {
                nserial::print("Magnet: ");
                nserial::print(magnetValue == 1 ? "On" : "Off");  // Print the magnet value to the serial console
                nserial::println("");
                lastMagnetValue = magnetValue;
            }
#endif

#ifdef USE_PIR_SENSOR
            u16 pirValue = 0;
            if (nsensors::nys312::read(gInputPin, &pirValue))
            {
                nserial::print("PIR: ");
                nserial::print((u32)pirValue, false); 
                nserial::println("");
                lastPIRValue = pirValue;
            }
#endif
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