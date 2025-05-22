#include "rdno_bedpresence/c_bedpresence.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_dio.h"
#include "rdno_core/c_adc.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_client.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"

#include "rdno_bedpresence/c_network_secrets.h"

namespace ncore
{
    static byte           sLocalMemory[1024];  // Local memory for the linear allocator
    static linear_alloc_t gLinearAlloc;

};  // namespace ncore

using namespace ncore;

static s16                    gClientIndex = -1;  // Client index for the TCP client
static nwifi::nstatus::enum_t gWifiStatus  = nwifi::nstatus::Idle;

// Initialize the system
void setup()
{
    gLinearAlloc.setup(sLocalMemory, sizeof(sLocalMemory));

    // Initialize the WiFi module
    nwifi::BeginEncrypted(WIFI_SSID, WIFI_PASSWORD);  // Connect to the WiFi network with SSID "OBNOSIS8"

    // Create a new client and connect to the server
    gClientIndex = nclient::NewClient(&gLinearAlloc);
    s32 error    = nclient::Connect(gClientIndex, SERVER_IP, SERVER_PORT);  // Connect to the server

    // This is where you would set up your hardware, peripherals, etc.
    // TODO; we need a way to provide a 'logger', so that in the final build we can
    //       disable the serial output, and use a different method to log messages.
    ncore::nserial::Begin(ncore::nserial::nbaud::Rate115200);  // Initialize serial communication at 115200 baud

    ncore::npin::SetPinMode(2, ncore::npin::ModeOutput);  // Set the LED pin as output

    nserial::Println("Bed Presence ...");
}

// Constants
const float V_REF     = 5.0;                // Analog reference voltage (e.g., 5V or 3.3V)
const s32   R_BITS    = 10;                 // ADC resolution (bits)
const float ADC_STEPS = (1 << R_BITS) - 1;  // Number of steps (2^R_BITS - 1)

const u8 kLeftSideGPIO  = 10;  // Pin number for the left side sensor
const u8 kRightSideGPIO = 12;  // Pin number for the right side sensor

// Convert ADC value to voltage
// Formula: voltage = (adc_value / ADC_STEPS) * V_REF
inline float to_voltage(s32 adc_value) { return (static_cast<float>(adc_value) / ADC_STEPS) * V_REF; }

// Main loop of the application
void loop()
{
    s32 left_side   = nadc::analogRead(kLeftSideGPIO);  // Read the left side sensor value
    s32 left_side_v = to_voltage(left_side);            // Convert the sensor value to voltage

    s32 right_side   = nadc::analogRead(kRightSideGPIO);  // Read the right side sensor value
    s32 right_side_v = to_voltage(right_side);            // Convert the sensor value to voltage

    nwifi::nstatus::enum_t wifi_status = nwifi::Status();  // Get the current WiFi status
    if (wifi_status == nwifi::nstatus::Connected)
    {
        if (nclient::Connected(gClientIndex))
        {
            // Write a custom (binary-format) network message
            nclient::Write(gClientIndex, left_side_v);   // Send the left side voltage to the server
            nclient::Write(gClientIndex, right_side_v);  // Send the right side voltage to the server
        }
        else
        {
            // If the client is not connected, try to reconnect
            nclient::Connect(gClientIndex, SERVER_IP, SERVER_PORT);  // Reconnect to the server
            ntimer::Delay(1000);                                     // Wait for 1 second
        }
    }
    else
    {
        // If the client is not connected, try to reconnect
        nwifi::BeginEncrypted(WIFI_SSID, WIFI_PASSWORD);
        ntimer::Delay(1000);  // Wait for 1 second
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