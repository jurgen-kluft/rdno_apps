#include "rdno_bedpresence/c_bedpresence.h"
#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_dio.h"
#include "rdno_core/c_adc.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_client.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_sensor_packet.h"

#include "rdno_bedpresence/c_network.secret.h"

using namespace ncore;

static ncore::linear_alloc_t gAllocator;                           // Linear allocator for memory management
static s16                   gClientIndex  = -1;                   // Client index for the TCP client
static nstatus::status_t     gClientStatus = nstatus::Idle;        //
static nstatus::status_t     gWifiStatus   = nstatus::Idle;        //
static const char*           gHostName     = "BedPresenceDevice";  // Hostname for the device

// TODO; we need a way to provide a 'logger', so that in the final build we can
//       disable the serial output, and use a different method to log messages.

// Initialize the system
void setup()
{
    nserial::Begin(ncore::nserial::nbaud::Rate115200);  // Initialize serial communication at 115200 baud

    const u32 alloc_size = 1024 * 8;
    byte*     alloc_mem  = gMalloc(alloc_size);  // Allocate memory for the linear allocator
    gAllocator.setup(alloc_mem, alloc_size);     // Set up the linear allocator with the allocated memory

    // Initialize the WiFi module
    nwifi::ConfigIpAddrNone();
    nwifi::SetHostname(gHostName);
    nwifi::BeginEncrypted(WIFI_SSID, WIFI_PASSWORD);  // Connect to the WiFi network with SSID "OBNOSIS8"

    gWifiStatus = nwifi::Status();  // Get the current WiFi status
    if (gWifiStatus == nstatus::Connected)
    {
        nserial::Println("Connected to WiFi ...");
    }

    // Connect client to connect to the server
    gClientIndex  = nclient::NewClient(&gAllocator);                         // Create a new client
    gClientStatus = nclient::Connect(gClientIndex, SERVER_IP, SERVER_PORT);  // Connect to the server

    // This is where you would set up your hardware, peripherals, etc.
    npin::SetPinMode(2, ncore::npin::ModeOutput);  // Set the LED pin as output

    nserial::Println("Bed Presence ...");
}

// Constants
const s32 R_BITS    = 10;                          // ADC resolution (bits)
const f32 ADC_STEPS = 1.0f / ((1 << R_BITS) - 1);  // Number of steps (2^R_BITS - 1)

const u8 kLeftSideGPIO  = 10;  // Pin number for the left side sensor
const u8 kRightSideGPIO = 12;  // Pin number for the right side sensor

// Convert ADC value to a value between 0.0 and 1.0
// Formula: presence = (adc_value / ADC_STEPS)
inline float ToPresence(s32 adc_value) { return (static_cast<float>(adc_value) * ADC_STEPS); }

static nsensor::SensorPacket_t gSensorPacket;  // Sensor packet for sending data
static u16                     gSequence = 0;  // Sequence number for the packet
static const u8                kVersion  = 1;  // Version number for the packet

// Main loop of the application
void loop()
{
    gWifiStatus = nwifi::Status();  // Get the current WiFi status
    if (gWifiStatus == nstatus::Connected)
    {
        nserial::Println("[Loop] Connected to WiFi ...");

        gClientStatus = nclient::Connected(gClientIndex);
        if (gClientStatus == nstatus::Connected)
        {
            nserial::Println("[Loop] Connected to Server ...");

            const s32 left_side          = nadc::AnalogRead(kLeftSideGPIO);  // Read the left side sensor value
            const f32 left_side_presence = ToPresence(left_side);            // Convert the sensor value to voltage

            const s32 right_side          = nadc::AnalogRead(kRightSideGPIO);  // Read the right side sensor value
            const s32 right_side_presence = ToPresence(right_side);            // Convert the sensor value to voltage

            // Write a custom (binary-format) network message
            gSensorPacket.begin(gSequence++, kVersion);
            gSensorPacket.write_info(nsensor::DeviceLocation::Bedroom1, nsensor::DeviceLabel::BedPresence);                                                    // Write the header for the left side sensor
            gSensorPacket.write_sensor_value(nsensor::SensorType::Presence, nsensor::SensorChannel::Channel0, nsensor::SensorState::On, left_side_presence);   // Write the left side sensor value
            gSensorPacket.write_sensor_value(nsensor::SensorType::Presence, nsensor::SensorChannel::Channel1, nsensor::SensorState::On, right_side_presence);  // Write the left side sensor value
            gSensorPacket.finalize();

            nclient::Write(gClientIndex, gSensorPacket.Data, gSensorPacket.Size);  // Send the sensor packet to the server
        }
        else
        {  // If the client is not connected, try to reconnect
            nserial::Println("[Loop] Connecting to server ...");
            gClientStatus = nclient::Connect(gClientIndex, SERVER_IP, SERVER_PORT, 5000);  // Reconnect to the server (already has timeout=5 seconds)
        }
    }
    else
    {
        ntimer::Delay(2000);  // Wait for 2 seconds before trying to reconnect
        // nwifi::Reconnect();   // Reconnect to the WiFi network
    }

    ntimer::Delay(10000);  // Wait for 10 seconds
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