
#include "airquality/c_airquality.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_dio.h"
#include "rdno_core/c_adc.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_client.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_sensor_packet.h"

#include "airquality/c_network.secret.h"

using namespace ncore;

static ncore::linear_alloc_t gAllocator;                          // Linear allocator for memory management
static s16                   gClientIndex  = -1;                  // Client index for the TCP client
static nstatus::status_t     gClientStatus = nstatus::Idle;       //
static nstatus::status_t     gWifiStatus   = nstatus::Idle;       //
static const char*           gHostName     = "AirQualityDevice";  // Hostname for the device

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
    nwifi::BeginEncrypted(WIFI_SSID, WIFI_PASSWORD);  // Connect to the WiFi network

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

    nserial::Println("Setup done...");
}

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
        }
        else
        {  // If the client is not connected, try to reconnect
            nserial::Println("[Loop] Connecting to server ...");
            gClientStatus = nclient::Connect(gClientIndex, SERVER_IP, SERVER_PORT, 5000);  // Reconnect to the server (already has timeout=5 seconds)
        }
    }
    else
    {
        ntimer::Delay(10000);  // Wait for 10 seconds
        nwifi::Reconnect();    // Reconnect to the WiFi network
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