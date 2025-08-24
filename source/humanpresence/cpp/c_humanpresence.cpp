#include "humanpresence/c_humanpresence.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_dio.h"
#include "rdno_core/c_adc.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_client.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_sensor_packet.h"

#include "humanpresence/c_network.secret.h"

#include "Arduino.h"

using namespace ncore;

ncore::linear_alloc_t gAllocator;                    // Linear allocator for memory management
const char*           gHostName = "PresenceDevice";  // Hostname for the device

#include "rdno_sensors/c_hmmd.h"

void setup()
{
    nserial::Begin();  // Initialize serial communication at 115200 baud

    const u32 alloc_size = 1024 * 8;
    byte*     alloc_mem  = gMalloc(alloc_size);  // Allocate memory for the linear allocator
    gAllocator.setup(alloc_mem, alloc_size);     // Set up the linear allocator with the allocated memory

    // Initialize the sensors
    const u8 rx = 15;            // RX pin for HMMD
    const u8 tx = 16;            // TX pin for HMMD
    nsensors::initHMMD(rx, tx);  // Initialize the HMMD sensor

    // Initialize the WiFi module
    nwifi::Disconnect();                              // Disconnect from any existing WiFi connections
    ntimer::Delay(1000);                              // Wait for 1 second
    nwifi::BeginEncrypted(WIFI_SSID, WIFI_PASSWORD);  // Connect to the WiFi network

    nstatus::status_t wifiStatus = nwifi::Status();  // Get the current WiFi status
    while (wifiStatus != nstatus::Connected)
    {
        nserial::Println("Connecting to WiFi ...");
        ntimer::Delay(3000);  // Wait for 3 seconds before checking again
        wifiStatus = nwifi::Status();
    }
    nserial::Println("Connected to WiFi ...");

    // Connect client to the server
    bool clientOk = nclient::NewClient();  // Create a new client
    Serial.println("Client: " + String(clientOk));

    // This is where you would set up your hardware, peripherals, etc.
    // npin::SetPinMode(2, ncore::npin::ModeOutput);  // Set the LED pin as output

    nserial::Println("Setup done...");
}

nsensor::SensorPacket_t gSensorPacket;  // Sensor packet for sending data
u16                     gSequence = 0;  // Sequence number for the packet
const u8                kVersion  = 1;  // Version number for the packet

// Main loop of the application
void loop()
{
    if (nwifi::Status() == nstatus::Connected)
    {
        if (nclient::Connected() == nstatus::Connected)
        {
            // Read the HMMD sensor data
            f32 distance = 0.0f;
            if (nsensors::readHMMD(&distance))
            {
                Serial.print("Distance: ");
                Serial.print(distance);
                Serial.println(" cm");

                // Write a custom (binary-format) network message
                // gSensorPacket.begin(gSequence++, kVersion);
                // gSensorPacket.write_info(nsensor::DeviceLocation::Bedroom | nsensor::DeviceLocation::Location1 | nsensor::DeviceLocation::Area1, nsensor::DeviceLabel::Presence);
                // gSensorPacket.write_sensor_value(nsensor::SensorType::Presence, nsensor::SensorModel::HMMD, nsensor::SensorState::On, distance);
                // gSensorPacket.finalize();

                // nclient::Write(gSensorPacket.Data, gSensorPacket.Size);  // Send the sensor packet to the server
            }
        }
        else
        {
            // If the client is not connected -> reconnect
            nserial::Println("[Loop] Connecting to server ...");
            nclient::Connect(SERVER_IP, SERVER_PORT, 5000);  // Reconnect to the server (already has timeout=5 seconds)
            nstatus::status_t clientStatus = nclient::Connected();
            nstatus::status_t wifiStatus   = nwifi::Status();
            while (clientStatus != nstatus::Connected && wifiStatus == nstatus::Connected)
            {
                ntimer::Delay(3000);  // Wait for 3 seconds before checking again
                clientStatus = nclient::Connected();
                wifiStatus   = nwifi::Status();
            }
            if (wifiStatus == nstatus::Connected && clientStatus == nstatus::Connected)
            {
                nserial::Println("[Loop] Connected to server ...");

                IPAddress_t localIP = nclient::LocalIP();
                Serial.println("Local IP: " + String(localIP.m_address[0]) + "." + String(localIP.m_address[1]) + "." + String(localIP.m_address[2]) + "." + String(localIP.m_address[3]));
            }
        }
    }
    else
    {
        nwifi::Disconnect();                              // Disconnect from WiFi
        ntimer::Delay(5000);                              // Wait for 5 seconds
        nwifi::BeginEncrypted(WIFI_SSID, WIFI_PASSWORD);  // Reconnect to WiFi
        nstatus::status_t wifiStatus = nwifi::Status();
        while (wifiStatus != nstatus::Connected)
        {
            nserial::Println("Connecting to WiFi ...");
            ntimer::Delay(3000);  // Wait for 3 seconds before checking again
            wifiStatus = nwifi::Status();
        }

        nserial::Println("[Loop] Connected to WiFi ...");
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