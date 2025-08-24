
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

static ncore::linear_alloc_t gAllocator;                      // Linear allocator for memory management
static const char*           gHostName = "AirQualityDevice";  // Hostname for the device

#include "rdno_sensors/c_bh1750.h"
#include "rdno_sensors/c_bme280.h"
#include "rdno_sensors/c_scd4x.h"

void setup()
{
    nserial::Begin();  // Initialize serial communication at 115200 baud

    const u32 alloc_size = 1024 * 8;
    byte*     alloc_mem  = gMalloc(alloc_size);  // Allocate memory for the linear allocator
    gAllocator.setup(alloc_mem, alloc_size);     // Set up the linear allocator with the allocated memory

    // Initialize the sensors
    nsensors::initBH1750(&gAllocator, 0x23);  // Initialize the BH1750 sensor with the I2C address 0x23
    nsensors::initBME280(&gAllocator, 0x76);  // Initialize the BME280 sensor with the I2C address 0x76
    nsensors::initSCD41(&gAllocator, 0x62);   // Initialize the SCD4X sensor with the I2C address 0x62

    // Initialize the WiFi module
    nwifi::ConfigIpAddrNone();
    nwifi::SetHostname(gHostName);
    nwifi::BeginEncrypted(WIFI_SSID, WIFI_PASSWORD);  // Connect to the WiFi network

    nstatus::status_t wifiStatus = nwifi::Status();  // Get the current WiFi status
    if (wifiStatus == nstatus::Connected)
    {
        nserial::Println("Connected to WiFi ...");
    }

    // Connect client to the server
    nclient::NewClient();                                     // Create a new client
    nstatus::status_t clientStatus = nclient::Connect(SERVER_IP, SERVER_PORT);  // Connect to the server

    // This is where you would set up your hardware, peripherals, etc.
    npin::SetPinMode(2, ncore::npin::ModeOutput);  // Set the LED pin as output

    nserial::Println("Setup done...");
}

static nsensor::SensorPacket_t gSensorPacket;  // Sensor packet for sending data
static u16                     gSequence = 0;  // Sequence number for the packet
static const u8                kVersion  = 1;  // Version number for the packet

// Main loop of the application
void loop()
{
    nstatus::status_t wifiStatus = nwifi::Status();  // Get the current WiFi status
    if (wifiStatus == nstatus::Connected)
    {
        nserial::Println("[Loop] Connected to WiFi ...");

        nstatus::status_t clientStatus = nclient::Connected();
        if (clientStatus == nstatus::Connected)
        {
            nserial::Println("[Loop] Connected to Server ...");

            // Read the BH1750 sensor data
            s32 lux = 0;
            nsensors::updateBH1750(lux);

            // Read the BME280 sensor data
            f32 temperature = 0.0f;
            f32 pressure    = 0.0f;
            f32 humidity    = 0.0f;
            nsensors::updateBME280(temperature, pressure, humidity);

            // Read the SCD41 sensor data
            f32 humidity_scd    = 0.0f;  // Initialize humidity value for SCD41
            f32 temperature_scd = 0.0f;  // Initialize temperature value for SCD41
            u16 co2_scd         = 0;     // Initialize CO2 value
            nsensors::updateSCD41(humidity_scd, temperature_scd, co2_scd);

            // Write a custom (binary-format) network message
            gSensorPacket.begin(gSequence++, kVersion);
            gSensorPacket.write_info(nsensor::DeviceLocation::Bedroom | nsensor::DeviceLocation::Location1 | nsensor::DeviceLocation::Area1, nsensor::DeviceLabel::AirQuality);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Light, nsensor::SensorModel::BH1750, nsensor::SensorState::On, lux);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Temperature, nsensor::SensorModel::BME280, nsensor::SensorState::On, temperature);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Pressure, nsensor::SensorModel::BME280, nsensor::SensorState::On, pressure);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Humidity, nsensor::SensorModel::BME280, nsensor::SensorState::On, humidity);
            gSensorPacket.write_sensor_value(nsensor::SensorType::CO2, nsensor::SensorModel::SCD4X, nsensor::SensorState::On, co2_scd);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Humidity, nsensor::SensorModel::SCD4X, nsensor::SensorState::On, humidity_scd);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Temperature, nsensor::SensorModel::SCD4X, nsensor::SensorState::On, temperature_scd);
            gSensorPacket.finalize();

            nclient::Write(gSensorPacket.Data, gSensorPacket.Size);  // Send the sensor packet to the server
        }
        else
        {  // If the client is not connected, try to reconnect
            nserial::Println("[Loop] Connecting to server ...");
            clientStatus = nclient::Connect(SERVER_IP, SERVER_PORT, 5000);  // Reconnect to the server (already has timeout=5 seconds)
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