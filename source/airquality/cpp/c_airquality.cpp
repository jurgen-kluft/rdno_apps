
#include "airquality/c_airquality.h"

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

#include "airquality/c_network.secret.h"

#include "rdno_sensors/c_bh1750.h"
#include "rdno_sensors/c_bme280.h"
#include "rdno_sensors/c_scd4x.h"

using namespace ncore;

ncore::linear_alloc_t    gAllocator;  // Linear allocator for memory management
ncore::nvstore::config_t gConfig;     // Configuration structure for non-volatile storage
static u64               gLastSensorReadTimeInMillis = 0;

namespace ncore
{
    s16 key_to_index(str_t const& str)
    {
        s16 param_id = -1;
        switch (str_len(str))
        {
            case 4: param_id = str_eq_n(str, "ssid", 4, false) ? nvstore::PARAM_ID_SSID : -1; break;
            case 8: param_id = str_eq_n(str, "password", 8, false) ? nvstore::PARAM_ID_PASSWORD : -1; break;
            case 7: param_id = str_eq_n(str, "ap_ssid", 7, false) ? nvstore::PARAM_ID_AP_SSID : -1; break;
            case 11:
                param_id = str_eq_n(str, "ap_password", 11, false) ? nvstore::PARAM_ID_AP_PASSWORD : -1;
                param_id = param_id == -1 ? str_eq_n(str, "remote_port", 11, false) ? nvstore::PARAM_ID_REMOTE_PORT : -1 : param_id;
                break;
            case 13: param_id = str_eq_n(str, "remote_server", 13, false) ? nvstore::PARAM_ID_REMOTE_SERVER : -1; break;
        }
        if (param_id < 0)
        {
            s32 value = 0;
            if (from_str(str, &value, 10) && value >= 0 && value < 256)
            {
                param_id = static_cast<s16>(value);
            }
        }
        return param_id;
    }

    void setup_default_config(nvstore::config_t* config)
    {
        nvstore::reset(config);  // Reset the configuration to default values
        const str_t ssid = str_const(WIFI_SSID);
        const str_t pass = str_const(WIFI_PASSWORD);
        nvstore::set_string(config, nvstore::PARAM_ID_SSID, ssid);
        nvstore::set_string(config, nvstore::PARAM_ID_PASSWORD, pass);
        char  ap_ssid_buffer[32];
        str_t ap_ssid = str_mutable(ap_ssid_buffer, 32);
        str_append(ap_ssid, "AirQuality-");
        nsystem::get_unique_id(ap_ssid);
        nvstore::set_string(config, nvstore::PARAM_ID_AP_SSID, ap_ssid);
        const str_t ap_pass = str_const("32768");
        nvstore::set_string(config, nvstore::PARAM_ID_AP_PASSWORD, ap_pass);
        const str_t remote_server = str_const("192.168.8.88");
        nvstore::set_string(config, nvstore::PARAM_ID_REMOTE_SERVER, remote_server);
        const s32 remote_port = 31337;
        nvstore::set_int(config, nvstore::PARAM_ID_REMOTE_PORT, remote_port);
    }
}  // namespace ncore

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

    // Initialize the sensors
    nsensors::initBH1750(&gAllocator, 0x23);  // Initialize the BH1750 sensor with the I2C address 0x23
    nsensors::initBME280(&gAllocator, 0x76);  // Initialize the BME280 sensor with the I2C address 0x76
    nsensors::initSCD41(&gAllocator, 0x62);   // Initialize the SCD4X sensor with the I2C address 0x62

    // This is where you would set up your hardware, peripherals, etc.
    npin::set_pinmode(2, ncore::npin::ModeOutput);  // Set the LED pin as output

    gLastSensorReadTimeInMillis = ntimer::millis();

    nserial::println("Setup done...");
}

static nsensor::SensorPacket_t gSensorPacket;  // Sensor packet for sending data
static u8                      gSequence = 0;  // Sequence number for the packet
static const u8                kVersion  = 1;  // Version number for the packet

// Main loop of the application
void loop()
{
    if (nwifi::node_loop(&gConfig, ncore::key_to_index))
    {
        const u64 currentTimeInMillis = ntimer::millis();
        if (currentTimeInMillis - gLastSensorReadTimeInMillis >= 1 * 1000)  // read+send sensors every second
        {
            gLastSensorReadTimeInMillis = currentTimeInMillis;

            // Read the BH1750 sensor data
            s32 lux = 0;
            nsensors::updateBH1750(lux);
            const u16 lux_u16 = static_cast<u16>(lux);  // Light intensity to unsigned short (lux)

            // Read the BME280 sensor data
            f32 bme_temp = 0.0f;
            f32 bme_pres = 0.0f;
            f32 bme_humi = 0.0f;
            nsensors::updateBME280(bme_temp, bme_pres, bme_humi);
            const s8  bme_temp_s8  = static_cast<s8>(bme_temp);   // Temperature to one signed byte (°C)
            const u16 bme_pres_u16 = static_cast<u16>(bme_pres);  // Pressure to unsigned short (hPa)
            const u8  bme_humi_u8  = static_cast<u8>(bme_humi);   // Humidity to one unsigned byte (%)

            // Read the SCD41 sensor data
            f32 scd_humi = 0.0f;  // Initialize humidity value for SCD41
            f32 scd_temp = 0.0f;  // Initialize temperature value for SCD41
            u16 scd_co2  = 0;     // Initialize CO2 value
            nsensors::updateSCD41(scd_humi, scd_temp, scd_co2);
            const s8 scd_temp_s8 = static_cast<s8>(scd_temp);  // Temperature to one signed byte (°C)
            const u8 scd_humi_u8 = static_cast<u8>(scd_humi);  // Humidity to one unsigned byte (%)

            // Write a custom (binary-format) network message
            gSensorPacket.begin(gSequence++, kVersion);
            gSensorPacket.write_info(nsensor::DeviceLocation::Bedroom | nsensor::DeviceLocation::Location1, nsensor::DeviceLabel::AirQuality);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Light, nsensor::SensorModel::BH1750, nsensor::SensorState::On, lux_u16);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Temperature, nsensor::SensorModel::BME280, nsensor::SensorState::On, bme_temp_s8);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Pressure, nsensor::SensorModel::BME280, nsensor::SensorState::On, bme_pres_u16);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Humidity, nsensor::SensorModel::BME280, nsensor::SensorState::On, bme_humi_u8);
            gSensorPacket.write_sensor_value(nsensor::SensorType::CO2, nsensor::SensorModel::SCD4X, nsensor::SensorState::On, scd_co2);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Humidity, nsensor::SensorModel::SCD4X, nsensor::SensorState::On, scd_humi_u8);
            gSensorPacket.write_sensor_value(nsensor::SensorType::Temperature, nsensor::SensorModel::SCD4X, nsensor::SensorState::On, scd_temp_s8);
            gSensorPacket.finalize();

            nremote::write(gSensorPacket.Data, gSensorPacket.Size);  // Send the sensor packet to the server
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