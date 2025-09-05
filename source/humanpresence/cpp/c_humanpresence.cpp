#include "humanpresence/c_humanpresence.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_dio.h"
#include "rdno_core/c_adc.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_remote.h"
#include "rdno_wifi/c_node.h"
#include "rdno_core/c_nvstore.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_sensor_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"

#include "humanpresence/c_network.secret.h"

#include "rdno_sensors/c_hmmd.h"

using namespace ncore;

ncore::linear_alloc_t    gAllocator;  // Linear allocator for memory management
ncore::nvstore::config_t gConfig;     // Configuration structure for non-volatile storage
u64                      gLastSensorReadTimeInMillis = 0;

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
        str_append(ap_ssid, "HumanPresence-");
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

    // Initialize the sensors
    const u8 rx = 15;            // RX pin for HMMD
    const u8 tx = 16;            // TX pin for HMMD
    nsensors::initHMMD(rx, tx);  // Initialize the HMMD sensor

    // Initialize the WiFi node
    nserial::println("Load config from non-volatile memory.");
    if (!nvstore::load(&gConfig))  // Load configuration from non-volatile storage
    {
        nserial::println("Failed to Load config from non-volatile memory.");
        nserial::println("Initializing default config.");
        setup_default_config(&gConfig);  // Set up default configuration values
        nserial::println("Saving config to non-volatile memory.");
        nvstore::save(&gConfig);  // Save the default configuration to non-volatile storage
    }

    // nserial::println("Node setup started...");
    // nwifi::node_setup(&gConfig, ncore::key_to_index);  // Set up the WiFi node with the configuration

    // This is where you would set up your hardware, peripherals, etc.
    // npin::SetPinMode(2, ncore::npin::ModeOutput);  // Set the LED pin as output

    gLastSensorReadTimeInMillis = ntimer::millis();

    nserial::println("Setup done...");
}

nsensor::SensorPacket_t gSensorPacket;              // Sensor packet for sending data
u16                     gSequence         = 0;      // Sequence number for the packet
const u8                kVersion          = 1;      // Version number for the packet
s16                     gLastDistanceInCm = 32768;  // Last distance value read from the sensor

// Main loop of the application
void loop()
{
    //if (nwifi::node_loop(&gConfig, ncore::key_to_index))
    {
        const u64 currentTimeInMillis = ntimer::millis();
        if (currentTimeInMillis - gLastSensorReadTimeInMillis >= 100)  // 10 times per second
        {
            gLastSensorReadTimeInMillis = currentTimeInMillis;

            // Read the HMMD sensor data
            u8  detection    = 0;
            u16 distanceInCm = 0;
            while (nsensors::readHMMD2(&detection, &distanceInCm))
            {
                if (distanceInCm != gLastDistanceInCm)
                {
                    gLastDistanceInCm = distanceInCm;

                    nserial::print("Presence: ");
                    nserial::println(detection ? "On" : "Off");
                    nserial::print("Distance: ");
                    char  distanceStrBuffer[16];
                    str_t distanceStr = str_mutable(distanceStrBuffer, 16);
                    to_str(distanceStr, (s32)distanceInCm, 10);
                    nserial::print(distanceStr.m_const);
                    nserial::println(" cm");

                    // Write a custom (binary-format) network message
                    gSensorPacket.begin(gSequence++, kVersion);
                    gSensorPacket.write_info(nsensor::DeviceLocation::Bedroom | nsensor::DeviceLocation::Location1, nsensor::DeviceLabel::Presence);
                    gSensorPacket.write_sensor_value(nsensor::SensorType::Presence, nsensor::SensorModel::HMMD, nsensor::SensorState::On, detection);
                    gSensorPacket.write_sensor_value(nsensor::SensorType::Distance, nsensor::SensorModel::HMMD, nsensor::SensorState::On, distanceInCm);
                    gSensorPacket.finalize();
                    // nremote::write(gSensorPacket.Data, gSensorPacket.Size);  // Send the sensor packet to the server
                }
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