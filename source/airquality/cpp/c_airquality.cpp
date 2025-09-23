#include "airquality/c_airquality.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_gpio.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"
#include "rdno_core/c_wire.h"

#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_remote.h"
#include "rdno_wifi/c_node.h"

#include "common/c_common.h"

#include "rdno_sensors/c_environment.h"
#include "rdno_sensors/c_bh1750.h"
#include "rdno_sensors/c_bme280.h"
#include "rdno_sensors/c_scd4x.h"

using namespace ncore;

ncore::linear_alloc_t    gAllocator;  // Linear allocator for memory management
ncore::nvstore::config_t gConfig;     // Configuration structure for non-volatile storage
static u64               gLastSensorReadTimeInMillis = 0;

#define SENSOR_LOCATION (nsensor::DeviceLocation::Bedroom | nsensor::DeviceLocation::Location1)

#define ENABLE_BH1750
#define ENABLE_BME280
#define ENABLE_SCD41

static u64 gBme280ReadPeriodInMillis = 60 * 1000;  // Frequency to read BME280 sensor (every 60 seconds)
static u64 gBh1750ReadPeriodInMillis = 2 * 1000;   // Frequency to read BH1750 sensor (every 2 seconds)
static u64 gScd41ReadPeriodInMillis  = 5 * 1000;   // Frequency to read SCD41 sensor (every 5 seconds)

static u64 gBme280LastReadInMillis = 0;  // Frequency to read BME280 sensor (60 seconds)
static u64 gBh1750LastReadInMillis = 0;  // Frequency to read BH1750 sensor (5 times per second)
static u64 gScd41LastReadInMillis  = 0;  // Frequency to read SCD41 sensor (5 seconds)

static bool trigger_read(const u64 currentTimeInMillis, u64& lastReadInMillis, const u64 readPeriodInMillis)
{
    if (currentTimeInMillis - lastReadInMillis >= readPeriodInMillis)
    {
        lastReadInMillis = currentTimeInMillis;
        return true;
    }
    return false;
}

void setup()
{
    nserial::begin();      // Initialize serial communication at 115200 baud
    nwire::begin(21, 22);  // Initialize I2C communication

    const u32 alloc_size = 1024 * 8;
    byte*     alloc_mem  = nsystem::malloc(alloc_size);  // Allocate memory for the linear allocator
    gAllocator.setup(alloc_mem, alloc_size);             // Set up the linear allocator with the allocated memory

    // Initialize the WiFi node
    //    if (!nvstore::load(&gConfig))  // Load configuration from non-volatile storage
    {
        setup_default_config(&gConfig);  // Set up default configuration values
        nvstore::save(&gConfig);         // Save the default configuration to non-volatile storage
    }

    // Initialize the WiFi node
    nwifi::node_setup(&gConfig, ncore::key_to_index);

// Initialize the sensors
#ifdef ENABLE_BH1750
    nsensors::initBH1750(&gAllocator);  // Initialize the BH1750 sensor
#endif
#ifdef ENABLE_BME280
    nsensors::initBME280(&gAllocator);  // Initialize the BME280 sensor
#endif
#ifdef ENABLE_SCD41
    nsensors::initSCD41(&gAllocator);  // Initialize the SCD4X sensor
#endif

    // This is where you would set up your hardware, peripherals, etc.
    // npin::set_pinmode(2, ncore::npin::ModeOutput);  // Set the LED pin as output

    gLastSensorReadTimeInMillis = ntimer::millis();

    gBme280LastReadInMillis = gLastSensorReadTimeInMillis;
    gBh1750LastReadInMillis = gLastSensorReadTimeInMillis;
    gScd41LastReadInMillis  = gLastSensorReadTimeInMillis;

    nserial::println("Setup done...");
}

static npacket::packet_t gSensorPacket;  // Sensor packet for sending data
static const u8          kVersion = 1;   // Version number for the packet

static u16 gLastLux = 0;  // Last read light intensity value

static s8  gLastBme_temp_s8  = 0;
static u16 gLastBme_pres_u16 = 0;
static u8  gLastBme_humi_u8  = 0;

static u16 gLastScd_co2     = 0;
static s8  gLastScd_temp_s8 = 0;
static u8  gLastScd_humi_u8 = 0;

// Main loop of the application
void loop()
{
    if (nwifi::node_loop(&gConfig, ncore::key_to_index))
    {
        const u64 currentTimeInMillis = ntimer::millis();
        if (currentTimeInMillis - gLastSensorReadTimeInMillis >= 1 * 100)
        {
            // npin::write_pin(2, true);
            gLastSensorReadTimeInMillis = currentTimeInMillis;

            // Write a custom (binary-format) network message
            gSensorPacket.begin((u32)nwifi::node_timesync(), false);

            // TODO whenever a sensor cannot be read (faulty?) we need to know so that we can
            //      mark the sensor as "error" in the sensor packet

            // Read the BH1750 sensor data
            u16 lux = 0;
#ifdef ENABLE_BH1750
            bool valid_bh1750 = false;
            if (trigger_read(currentTimeInMillis, gBh1750LastReadInMillis, gBh1750ReadPeriodInMillis))
            {
                valid_bh1750 = true;
                nsensors::updateBH1750(lux);
            }
#else
            const bool valid_bh1750 = false;
#endif

            if (valid_bh1750)
            {
                if (gLastLux != lux)
                {
                    gLastLux = lux;
                    nserial::print("Light: ");
                    nserial::print((u32)lux, false);
                    nserial::println(" lx");

                    gSensorPacket.write_value(npacket::ntype::Light, (u64)lux);
                }
            }

            // Read the BME280 sensor data
            f32 bme_temp = 0.0f;
            f32 bme_pres = 0.0f;
            f32 bme_humi = 0.0f;
#ifdef ENABLE_BME280
            bool valid_bme280 = false;
            if (trigger_read(currentTimeInMillis, gBme280LastReadInMillis, gBme280ReadPeriodInMillis))
            {
                valid_bme280 = true;
                nsensors::updateBME280(bme_pres, bme_temp, bme_humi);
            }
#else
            const bool valid_bme280 = false;
#endif
            const s8  bme_temp_s8  = static_cast<s8>(bme_temp);   // Temperature to one signed byte (°C)
            const u16 bme_pres_u16 = static_cast<u16>(bme_pres);  // Pressure to unsigned short (hPa)
            const u8  bme_humi_u8  = static_cast<u8>(bme_humi);   // Humidity to one unsigned byte (%)

            if (valid_bme280)
            {
                if (gLastBme_temp_s8 != bme_temp_s8)
                {
                    gLastBme_temp_s8 = bme_temp_s8;
                    nserial::print("Temperature: ");
                    nserial::print((s32)bme_temp_s8);
                    nserial::println(" °C");
                    gSensorPacket.write_value(npacket::ntype::Temperature, (u64)bme_temp_s8);
                }
                if (gLastBme_pres_u16 != bme_pres_u16)
                {
                    gLastBme_pres_u16 = bme_pres_u16;
                    nserial::print("Pressure: ");
                    nserial::print((u32)bme_pres_u16, false);
                    nserial::println(" hPa");
                    gSensorPacket.write_value(npacket::ntype::Pressure, (u64)bme_pres_u16);
                }
                if (gLastBme_humi_u8 != bme_humi_u8)
                {
                    gLastBme_humi_u8 = bme_humi_u8;
                    nserial::print("Humidity: ");
                    nserial::print((u32)bme_humi_u8, false);
                    nserial::println(" %");
                    gSensorPacket.write_value(npacket::ntype::Humidity, (u64)bme_humi_u8);
                }
            }

            // Read the SCD41 sensor data
            f32 scd_humi = 0.0f;  // Initialize humidity value for SCD41
            f32 scd_temp = 0.0f;  // Initialize temperature value for SCD41
            u16 scd_co2  = 0;     // Initialize CO2 value
#ifdef ENABLE_SCD41
            bool valid_scd41 = false;
            if (trigger_read(currentTimeInMillis, gScd41LastReadInMillis, gScd41ReadPeriodInMillis))
            {
                valid_scd41 = true;
                nsensors::updateSCD41(scd_humi, scd_temp, scd_co2);
            }
#else
            const bool valid_scd41 = false;
#endif
            const s8 scd_temp_s8 = static_cast<s8>(scd_temp);  // Temperature to one signed byte (°C)
            const u8 scd_humi_u8 = static_cast<u8>(scd_humi);  // Humidity to one unsigned byte (%)

            if (valid_scd41)
            {
                if (gLastScd_co2 != scd_co2)
                {
                    gLastScd_co2 = scd_co2;
                    nserial::print("CO2: ");
                    nserial::print((u32)scd_co2, false);
                    nserial::println(" ppm");
                    gSensorPacket.write_value(npacket::ntype::CO2, (u64)scd_co2);
                }
                if (gLastScd_temp_s8 != scd_temp_s8)
                {
                    gLastScd_temp_s8 = scd_temp_s8;
                    nserial::print("SCD Temperature: ");
                    nserial::print((s32)scd_temp_s8, false);
                    nserial::println(" °C");
                    gSensorPacket.write_value(npacket::ntype::Temperature, (u64)scd_temp_s8);
                }
                if (gLastScd_humi_u8 != scd_humi_u8)
                {
                    gLastScd_humi_u8 = scd_humi_u8;
                    nserial::print("SCD Humidity: ");
                    nserial::print((u32)scd_humi_u8, false);
                    nserial::println(" %");
                    gSensorPacket.write_value(npacket::ntype::Humidity, (u64)scd_humi_u8);
                }
            }

            if (gSensorPacket.finalize() > 0)
            {
                nremote::write(gSensorPacket.Data, gSensorPacket.Size);  // Send the sensor packet to the server
            }
        }
        else
        {
            // npin::write_pin(2, false);
        }
    }
    else
    {
        gLastSensorReadTimeInMillis = ntimer::millis();

        gBme280LastReadInMillis = gLastSensorReadTimeInMillis;
        gBh1750LastReadInMillis = gLastSensorReadTimeInMillis;
        gScd41LastReadInMillis  = gLastSensorReadTimeInMillis;

        // Not connected to WiFi
        // npin::write_pin(2, false);
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