#include "airquality/c_airquality.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_gpio.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"
#include "rdno_core/c_wire.h"

#include "rdno_wifi/c_tcp.h"
#include "rdno_wifi/c_node.h"

#include "common/c_common.h"

#include "rdno_sensors/c_bh1750.h"
#include "rdno_sensors/c_bme280.h"
#include "rdno_sensors/c_scd4x.h"

#define ENABLE_BH1750
#define ENABLE_BME280
#define ENABLE_SCD41

namespace ncore
{
    struct state_app_t
    {
        npacket::packet_t gSensorPacket;  // Sensor packet for sending data
        const u8          kVersion = 1;   // Version number for the packet

        u16 gLastLux = 0;  // Last read light intensity value

        s8  gLastBme_temp_s8  = 0;
        u16 gLastBme_pres_u16 = 0;
        u8  gLastBme_humi_u8  = 0;

        u16 gLastScd_co2     = 0;
        s8  gLastScd_temp_s8 = 0;
        u8  gLastScd_humi_u8 = 0;
    };
    state_app_t gAppState;

    static bool trigger_read(const u64 currentTimeInMillis, u64& lastReadInMillis, const u64 readPeriodInMillis)
    {
        if (currentTimeInMillis - lastReadInMillis >= readPeriodInMillis)
        {
            lastReadInMillis -= currentTimeInMillis;
            if (currentTimeInMillis - lastReadInMillis >= readPeriodInMillis)
            {
                // If the gap is too large, hard reset to the current time
                lastReadInMillis = currentTimeInMillis;
            }
            return true;
        }
        return false;
    }

    ntask::result_t read_bh1750(state_t* state)
    {
#ifdef ENABLE_BH1750
        // TODO whenever a sensor cannot be read (faulty?) we need to know so that we can
        //      send a 'state' packet that indicates the sensor is not working.

        // Read the BH1750 sensor data
        u16        lux          = 0;
        const bool valid_bh1750 = nsensors::updateBH1750(lux);

        if (valid_bh1750)
        {
            if (gAppState.gLastLux != lux)
            {
                gAppState.gLastLux = lux;

                u16 id;
                if (nconfig::get_uint16(state->config, nconfig::PARAM_ID_LUX, id))
                {
                    // Write a custom (binary-format) network message
                    gAppState.gSensorPacket.begin();

                    nserial::printf("Light: %d lx\n", va_t((u32)lux));
                    gAppState.gSensorPacket.write_sensor(id, lux);

                    if (gAppState.gSensorPacket.finalize() > 0)
                    {
                        ntcp::write(state->tcp, state->node->tcp_client, gAppState.gSensorPacket.Data, gAppState.gSensorPacket.Size);  // Send the sensor packet to the server
                    }
                }
            }
        }
#endif
        return ntask::RESULT_OK;
    }

    ntask::result_t read_bme280(state_t* state)
    {
#ifdef ENABLE_BME280
        // Read the BME280 sensor data
        f32        bme_temp     = 0.0f;
        f32        bme_pres     = 0.0f;
        f32        bme_humi     = 0.0f;
        const bool valid_bme280 = nsensors::updateBME280(bme_pres, bme_temp, bme_humi);

        if (valid_bme280)
        {
            const s8  bme_temp_s8  = static_cast<s8>(bme_temp);   // Temperature to one signed byte (°C)
            const u16 bme_pres_u16 = static_cast<u16>(bme_pres);  // Pressure to unsigned short (hPa)
            const u8  bme_humi_u8  = static_cast<u8>(bme_humi);   // Humidity to one unsigned byte (%)

            // Write a custom (binary-format) network message
            gAppState.gSensorPacket.begin();

            if (gAppState.gLastBme_temp_s8 != bme_temp_s8)
            {
                u16 id;
                if (nconfig::get_uint16(state->config, nconfig::PARAM_ID_T, id))
                {
                    gAppState.gLastBme_temp_s8 = bme_temp_s8;
                    nserial::printf("Temperature: %d °C\n", va_t((s32)bme_temp_s8));
                    gAppState.gSensorPacket.write_sensor(id, (u16)bme_temp_s8);
                }
            }
            if (gAppState.gLastBme_pres_u16 != bme_pres_u16)
            {
                u16 id;
                if (nconfig::get_uint16(state->config, nconfig::PARAM_ID_P, id))
                {
                    gAppState.gLastBme_pres_u16 = bme_pres_u16;
                    nserial::printf("Pressure: %d hPa\n", va_t((u32)bme_pres_u16));
                    gAppState.gSensorPacket.write_sensor(id, (u16)bme_pres_u16);
                }
            }
            if (gAppState.gLastBme_humi_u8 != bme_humi_u8)
            {
                u16 id;
                if (nconfig::get_uint16(state->config, nconfig::PARAM_ID_H, id))
                {
                    gAppState.gLastBme_humi_u8 = bme_humi_u8;
                    nserial::printf("Humidity: %d %%\n", va_t((u32)bme_humi_u8));
                    gAppState.gSensorPacket.write_sensor(id, (u16)bme_humi_u8);
                }
            }

            if (gAppState.gSensorPacket.finalize() > 0)
            {
                ntcp::write(state->tcp, state->node->tcp_client, gAppState.gSensorPacket.Data, gAppState.gSensorPacket.Size);  // Send the sensor packet to the server
            }
        }

#endif
        return ntask::RESULT_OK;
    }

    ntask::result_t read_scd41(state_t* state)
    {
#ifdef ENABLE_SCD41
        // Read the SCD41 sensor data
        f32        scd_humi    = 0.0f;  // Initialize humidity value for SCD41
        f32        scd_temp    = 0.0f;  // Initialize temperature value for SCD41
        u16        scd_co2     = 0;     // Initialize CO2 value
        const bool valid_scd41 = nsensors::updateSCD41(scd_humi, scd_temp, scd_co2);

        if (valid_scd41)
        {
            // Write a custom (binary-format) network message
            gAppState.gSensorPacket.begin();

            const s8 scd_temp_s8 = static_cast<s8>(scd_temp);  // Temperature to one signed byte (°C)
            const u8 scd_humi_u8 = static_cast<u8>(scd_humi);  // Humidity to one unsigned byte (%)

            if (gAppState.gLastScd_co2 != scd_co2)
            {
                gAppState.gLastScd_co2 = scd_co2;
                u16 id;
                if (nconfig::get_uint16(state->config, nconfig::PARAM_ID_CO2, id))
                {
                    nserial::printf("SCD CO2: %d ppm\n", va_t((u32)scd_co2));
                    gAppState.gSensorPacket.write_sensor(id, (u16)scd_co2);
                }
            }
            if (gAppState.gLastScd_temp_s8 != scd_temp_s8)
            {
                gAppState.gLastScd_temp_s8 = scd_temp_s8;
                u16 id;
                if (nconfig::get_uint16(state->config, nconfig::PARAM_ID_T, id))
                {
                    nserial::printf("SCD Temperature: %d °C\n", va_t((s32)scd_temp_s8));
                    gAppState.gSensorPacket.write_sensor(id, (u16)scd_temp_s8);
                }
            }
            if (gAppState.gLastScd_humi_u8 != scd_humi_u8)
            {
                gAppState.gLastScd_humi_u8 = scd_humi_u8;
                u16 id;
                if (nconfig::get_uint16(state->config, nconfig::PARAM_ID_H, id))
                {
                    nserial::printf("SCD Humidity: %d %%\n", va_t((u32)scd_humi_u8));
                    gAppState.gSensorPacket.write_sensor(id, (u16)scd_humi_u8);
                }
            }

            if (gAppState.gSensorPacket.finalize() > 0)
            {
                ntcp::write(state->tcp, state->node->tcp_client, gAppState.gSensorPacket.Data, gAppState.gSensorPacket.Size);  // Send the sensor packet to the server
            }
        }
#endif

        return ntask::RESULT_OK;
    }
}  // namespace ncore

namespace ncore
{
    namespace napp
    {
        ntask::periodic_t periodic_bh1750(1000); // Every 1000 ms
        ntask::periodic_t periodic_bme280(1000); // Every 1000 ms
        ntask::periodic_t periodic_scd41(2000);  // Every 2000 ms

        void main_program(ntask::scheduler_t* exec, state_t* state)
        {
            if (ntask::is_first_call(exec))
            {
                ntask::init_periodic(exec, periodic_bh1750);
                ntask::init_periodic(exec, periodic_bme280);
                ntask::init_periodic(exec, periodic_scd41);
            }

#ifdef ENABLE_BH1750
            if (ntask::periodic(exec, periodic_bh1750))
            {
                ntask::call(exec, read_bh1750);
            }
#endif
#ifdef ENABLE_BME280
            if (ntask::periodic(exec, periodic_bme280))
            {
                ntask::call(exec, read_bme280);
            }
#endif
#ifdef ENABLE_SCD41
            if (ntask::periodic(exec, periodic_scd41))
            {
                ntask::call(exec, read_scd41);
            }
#endif
        }
        ntask::program_t gMainProgram(main_program);
        state_task_t     gAppTask;

        void setup(state_t* state)
        {
#ifdef ENABLE_BH1750
            nsensors::initBH1750();  // Initialize the BH1750 sensor
#endif
#ifdef ENABLE_BME280
            nsensors::initBME280();  // Initialize the BME280 sensor
#endif
#ifdef ENABLE_SCD41
            nsensors::initSCD41();  // Initialize the SCD4X sensor
#endif

            ntask::set_main(state, &gAppTask, &gMainProgram);
            nnode::initialize(state, &gAppTask);
        }

        void tick(state_t* state)
        {
            ntask::tick(state, &gAppTask);
        }

    }  // namespace napp
}  // namespace ncore