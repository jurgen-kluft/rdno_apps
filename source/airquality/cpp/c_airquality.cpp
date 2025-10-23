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

    ntask::result_t read_bh1750(ntask::state_t* state)
    {
#ifdef ENABLE_BH1750
        // TODO whenever a sensor cannot be read (faulty?) we need to know so that we can
        //      send a 'state' packet that indicates the sensor is not working.

        // Read the BH1750 sensor data
        u16        lux          = 0;
        const bool valid_bh1750 = nsensors::updateBH1750(lux);

        if (valid_bh1750)
        {
            ncore::state_app_t* appState = state->app;
            if (appState->gLastLux != lux)
            {
                appState->gLastLux = lux;

                u8 id;
                if (nconfig::get_uint8(state->config, nconfig::PARAM_ID_LUX, id))
                {
                    // Write a custom (binary-format) network message
                    appState->gSensorPacket.begin();

                    nserial::printf("Light: %d lx\n", va_t((u32)lux));
                    appState->gSensorPacket.write_sensor(id, lux);

                    if (appState->gSensorPacket.finalize() > 0)
                    {
                        ntcp::write(state->tcp, state->wifi->tcp_client, appState->gSensorPacket.Data, appState->gSensorPacket.Size);  // Send the sensor packet to the server
                    }
                }
            }
        }
#endif
        return ntask::RESULT_OK;
    }

    ntask::result_t read_bme280(ntask::state_t* state)
    {
#ifdef ENABLE_BME280
        // Read the BME280 sensor data
        f32        bme_temp     = 0.0f;
        f32        bme_pres     = 0.0f;
        f32        bme_humi     = 0.0f;
        const bool valid_bme280 = nsensors::updateBME280(bme_pres, bme_temp, bme_humi);

        if (valid_bme280)
        {
            ncore::state_app_t* appState = state->app;

            const s8  bme_temp_s8  = static_cast<s8>(bme_temp);   // Temperature to one signed byte (°C)
            const u16 bme_pres_u16 = static_cast<u16>(bme_pres);  // Pressure to unsigned short (hPa)
            const u8  bme_humi_u8  = static_cast<u8>(bme_humi);   // Humidity to one unsigned byte (%)

            // Write a custom (binary-format) network message
            appState->gSensorPacket.begin();

            if (appState->gLastBme_temp_s8 != bme_temp_s8)
            {
                u8 id;
                if (nconfig::get_uint8(state->config, nconfig::PARAM_ID_T, id))
                {
                    appState->gLastBme_temp_s8 = bme_temp_s8;
                    nserial::printf("Temperature: %d °C\n", va_t((s32)bme_temp_s8));
                    appState->gSensorPacket.write_sensor(id, (u16)bme_temp_s8);
                }
            }
            if (appState->gLastBme_pres_u16 != bme_pres_u16)
            {
                u8 id;
                if (nconfig::get_uint8(state->config, nconfig::PARAM_ID_P, id))
                {
                    appState->gLastBme_pres_u16 = bme_pres_u16;
                    nserial::printf("Pressure: %d hPa\n", va_t((u32)bme_pres_u16));
                    appState->gSensorPacket.write_sensor(id, (u16)bme_pres_u16);
                }
            }
            if (appState->gLastBme_humi_u8 != bme_humi_u8)
            {
                u8 id;
                if (nconfig::get_uint8(state->config, nconfig::PARAM_ID_H, id))
                {
                    appState->gLastBme_humi_u8 = bme_humi_u8;
                    nserial::printf("Humidity: %d %%\n", va_t((u32)bme_humi_u8));
                    appState->gSensorPacket.write_sensor(id, (u16)bme_humi_u8);
                }
            }

            if (appState->gSensorPacket.finalize() > 0)
            {
                ntcp::write(state->tcp,state->wifi->tcp_client, appState->gSensorPacket.Data, appState->gSensorPacket.Size);  // Send the sensor packet to the server
            }
        }

#endif
        return ntask::RESULT_OK;
    }

    ntask::result_t read_scd41(ntask::state_t* state)
    {
#ifdef ENABLE_SCD41
        // Read the SCD41 sensor data
        f32        scd_humi    = 0.0f;  // Initialize humidity value for SCD41
        f32        scd_temp    = 0.0f;  // Initialize temperature value for SCD41
        u16        scd_co2     = 0;     // Initialize CO2 value
        const bool valid_scd41 = nsensors::updateSCD41(scd_humi, scd_temp, scd_co2);

        if (valid_scd41)
        {
            ncore::state_app_t* appState = state->app;

            // Write a custom (binary-format) network message
            appState->gSensorPacket.begin();

            const s8 scd_temp_s8 = static_cast<s8>(scd_temp);  // Temperature to one signed byte (°C)
            const u8 scd_humi_u8 = static_cast<u8>(scd_humi);  // Humidity to one unsigned byte (%)

            if (appState->gLastScd_co2 != scd_co2)
            {
                appState->gLastScd_co2 = scd_co2;
                u8 id;
                if (nconfig::get_uint8(state->config, nconfig::PARAM_ID_CO2, id))
                {
                    nserial::printf("SCD CO2: %d ppm\n", va_t((u32)scd_co2));
                    appState->gSensorPacket.write_sensor(id, (u16)scd_co2);
                }
            }
            if (appState->gLastScd_temp_s8 != scd_temp_s8)
            {
                appState->gLastScd_temp_s8 = scd_temp_s8;
                u8 id;
                if (nconfig::get_uint8(state->config, nconfig::PARAM_ID_T, id))
                {
                    nserial::printf("SCD Temperature: %d °C\n", va_t((s32)scd_temp_s8));
                    appState->gSensorPacket.write_sensor(id, (u16)scd_temp_s8);
                }
            }
            if (appState->gLastScd_humi_u8 != scd_humi_u8)
            {
                appState->gLastScd_humi_u8 = scd_humi_u8;
                u8 id;
                if (nconfig::get_uint8(state->config, nconfig::PARAM_ID_H, id))
                {
                    nserial::printf("SCD Humidity: %d %%\n", va_t((u32)scd_humi_u8));
                    appState->gSensorPacket.write_sensor(id, (u16)scd_humi_u8);
                }
            }

            if (appState->gSensorPacket.finalize() > 0)
            {
                ntcp::write(state->tcp,state->wifi->tcp_client, appState->gSensorPacket.Data, appState->gSensorPacket.Size);  // Send the sensor packet to the server
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
        void setup(ntask::executor_t* exec, ntask::state_t* state)
        {
            state->app = &gAppState;

#ifdef ENABLE_BH1750
            nsensors::initBH1750();  // Initialize the BH1750 sensor
#endif
#ifdef ENABLE_BME280
            nsensors::initBME280();  // Initialize the BME280 sensor
#endif
#ifdef ENABLE_SCD41
            nsensors::initSCD41();  // Initialize the SCD4X sensor
#endif

            // the main program to execute sensor reading
            ntask::program_t main_program = program(exec, "airquality main program");
            op_begin(exec, main_program);
            {
#ifdef ENABLE_BH1750
                op_run_periodic(exec, read_bh1750, 1000);  // every 1 s
#endif
#ifdef ENABLE_BME280
                op_run_periodic(exec, read_bme280, 1000);  // every 1 s
#endif
#ifdef ENABLE_SCD41
                op_run_periodic(exec, read_scd41, 2000);  // every 2 s
#endif
                op_return(exec);
            }
            op_end(exec);

            nnode::initialize(exec, main_program, state);
        }
    }  // namespace napp
}  // namespace ncore