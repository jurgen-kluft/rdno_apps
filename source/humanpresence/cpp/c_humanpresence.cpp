#include "humanpresence/c_humanpresence.h"

#include "rdno_core/c_app.h"
#include "rdno_core/c_config.h"
#include "rdno_core/c_malloc.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_remote.h"
#include "rdno_wifi/c_node.h"
#include "rdno_core/c_nvstore.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"
#include "rdno_core/c_task.h"

#include "common/c_common.h"

#include "rdno_sensors/c_hmmd.h"

namespace ncore
{
    struct state_app_t
    {
        u64               gLastSensorReadTimeInMillis = 0;
        npacket::packet_t gSensorPacket;              // Sensor packet for sending data
        u16               gSequence         = 0;      // Sequence number for the packet
        const u8          kVersion          = 1;      // Version number for the packet
        s16               gLastDistanceInCm = 32768;  // Last distance value read from the sensor
        s8                gLastPresence     = 0;      // Last presence value read from the sensor
    };

    state_app_t gAppState;
}  // namespace ncore

namespace ncore
{
    ntask::result_t app_main(ntask::state_t* state);

    namespace napp
    {
        void setup(ntask::executor_t* exec, ntask::state_t* state)
        {
            state->app = &gAppState;

            // Initialize the sensors
            const u8 rx = 15;            // RX pin for HMMD
            const u8 tx = 16;            // TX pin for HMMD
            nsensors::initHMMD(rx, tx);  // Initialize the HMMD sensor

            // the main program to execute sensor reading
            ntask::program_t main_program = program(exec, "human presence main program");
            xbegin(exec, main_program);
            {
                xrun_periodic(exec, app_main, 100);  // every 100 ms
                xreturn(exec);
            }
            xend(exec);

            nnode::connected(exec, main_program, state);

            nserial::println("Setup done...");
        }
    }  // namespace napp
}  // namespace ncore

namespace ncore
{
    ntask::result_t app_main(ntask::state_t* state)
    {
        ncore::state_app_t* appState = state->app;

        // Read the HMMD sensor data
        s8  presence     = appState->gLastPresence;
        u16 distanceInCm = appState->gLastDistanceInCm;
        s32 count        = 0;
        while (nsensors::readHMMD2(&presence, &distanceInCm))
        {
#if 0
            nserial::print("Read Presence: ");
            nserial::println(presence == 1 ? "On" : "Off");
            nserial::print("Read Distance: ");
            char  distanceStrBuffer[16];
            str_t distanceStr = str_mutable(distanceStrBuffer, 16);
            to_str(distanceStr, (s32)distanceInCm, 10);
            nserial::print(distanceStr.m_const);
            nserial::println(" cm");
            nserial::print("Count = ");
            nserial::print(count);
            nserial::println("");
            count++;
#endif
        }

        // Write a custom (binary-format) network message
        appState->gSensorPacket.begin();
        if (presence != appState->gLastPresence)
        {
            u8 id;
            nconfig::get_uint8(state->config, nconfig::PARAM_ID_P1, id);

            appState->gLastPresence = presence;
            appState->gSensorPacket.write_sensor(id, nsensor_type::Presence1, (u64)presence);
        }
        if ((distanceInCm > 0 && distanceInCm < 2000) && distanceInCm != appState->gLastDistanceInCm)
        {
            u8 id;
            nconfig::get_uint8(state->config, nconfig::PARAM_ID_D1, id);

            appState->gLastDistanceInCm = distanceInCm;
            appState->gSensorPacket.write_sensor(id, nsensor_type::Distance1, (u64)distanceInCm);
        }

        if (appState->gSensorPacket.finalize() > 0)
        {
#ifdef TARGET_DEBUG
            nserial::print("Sending presence=");
            nserial::print(presence == 1 ? "On" : "Off");
            nserial::print(", distance=");
            char  distanceStrBuffer[16];
            str_t distanceStr = str_mutable(distanceStrBuffer, 16);
            to_str(distanceStr, (s32)distanceInCm, 10);
            nserial::print(distanceStr.m_const);
            nserial::println(" cm");
#endif
            nremote::write(appState->gSensorPacket.Data, appState->gSensorPacket.Size);  // Send the sensor packet to the server
        }

        return ntask::RESULT_OK;
    }
}  // namespace ncore

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