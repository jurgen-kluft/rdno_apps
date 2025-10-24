#include "humanpresence/c_humanpresence.h"

#include "rdno_core/c_app.h"
#include "rdno_core/c_config.h"
#include "rdno_core/c_malloc.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_node.h"
#include "rdno_wifi/c_tcp.h"
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
        npacket::packet_t gSensorPacket;                // Sensor packet for sending data
        u16               gSequence           = 0;      // Sequence number for the packet
        const u8          kVersion            = 1;      // Version number for the packet
        s16               gLastDistanceInCm   = 32768;  // Last distance value read from the sensor
        s8                gLastPresence       = 0;      // Last presence value read from the sensor
        u64               gLastPresenceStream = 0;      // Last presence value read from the sensor
        s8                gLastPresence0      = 64;
        s8                gLastPresence1      = 0;
    };

    state_app_t gAppState;
}  // namespace ncore

namespace ncore
{
    namespace napp
    {
        ntask::result_t func_read(state_t* state)
        {
            ncore::state_app_t* appState = state->app;

            // Read the HMMD sensor data
            s8  presence     = appState->gLastPresence;
            u16 distanceInCm = appState->gLastDistanceInCm;
            if (nsensors::readHMMD2(&presence, &distanceInCm))
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
#endif
                if ((appState->gLastPresenceStream & 0x8000000000000000) == 0)
                {
                    appState->gLastPresence0 -= 1;
                }
                else
                {
                    appState->gLastPresence1 -= 1;
                }

                // Write a custom (binary-format) network message
                appState->gSensorPacket.begin();
                if (presence != 0)
                {
                    appState->gLastPresenceStream = (appState->gLastPresenceStream << 1) | 1;
                    appState->gLastPresence1 += 1;
                }
                else
                {
                    appState->gLastPresenceStream = (appState->gLastPresenceStream << 1) | 0;
                    appState->gLastPresence0 += 1;
                }

                // Presence is true if in the stream of last 64 samples more than 56 samples were 'presence detected'
                presence = (appState->gLastPresence1 > 56) ? 1 : 0;

                if (presence != appState->gLastPresence)
                {
                    appState->gLastPresence = presence;

                    u16 id;
                    if (nconfig::get_uint16(state->config, nconfig::PARAM_ID_P1, id))
                    {
                        appState->gSensorPacket.write_sensor(id, (u16)presence);
                    }
                    if (distanceInCm > 0 && distanceInCm < 3200)
                    {
                        if (nconfig::get_uint16(state->config, nconfig::PARAM_ID_D1, id))
                        {
                            appState->gLastDistanceInCm = distanceInCm;
                            appState->gSensorPacket.write_sensor(id, (u16)distanceInCm);
                        }
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
                        ntcp::write(state->tcp, state->node->tcp_client, appState->gSensorPacket.Data, appState->gSensorPacket.Size);  // Send the sensor packet to the server
                    }
                }
            }
            return ntask::RESULT_OK;
        }

        ntask::periodic_t gReadPeriodic(100);  // Every 100 ms
        void main_program(ntask::scheduler_t* exec, state_t* state)
        {
            if (ntask::is_first_call(exec))
            {
                ntask::init_periodic(exec, gReadPeriodic);
            }
            else
            {
                if (ntask::periodic(exec, gReadPeriodic))
                {
                    ntask::call(exec, func_read);
                }
            }
        }
        ntask::program_t gMainProgram(main_program);
        state_task_t     gAppTask;

        void setup(state_t* state)
        {
            state->app = &gAppState;

            // Initialize the sensors
            const u8 rx = 15;            // RX pin for HMMD
            const u8 tx = 16;            // TX pin for HMMD
            nsensors::initHMMD(rx, tx);  // Initialize the HMMD sensor

            ntask::set_main(state, &gAppTask, &gMainProgram);
            nnode::initialize(state, &gAppTask);

            nserial::println("Setup done...");
        }

        void tick(state_t* state)
        {
            ntask::tick(state, &gAppTask);
        }

    }  // namespace napp
}  // namespace ncore
