#include "rd03d/c_rd03d.h"

#include "rdno_core/c_gpio.h"
#include "rdno_wifi/c_node.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"
#include "rdno_core/c_task.h"

#include "rdno_sensors/c_rd03d.h"

#include "common/c_common.h"

#define ENABLE_RD03D

namespace ncore
{
    struct rd03d_data_t
    {
        u32 DetectionBits[3];
        u8  Detected[3];
        u8  LastSendDetected[3];

        void reset()
        {
            for (s8 i = 0; i < 3; ++i)
            {
                DetectionBits[i]    = 0;
                Detected[i]         = 4;  // Unknown state
                LastSendDetected[i] = 3;
            }
        }
    };

    struct state_app_t
    {
        npacket::packet_t gSensorPacket;  // Sensor packet for sending data
        rd03d_data_t      gCurrentRd03d;
    };

    state_app_t gAppState;
}  // namespace ncore

namespace ncore
{
    namespace napp
    {
        ntask::result_t read_rd03d(state_t* state)
        {
#ifdef ENABLE_RD03D
            if (nsensors::nrd03d::update())
            {
                for (s8 i = 0; i < 3; ++i)
                {
                    nsensors::nrd03d::target_t tgt;
                    if (nsensors::nrd03d::getTarget(i, tgt))
                    {
                        gAppState.gCurrentRd03d.DetectionBits[i] = (gAppState.gCurrentRd03d.DetectionBits[i] << 1) | 1;
                        // nserial::printf("T%d: %d, %d\n", va_t(i), va_t(tgt[i].x), va_t(tgt[i].y));
                    }
                    else
                    {
                        gAppState.gCurrentRd03d.DetectionBits[i] = (gAppState.gCurrentRd03d.DetectionBits[i] << 1) | 0;
                    }

                    u8 detected = gAppState.gCurrentRd03d.Detected[i] & 3;  // Current detection state

                    const bool dseen = (gAppState.gCurrentRd03d.DetectionBits[i] & 0x3F) == 0x3F;
                    if (dseen)
                    {
                        // Too transition from no-presence to presence we must have seen 3 detections in a row (300 ms)
                        detected = ((detected << 1) | 1);
                    }
                    else
                    {
                        const bool dnone = (gAppState.gCurrentRd03d.DetectionBits[i] & 0x3FFFFFFF) == 0;
                        if (dnone)
                        {
                            // To transition from presence to no-presence we must have seen 30 no-detections in a row (3 seconds)
                            detected = ((detected << 1) | 0);
                        }
                    }
                    gAppState.gCurrentRd03d.Detected[i] = detected;

                    nserial::printf("T%d detection: %s\n", va_t(i), va_t((detected != 0) ? "PRESENCE" : "ABSENCE"));
                }
            }
#endif
            return ntask::RESULT_OK;
        }

        ntask::result_t send_rd03d(state_t* state)
        {
#ifdef ENABLE_RD03D
            // Write a custom (binary-format) network message
            gAppState.gSensorPacket.begin(state->wifi->m_mac);

            for (s8 i = 0; i < 3; ++i)
            {
                const u8 detected = gAppState.gCurrentRd03d.Detected[i];
                if (gAppState.gCurrentRd03d.LastSendDetected[i] != detected)
                {
                    gAppState.gCurrentRd03d.LastSendDetected[i] = detected;
                    gAppState.gSensorPacket.write_sensor(npacket::nsensorid::ID_PRESENCE1 + i, detected);
                }
            }

            if (gAppState.gSensorPacket.finalize() > 0)
            {
                nnode::send_sensor_data(state, gAppState.gSensorPacket.Data, gAppState.gSensorPacket.Size);
            }
#endif
            return ntask::RESULT_OK;
        }

        ntask::periodic_t periodic_read_rd03d(100);
        ntask::periodic_t periodic_send_rd03d(200 + 3);

        void main_program(ntask::scheduler_t* exec, state_t* state)
        {
            if (ntask::is_first_call(exec))
            {
                ntask::init_periodic(exec, periodic_read_rd03d);
                ntask::init_periodic(exec, periodic_send_rd03d);
            }

            // Reading sensor data
#ifdef ENABLE_RD03D
            if (ntask::periodic(exec, periodic_read_rd03d))
            {
                ntask::call(exec, read_rd03d);
            }
#endif

            // Sending sensor data
#ifdef ENABLE_RD03D
            if (ntask::periodic(exec, periodic_send_rd03d))
            {
                ntask::call(exec, send_rd03d);
            }
#endif
        }
        ntask::program_t gMainProgram(main_program);

        state_task_t gAppTask;

        void presetup()
        {
            // Initialize RD03D sensor with rx and tx pin
            nsensors::nrd03d::begin(20, 21);
        }

        void setup(state_t* state)
        {
            ntask::set_main(state, &gAppTask, &gMainProgram);
            nnode::initialize(state, &gAppTask);
        }

        void tick(state_t* state) { ntask::tick(state, &gAppTask); }

    }  // namespace napp
}  // namespace ncore
