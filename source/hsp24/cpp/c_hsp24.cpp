#include "hsp24/c_hsp24.h"

#include "rdno_core/c_gpio.h"
#include "rdno_wifi/c_node.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"
#include "rdno_core/c_task.h"

#include "rdno_sensors/c_hsp24.h"

#include "common/c_common.h"

#define ENABLE_HSP24

namespace ncore
{
    struct hsp24_data_t
    {
        u64 DetectionBits;
        u8  Detected;
        u8  LastSendDetected;

        void reset()
        {
            DetectionBits    = 0;
            Detected         = 4;  // Unknown state
            LastSendDetected = 3;
        }
    };

    struct state_app_t
    {
        npacket::packet_t          gSensorPacket;  // Sensor packet for sending data
        hsp24_data_t               gCurrentHsp24;
        nsensors::nseeed::hsp24_t* gSensor;
    };

    state_app_t gAppState;
}  // namespace ncore

namespace ncore
{
    namespace napp
    {
        ntask::result_t read_presence(state_t* state)
        {
#ifdef ENABLE_HSP24
            nsensors::nseeed::RadarStatus status;
            if (nsensors::nseeed::getStatus(gAppState.gSensor, status) == nsensors::nseeed::Success)
            {
                // for (s8 i = 0; i < 3; ++i)
                // {
                //     nsensors::nseeed::target_t tgt;
                //     if (nsensors::nseeed::getTarget(i, tgt))
                //     {
                //         gAppState.gCurrentHsp24.DetectionBits[i] = (gAppState.gCurrentHsp24.DetectionBits[i] << 1) | 1;
                //         // nserial::printf("T%d: %d, %d\n", va_t(i), va_t(tgt[i].x), va_t(tgt[i].y));
                //     }
                //     else
                //     {
                //         gAppState.gCurrentHsp24.DetectionBits[i] = (gAppState.gCurrentHsp24.DetectionBits[i] << 1) | 0;
                //     }

                //     u8 detected = gAppState.gCurrentHsp24.Detected[i] & 3;  // Current detection state

                //     const bool dseen = (gAppState.gCurrentHsp24.DetectionBits[i] & 0x3F) == 0x3F;
                //     if (dseen)
                //     {
                //         // Too transition from no-presence to presence we must have seen 3 detections in a row (300 ms)
                //         detected = ((detected << 1) | 1);
                //     }
                //     else
                //     {
                //         const bool dnone = (gAppState.gCurrentHsp24.DetectionBits[i] & 0x3FFFFFFF) == 0;
                //         if (dnone)
                //         {
                //             // To transition from presence to no-presence we must have seen 30 no-detections in a row (3 seconds)
                //             detected = ((detected << 1) | 0);
                //         }
                //     }
                //     gAppState.gCurrentHsp24.Detected[i] = detected;
                //     nserial::printf("T%d detection: %s\n", va_t(i), va_t((detected != 0) ? "PRESENCE" : "ABSENCE"));
                // }
                const u64 detectionBit                = isTargetDetected(status.targetStatus) ? 1 : 0;
                gAppState.gCurrentHsp24.DetectionBits = (gAppState.gCurrentHsp24.DetectionBits << 1) | detectionBit;

                u8 detected = gAppState.gCurrentHsp24.Detected & 3;
                const bool dseen = (gAppState.gCurrentHsp24.DetectionBits & 0x3F) == 0x3F;
                if (dseen)
                {
                    // Too transition from no-presence to presence we must have seen 3 detections in a row (300 ms)
                    detected = ((detected << 1) | 1);
                }
                else
                {
                    const bool dnone = (gAppState.gCurrentHsp24.DetectionBits & 0x3FFFFFFFFFFFFFFFUL) == 0;
                    if (dnone)
                    {
                        // To transition from presence to no-presence we must have seen 30 no-detections in a row (3 seconds)
                        detected = ((detected << 1) | 0);
                    }
                }
                gAppState.gCurrentHsp24.Detected = detected;
                nserial::printf("Status: %s (distance: %d)\n", va_t((detected != 0) ? "PRESENCE" : "ABSENCE"), va_t(status.detectionDistance));
            }
#endif
            return ntask::RESULT_OK;
        }

        ntask::result_t send_presence(state_t* state)
        {
#ifdef ENABLE_HSP24
            // Write a custom (binary-format) network message
            gAppState.gSensorPacket.begin(state->wifi->m_mac);

            const u8 detected = gAppState.gCurrentHsp24.Detected;
            if (gAppState.gCurrentHsp24.LastSendDetected != detected)
            {
                gAppState.gCurrentHsp24.LastSendDetected = detected;
                gAppState.gSensorPacket.write_sensor(npacket::nsensorid::ID_PRESENCE1, detected);
            }

            if (gAppState.gSensorPacket.finalize() > 0)
            {
                nnode::send_sensor_data(state, gAppState.gSensorPacket.Data, gAppState.gSensorPacket.Size);
            }
#endif
            return ntask::RESULT_OK;
        }

        ntask::periodic_t periodic_read_presence(100);
        ntask::periodic_t periodic_send_presence(200 + 3);

        void main_program(ntask::scheduler_t* exec, state_t* state)
        {
            if (ntask::is_first_call(exec))
            {
                ntask::init_periodic(exec, periodic_read_presence);
                ntask::init_periodic(exec, periodic_send_presence);
            }

            // Reading sensor data
#ifdef ENABLE_HSP24
            if (ntask::periodic(exec, periodic_read_presence))
            {
                ntask::call(exec, read_presence);
            }
#endif

            // Sending sensor data
#ifdef ENABLE_HSP24
            if (ntask::periodic(exec, periodic_send_presence))
            {
                ntask::call(exec, send_presence);
            }
#endif
        }
        ntask::program_t gMainProgram(main_program);

        state_task_t gAppTask;

        void presetup()
        {
            // Initialize RD03D sensor with rx and tx pin
            gAppState.gSensor = nsensors::nseeed::create_hsp24(ncore::nserial1::getStream());
            nserial1::begin(nbaud::Rate9600, nconfig::MODE_8N1, 4, 5);
        }

        void setup(state_t* state)
        {
            ntask::set_main(state, &gAppTask, &gMainProgram);
            nnode::initialize(state, &gAppTask);
        }

        void tick(state_t* state) { ntask::tick(state, &gAppTask); }

    }  // namespace napp
}  // namespace ncore
