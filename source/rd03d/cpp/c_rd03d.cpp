#include "rd03d/c_rd03d.h"

#include "rdno_core/c_gpio.h"
#include "rdno_wifi/c_node.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"
#include "rdno_core/c_task.h"

#include "rdno_sensors/c_rd03d.h"

#include "common/c_common.h"

namespace ncore
{
    struct state_app_t
    {
    };

    state_app_t gAppState;
}  // namespace ncore

namespace ncore
{
    namespace napp
    {
        ntask::result_t func_read(state_t* state)
        {
            if (nsensors::nrd03d::update())
            {
                nsensors::nrd03d::target_t tgt[3];
                for (s8 i = 0; i < 3; ++i)
                {
                    nsensors::nrd03d::getTarget(i, tgt[i]);
                }

                // nserial::println("-------------------------");
                // nserial::printf("X: %d, %d, %d\n", va_t((s32)tgt[0].x), va_t((s32)tgt[1].x), va_t((s32)tgt[2].x));
                // nserial::printf("Y: %d, %d, %d\n", va_t((s32)tgt[0].y), va_t((s32)tgt[1].y), va_t((s32)tgt[2].y));
                // nserial::printf("S: %d, %d, %d\n", va_t((s32)tgt[0].s), va_t((s32)tgt[1].s), va_t((s32)tgt[2].s));

                // A sensor packet size for max 3 targets
                // x=s16, y=s16, z=s16 = 9 bytes
                // total = 2 + 3*9 = 29 bytes
            }

            return ntask::RESULT_OK;
        }

        // the main program to execute sensor reading
        ntask::periodic_t gReadPeriodic(100);  // Every 100 ms
        void              main_program(ntask::scheduler_t* exec, state_t* state)
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

        void presetup()
        {
            // This is where you would set up gpio pins
        }

        void setup(state_t* state)
        {
            // Initialize RD03D sensor with rx and tx pin
            nsensors::nrd03d::begin(20, 21);

            ntask::set_main(state, &gAppTask, &gMainProgram);
            nnode::initialize(state, &gAppTask);
        }

        void tick(state_t* state) 
        {
            ntask::tick(state, &gAppTask);
        }

    }  // namespace napp
}  // namespace ncore
