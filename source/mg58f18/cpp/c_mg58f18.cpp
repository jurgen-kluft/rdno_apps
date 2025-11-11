#include "mg58f18/c_mg58f18.h"

#include "rdno_core/c_gpio.h"
#include "rdno_wifi/c_node.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"

#include "rdno_sensors/c_mg58f18.h"

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
        // the main program to execute sensor reading
        bool gLastDetected = false;
        void main_program(ntask::scheduler_t* exec, state_t* state)
        {
            if (ntask::is_first_call(exec)) {}

            bool detected = nsensors::nmg58f18::is_detecting(5);
            if (gLastDetected != detected)
            {
                gLastDetected = detected;
                nserial::println(detected ? "Presence!" : "No presence.");
            }
            else
            {
                ntimer::delay(50);
            }
        }
        ntask::program_t gMainProgram(main_program);
        state_task_t     gAppTask;

        void presetup() 
        { 
            nsensors::nmg58f18::initialize(20, 21, 5); 
        }

        void setup(state_t* state)
        {
            ntask::set_main(state, &gAppTask, &gMainProgram);
            nnode::initialize(state, &gAppTask);
        }

        void tick(state_t* state) { ntask::tick(state, &gAppTask); }

    }  // namespace napp
}  // namespace ncore
