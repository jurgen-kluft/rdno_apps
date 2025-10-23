#include "rd03d/c_rd03d.h"
#include "rdno_sensors/c_rd03d.h"

#include "rdno_core/c_gpio.h"
#include "rdno_wifi/c_node.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"

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
    ntask::result_t app_main(ntask::state_t* state);

    namespace ntask
    {
        // the main program to execute sensor reading
        program_t create_main_program(ntask::executor_t* exec)
        {
            program_t main_program = program(exec, "rd03d main program");
            op_begin(exec, main_program);
            {
                op_run_periodic(exec, app_main, 100);  // every 100 ms
            }
            op_end(exec);
            return main_program;
        }
    }  // namespace ntask

    namespace napp
    {
        void setup(ntask::executor_t* exec, ntask::state_t* state)
        {
            state->app = &gAppState;

            // Initialize RD03D sensor with rx and tx pin
            nsensors::nrd03d::begin(20, 21);

            ntask::program_t main_program = ntask::create_main_program(exec);
            nnode::initialize(exec, main_program, state);
        }
    }  // namespace napp
}  // namespace ncore

namespace ncore
{
    ntask::result_t app_main(ntask::state_t* state)
    {
        ncore::state_app_t* appState = state->app;

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
