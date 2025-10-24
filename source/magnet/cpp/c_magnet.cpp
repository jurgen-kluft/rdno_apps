#include "magnet/c_magnet.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_gpio.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_node.h"
#include "rdno_core/c_nvstore.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"

#include "rdno_sensors/c_ys312.h"

#include "common/c_common.h"

namespace ncore
{
    struct state_app_t
    {
// #define USE_MAGNET_SENSOR
#define USE_PIR_SENSOR

#ifdef USE_MAGNET_SENSOR
        const s8  m_input_pin      = 2;    // GPIO pin connected to the magnet sensor
        const s32 gMagnetThreshold = 100;  // Threshold value for magnet detection, < 100 means magnet is closed, >= 100 means magnet is open
        s32       lastMagnetValue  = -1;   // Last read value of the magnet sensor
#endif

#ifdef USE_PIR_SENSOR
        const s8  m_input_pin   = 2;    // GPIO pin connected to the magnet sensor
        const s32 gPIRThreshold = 500;  // Threshold value for PIR detection, < 500 means no motion, >= 500 means motion detected
        s32       lastPIRValue  = -1;   // Last read value of the PIR sensor
#endif
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

            // This is where you would set up your hardware, peripherals, etc.
            ngpio::set_pinmode(gAppState.m_input_pin, ncore::ngpio::ModeInput);  // Set D0 pin as input

            // the main program to execute sensor reading
            ntask::program_t main_program = program(exec, "magnet main program");
            op_begin(exec, main_program);
            {
                op_run_periodic(exec, app_main, 100);  // every 100 ms
                op_return(exec);
            }
            op_end(exec);

            nnode::initialize(exec, main_program, state);
            nserial::println("Setup done...");
        }
    }  // namespace napp
}  // namespace ncore

namespace ncore
{
    ntask::result_t app_main(ntask::state_t* state)
    {
        ncore::state_app_t* appState = state->app;
#ifdef USE_MAGNET_SENSOR
        const s32 magnetValue = ngpio::read_analog(appState->m_input_pin) < appState->gMagnetThreshold ? 1 : 0;  // Read the magnet sensor
        if (magnetValue != appState->lastMagnetValue)
        {
            nserial::printf("Magnet: %s\n", va_t(magnetValue == 1 ? "On" : "Off"));
            appState->lastMagnetValue = magnetValue;
        }
#endif

#ifdef USE_PIR_SENSOR
        u16 pirValue = 0;
        if (nsensors::nys312::read(appState->m_input_pin, &pirValue))
        {
            nserial::printf("PIR: %d\n", va_t((s32)pirValue));
            appState->lastPIRValue = pirValue;
        }
#endif
    
        return ntask::RESULT_OK;
    }
}  // namespace ncore
