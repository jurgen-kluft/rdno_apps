#include "magnet/c_magnet.h"

#include "rdno_core/c_gpio.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_node.h"
#include "rdno_wifi/c_udp.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_state.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_state.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"

#include "common/c_common.h"

#ifndef TARGET_ESP8266
    #define A0 17
#else
    #include "Arduino.h"
#endif

namespace ncore
{
    struct state_app_t
    {
        const s8 m_switch_pin   = 13;  // GPIO pin connected to switch
        const s8 m_poweroff_pin = 16;  // GPIO pin connected to end line
    };
    state_app_t  gAppState;
    state_task_t gAppTask;
}  // namespace ncore

namespace ncore
{
    namespace napp
    {
        void main_program(ntask::scheduler_t* exec, state_t* state)
        {
            // Read the state of the switch and send open or closed
            const s8  switch_state  = ngpio::read_digital(gAppState.m_switch_pin);
            const s32 battery_level = (ngpio::read_analog(A0) * 33 * 10 / 1023);  // Percentage

            npacket::packet_t pkt;
            pkt.begin(state->wifi->m_mac);
            pkt.write_sensor(npacket::nsensorid::ID_SWITCH, (u16)switch_state);    // Open/Close
            pkt.write_sensor(npacket::nsensorid::ID_BATTERY, (u16)battery_level);  // Battery level
            pkt.finalize();

            nnode::send_sensor_data(state, pkt.Data, pkt.Size);

            ntimer::delay(100);
            ngpio::write_digital(gAppState.m_poweroff_pin, 0);  // Switch off supply
        }
        ntask::program_t gMainProgram(main_program);

        void setup(state_t* state)
        {
            // This is where you would set up your hardware, peripherals, etc.
            ngpio::set_pinmode(gAppState.m_switch_pin, ncore::ngpio::ModeInput);
            ngpio::set_pinmode(gAppState.m_poweroff_pin, ncore::ngpio::ModeOutput);

            // If we do not have a valid configuration, we need to launch configuration mode
            ntask::set_main(state, &gAppTask, &gMainProgram);
            nnode::initialize(state, &gAppTask);

            ngpio::write_digital(gAppState.m_poweroff_pin, 1);

            nserial::println("Setup done...");
        }
 
        void tick(state_t* state) { ntask::tick(state, &gAppTask); }

    }  // namespace napp
}  // namespace ncore
