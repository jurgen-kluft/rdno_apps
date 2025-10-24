#include "magnet/c_magnet.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_linear_allocator.h"
#include "rdno_core/c_gpio.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_node.h"
#include "rdno_wifi/c_udp.h"
#include "rdno_core/c_nvstore.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_state.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"
#include "rdno_core/c_timer.h"

#include "rdno_sensors/c_ys312.h"

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
    state_app_t gAppState;
}  // namespace ncore

namespace ncore
{
    namespace napp
    {
        void setup(state_t* state)
        {
            state->app = &gAppState;

            // This is where you would set up your hardware, peripherals, etc.
            ngpio::set_pinmode(gAppState.m_switch_pin, ncore::ngpio::ModeInput);
            ngpio::set_pinmode(gAppState.m_poweroff_pin, ncore::ngpio::ModeOutput);

            ngpio::write_digital(gAppState.m_poweroff_pin, 1);

            nserial::println("Setup done...");
        }

        void tick(state_t* state)
        {
            // Read the state of the switch and send open or closed
            const s8  switch_state  = ngpio::read_digital(gAppState.m_switch_pin);
            const s32 battery_level = (ngpio::read_analog(A0) * 42 * 10 / 10230);

            // Send message UDP
            u32 remote_ip;
            nconfig::get_uint32(state->config, nconfig::PARAM_ID_REMOTE_IP, remote_ip);
            u16 remote_port;
            nconfig::get_uint16(state->config, nconfig::PARAM_ID_REMOTE_PORT, remote_port);

            IPAddress_t remote_ip_addr;
            remote_ip_addr.from(remote_ip);

            u16 switch_id;
            nconfig::get_uint16(state->config, nconfig::PARAM_ID_OC, switch_id);

            nudp::sock_t udp_sock;
            nudp::open(udp_sock, 42420);

            npacket::packet_t pkt;
            pkt.begin();
            pkt.write_sensor(switch_id, (u16)switch_state);  // Open/Close
            pkt.finalize();
            nudp::send_to(udp_sock, pkt.Data, pkt.Size, remote_ip_addr, remote_port);

            ntimer::delay(100);
            ngpio::write_digital(gAppState.m_poweroff_pin, 0);  // Switch off supply
        }

    }  // namespace napp
}  // namespace ncore
