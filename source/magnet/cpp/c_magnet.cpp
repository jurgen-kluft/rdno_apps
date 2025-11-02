#include "magnet/c_magnet.h"

#include "rdno_core/c_state.h"
#include "rdno_core/c_gpio.h"
#include "rdno_wifi/c_wifi.h"
#include "rdno_wifi/c_node.h"
#include "rdno_wifi/c_udp.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_state.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_state.h"
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
    state_app_t gAppState;
}  // namespace ncore

namespace ncore
{
    namespace napp
    {
        u64 gStartTimeMs;
        // This is where you would set up GPIO pins and other hardware before setup() is called
        void presetup()
        {
            // Record start time
            gStartTimeMs = ntimer::millis();

            // Initialize poweroff pin and set it high to keep power on
            ngpio::set_pin_as_output(gAppState.m_poweroff_pin);
            ngpio::write_digital(gAppState.m_poweroff_pin, true);
            // Initialize switch pin as input
            ngpio::set_pin_as_input(gAppState.m_switch_pin);
            // Note: Battery measurement pin is A0, and this is the only analog pin, so no need to set it up
        }

        void setup(state_t* state)
        {
            nwifi::init_state(state, false);
            nudp::init_state(state);

            // wait for 5 seconds maximum for WiFi connection
            nwifi::connect(state);
            const u64 start_time = ntimer::millis();
            while (!nwifi::connected(state))
            {
                if (ntimer::millis() - start_time < 5000)
                {
                    ntimer::delay(50);
                    nserial::print(".");
                }
                else
                {
                    nserial::println("WiFi connection failed!");
                    ngpio::write_digital(gAppState.m_poweroff_pin, 1);
                    return;
                }
            }

            nserial::println("WiFi connection failed!");

            nudp::open(state, state->ServerUdpPort);

            nserial::println("Setup done...");
        }

        void tick(state_t* state)
        {
            const s8  switch_state  = ngpio::read_digital(gAppState.m_switch_pin);  // Read switch state
            const s32 battery_level = (ngpio::read_analog(A0) * 42 / 1023);         // Percentage (0-100 %)
            const s32 RSSI          = nwifi::get_RSSI(state);                       // WiFi signal strength
            const u64 boottime      = ntimer::millis() - gStartTimeMs;              // Time since boot until we send the data

            npacket::packet_t pkt;
            pkt.begin(state->wifi->m_mac);                                         // Initialize packet with MAC address
            pkt.write_sensor(npacket::nsensorid::ID_SWITCH, (u16)switch_state);    // Open/Close
            pkt.write_sensor(npacket::nsensorid::ID_BATTERY, (u16)battery_level);  // Battery level
            pkt.write_sensor(npacket::nsensorid::ID_RSSI, (u16)RSSI);              // WiFi signal strength
            pkt.write_sensor(npacket::nsensorid::ID_PERF1, (u16)boottime);         // Performance metric 1 (boot time in ms)
            pkt.finalize();

            IPAddress_t server_ip;
            server_ip.from(state->ServerIP);

            nudp::send_to(state, pkt.Data, pkt.Size, server_ip, state->ServerUdpPort);

            // ngpio::write_digital(gAppState.m_poweroff_pin, 1);
            ntimer::delay(5000);  // Wait for 5 seconds before next reading
        }

    }  // namespace napp
}  // namespace ncore
