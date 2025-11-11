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
    };
    state_app_t gAppState;

    ngpio::input_pin_t  switch_pin(13);    // GPIO pin connected to switch
    ngpio::output_pin_t poweroff_pin(16);  // GPIO pin connected to end line
    ngpio::analog_pin_t battery_pin(A0);   // GPIO pin connected to battery measurement
}  // namespace ncore

namespace ncore
{
    namespace napp
    {
        u64 gStartTimeMs;
        // This is where you would set up GPIO pins and other hardware before setup() is called
        void presetup()
        {
            // Initialize poweroff pin and set HIGH
            poweroff_pin.setup();
            poweroff_pin.set_high();
            // Initialize switch pin
            switch_pin.setup();
            // Note: Battery measurement pin is A0, and this is the only analog pin, so no need to set it up

            // Record start time
            gStartTimeMs = ntimer::millis();
        }

        void setup(state_t* state)
        {
            nwifi::init_state(state, true);
            nudp::init_state(state);

            // wait for 5 seconds maximum for WiFi connection
            nwifi::connect(state);
            u64 start_time = ntimer::millis();
            while (!nwifi::connected(state))
            {
                if (ntimer::millis() - start_time < 5000)
                {
                    ntimer::delay(10);
                }
                else
                {
                    nwifi::disconnect(state);
                    nserial::println("Retrying WiFi connection in normal mode...");
                    nwifi::connect(state, true);
                    break;
                }
            }

            start_time = ntimer::millis();
            while (!nwifi::connected(state))
            {
                if (ntimer::millis() - start_time < 5000)
                {
                    ntimer::delay(10);
                    nserial::print(".");
                }
                else
                {
                    nserial::println("WiFi connection failed, turning OFF device!");
                    poweroff_pin.set_low();
                    return;
                }
            }

            nserial::println("WiFi connected");
            nudp::open(state, state->ServerUdpPort);
        }

        void tick(state_t* state)
        {
            const s8  switch_state  = switch_pin.is_high() ? 1 : 0;      // Read switch state
            const s32 battery_level = (battery_pin.read() * 42) / 1023;  // Percentage (0-100 %)
            const s32 RSSI          = nwifi::get_RSSI(state);            // WiFi signal strength
            const u64 boottime      = ntimer::millis() - gStartTimeMs;   // Time since boot until we send the data

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
            ntimer::delay(100);  // Give some time for the UDP packet to be sent

            poweroff_pin.set_low();
            nserial::println("UDP packet has been sent, turning OFF device!");
            ntimer::delay(5000);  // Delay for 5 seconds
        }

    }  // namespace napp
}  // namespace ncore
