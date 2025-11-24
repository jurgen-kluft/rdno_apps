#include "magnet/c_magnet.h"

#include "rdno_core/c_state.h"
#include "rdno_core/c_gpio.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_state.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_packet.h"
#include "rdno_espnow/c_espnow.h"

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
        // Gateway MAC Addresses (replace with your gateway's MAC address)
        const u8 gGatewayMacAddress1[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};
        const u8 gGatewayMacAddress2[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

        u64 gStartTimeMs;

        // This is where you would set up GPIO pins and other hardware before setup() is called
        void presetup(state_t* state)
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
            nespnow::init(true);

            nespnow::add_peer(gGatewayMacAddress1);
            nespnow::add_peer(gGatewayMacAddress2);
        }

        void tick(state_t* state)
        {
            s8  switch_state_cur      = switch_pin.is_high() ? 1 : 0;  // Read switch state
            s8  switch_state_prev     = 1 - switch_state_cur;          // Set previous state to opposite to ensure we enter the loop
            u16 switch_debounce_count = 0;

            // Read the switch state again before powering off, and if it is
            // still the same continue the power off sequence. Otherwise, handle
            // the switch state change by sending another ESPNOW packet.
            while (switch_state_cur != switch_state_prev && switch_debounce_count < 3)
            {
                const s32 battery_level = (battery_pin.read() * 42) / 1023;  // Percentage (0-100 %)
                const s32 RSSI          = nwifi::get_RSSI(state);            // WiFi signal strength
                const u64 boottime      = ntimer::millis() - gStartTimeMs;   // Time since boot until we send the data

                npacket::packet_t pkt;
                pkt.begin();
                nespnow::get_mac(&pkt.Data[npacket::MacOffset]);                         // Get device MAC address
                pkt.write_sensor(npacket::nsensorid::ID_SWITCH, (u16)switch_state_cur);  // Open/Close
                pkt.write_sensor(npacket::nsensorid::ID_BATTERY, (u16)battery_level);    // Battery level
                pkt.write_sensor(npacket::nsensorid::ID_RSSI, (u16)RSSI);                // WiFi signal strength
                pkt.write_sensor(npacket::nsensorid::ID_PERF1, (u16)boottime);           // Performance metric 1 (boot time in ms, max 65 seconds)
                pkt.write_sensor(npacket::nsensorid::ID_PERF2, switch_debounce_count);   // Performance metric 1 (debounce count)
                pkt.finalize();

                if (!nespnow::send_sync(nullptr, pkt.Data, pkt.Size, 200))  // Send to broadcast address with 200 ms timeout
                {
                    // Retry once if sending failed
                    nespnow::send_sync(nullptr, pkt.Data, pkt.Size, 200);
                }

                switch_state_prev = switch_state_cur;
                switch_state_cur  = switch_pin.is_high() ? 1 : 0;  // Read switch state again
                switch_debounce_count++;
            }

            poweroff_pin.set_low();
            nserial::println("ESPNOW packet has been sent, turning OFF device!");
            ntimer::delay(5000);  // Delay for 5 seconds
        }

    }  // namespace napp
}  // namespace ncore
