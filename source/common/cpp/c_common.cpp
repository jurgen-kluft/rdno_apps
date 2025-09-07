#include "rdno_core/c_nvstore.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"

#include "common/c_network.secret.h"

namespace ncore
{
    s16 key_to_index(str_t const& str)
    {
        s16 param_id = -1;
        switch (str_len(str))
        {
            case 4: param_id = str_eq_n(str, "ssid", 4, false) ? nvstore::PARAM_ID_SSID : -1; break;
            case 7: param_id = str_eq_n(str, "ap_ssid", 7, false) ? nvstore::PARAM_ID_AP_SSID : -1; break;
            case 8: param_id = str_eq_n(str, "password", 8, false) ? nvstore::PARAM_ID_PASSWORD : -1; break;
            case 11:
                if (str_eq_n(str, "ap_password", 11, false))
                    param_id = nvstore::PARAM_ID_AP_PASSWORD;
                else if (str_eq_n(str, "remote_port", 11, false))
                    param_id = nvstore::PARAM_ID_REMOTE_PORT;
                break;
            case 13: param_id = str_eq_n(str, "remote_server", 13, false) ? nvstore::PARAM_ID_REMOTE_SERVER : -1; break;
        }
        if (param_id < 0)
        {
            s32 value = 0;
            if (from_str(str, &value, 10) && value >= 0 && value < 256)
            {
                param_id = static_cast<s16>(value);
            }
        }
        return param_id;
    }

    void setup_default_config(nvstore::config_t* config)
    {
        nvstore::reset(config);  // Reset the configuration to default values
        const str_t ssid = str_const(WIFI_SSID);
        const str_t pass = str_const(WIFI_PASSWORD);
        nvstore::set_string(config, nvstore::PARAM_ID_SSID, ssid);
        nvstore::set_string(config, nvstore::PARAM_ID_PASSWORD, pass);
        char  ap_ssid_buffer[64];
        str_t ap_ssid = str_mutable(ap_ssid_buffer, sizeof(ap_ssid_buffer));
        str_append(ap_ssid, "HumanPresence-");
        nsystem::get_unique_id(ap_ssid);
        nvstore::set_string(config, nvstore::PARAM_ID_AP_SSID, ap_ssid);
        const str_t ap_pass = str_const("32768");
        nvstore::set_string(config, nvstore::PARAM_ID_AP_PASSWORD, ap_pass);
        const str_t remote_server = str_const(SERVER_IP);
        nvstore::set_string(config, nvstore::PARAM_ID_REMOTE_SERVER, remote_server);
        const s32 remote_port = SERVER_PORT;
        nvstore::set_int(config, nvstore::PARAM_ID_REMOTE_PORT, remote_port);
    }
}  // namespace ncore
