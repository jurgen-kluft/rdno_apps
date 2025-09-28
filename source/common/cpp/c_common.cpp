#include "rdno_core/c_config.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"
#include "rdno_core/c_app.h"

#include "common/c_network.secret.h"

namespace ncore
{
    namespace napp
    {
        struct str_2_id_t
        {
            const char* m_str;
            s16         m_len;
            s8          m_id;
            s8          m_type;
        };

        static str_2_id_t str2idmap_sorted[] = {
          /*Access Point Password*/ {"AP_PW", 5, nconfig::PARAM_ID_AP_PASSWORD, nconfig::PARAM_TYPE_STRING},
          /*Access Point SSID*/ {"AP_SSID", 7, nconfig::PARAM_ID_AP_SSID, nconfig::PARAM_TYPE_STRING},
          /*Battery (0-100%)*/ {"B", 1, nconfig::PARAM_ID_B, nconfig::PARAM_TYPE_U8},
          /*CO  */ {"CO", 2, nconfig::PARAM_ID_CO, nconfig::PARAM_TYPE_U16},
          /*CO2 */ {"CO2", 3, nconfig::PARAM_ID_CO2, nconfig::PARAM_TYPE_U16},
          /*Noise (db)*/ {"DB", 2, nconfig::PARAM_ID_N, nconfig::PARAM_TYPE_U8},
          /*Distance (mm)*/ {"D1", 2, nconfig::PARAM_ID_D1, nconfig::PARAM_TYPE_U16},
          /*Distance (mm)*/ {"D2", 2, nconfig::PARAM_ID_D2, nconfig::PARAM_TYPE_U16},
          /*Distance (mm)*/ {"D3", 2, nconfig::PARAM_ID_D3, nconfig::PARAM_TYPE_U16},
          /*Humidity*/ {"H", 2, nconfig::PARAM_ID_H, nconfig::PARAM_TYPE_U8},
          /*Pressure*/ {"P", 3, nconfig::PARAM_ID_P, nconfig::PARAM_TYPE_U16},
          /*Look at Point (x,y,z (mm))*/ {"LAP", 3, nconfig::PARAM_ID_LAP, nconfig::PARAM_TYPE_U64},
          /*Luminance (lux)*/ {"LUX", 3, nconfig::PARAM_ID_LUX, nconfig::PARAM_TYPE_U16},
          /*Open/Close */ {"OC", 2, nconfig::PARAM_ID_OC, nconfig::PARAM_TYPE_U8},
          /**/ {"P1", 2, nconfig::PARAM_ID_P1, nconfig::PARAM_TYPE_U8},
          /**/ {"P2", 2, nconfig::PARAM_ID_P2, nconfig::PARAM_TYPE_U8},
          /**/ {"P3", 2, nconfig::PARAM_ID_P3, nconfig::PARAM_TYPE_U8},
          /**/ {"PM1", 3, nconfig::PARAM_ID_PM1, nconfig::PARAM_TYPE_U16},
          /**/ {"PM2", 3, nconfig::PARAM_ID_PM2, nconfig::PARAM_TYPE_U16},
          /**/ {"PMA", 3, nconfig::PARAM_ID_PMA, nconfig::PARAM_TYPE_U16},
          /**/ {"PORT", 4, nconfig::PARAM_ID_REMOTE_PORT, nconfig::PARAM_TYPE_S16},
          /**/ {"POS", 3, nconfig::PARAM_ID_POS, nconfig::PARAM_TYPE_U64},
          /**/ {"PW", 2, nconfig::PARAM_ID_WIFI_PASSWORD, nconfig::PARAM_TYPE_STRING},
          /**/ {"RA1", 3, nconfig::PARAM_ID_RA1, nconfig::PARAM_TYPE_U64},
          /**/ {"RA2", 3, nconfig::PARAM_ID_RA2, nconfig::PARAM_TYPE_U64},
          /**/ {"RA3", 3, nconfig::PARAM_ID_RA3, nconfig::PARAM_TYPE_U64},
          /**/ {"S", 1, nconfig::PARAM_ID_S, nconfig::PARAM_TYPE_U16},
          /**/ {"SSID", 4, nconfig::PARAM_ID_WIFI_SSID, nconfig::PARAM_TYPE_STRING},
          /**/ {"SERVER", 6, nconfig::PARAM_ID_REMOTE_SERVER, nconfig::PARAM_TYPE_STRING},
          /**/ {"T", 1, nconfig::PARAM_ID_T, nconfig::PARAM_TYPE_S8},
          /**/ {"V", 1, nconfig::PARAM_ID_V, nconfig::PARAM_TYPE_U8},
          /**/ {"VOC", 3, nconfig::PARAM_ID_VOC, nconfig::PARAM_TYPE_U16},
          /**/ {"UV", 2, nconfig::PARAM_ID_UV, nconfig::PARAM_TYPE_U8},
        };

        s16 config_key_to_index(str_t const& str)
        {
            s32        i         = 0;
            const char firstchar = str.m_const[str.m_str];
            const s32  strlen    = str_len(str);
            const s32  n         = sizeof(str2idmap_sorted) / sizeof(str_2_id_t);
            bool       found     = false;
            while ((i < n) && !found)
            {
                const char* key = str2idmap_sorted[i].m_str;
                if (key[0] == firstchar)
                {
                    const s32 len = str2idmap_sorted[i].m_len;
                    if (strlen == len && (str_eq_n(str, key, len)))
                    {
                        return str2idmap_sorted[i].m_id;
                    }
                }
                i++;
            }

            return -1;
        }

        void setup_default_config(nconfig::config_t* config)
        {
            nconfig::reset(config);  // Reset the configuration to default values

            const s32 n = sizeof(str2idmap_sorted) / sizeof(str_2_id_t);
            for (s32 i = 0; i < n; i++)
            {
                const nconfig::param_id_t   id   = (nconfig::param_id_t)str2idmap_sorted[i].m_id;
                const nconfig::param_type_t type = (nconfig::param_type_t)str2idmap_sorted[i].m_type;
                switch (type)
                {
                    case nconfig::PARAM_TYPE_STRING: nconfig::set_string(config, id, str_const("")); break;
                    case nconfig::PARAM_TYPE_S8: nconfig::set_int8(config, id, 0); break;
                    case nconfig::PARAM_TYPE_U8: nconfig::set_uint8(config, id, 0); break;
                    case nconfig::PARAM_TYPE_S16: nconfig::set_int16(config, id, 0); break;
                    case nconfig::PARAM_TYPE_U16: nconfig::set_uint16(config, id, 0); break;
                    case nconfig::PARAM_TYPE_U64: nconfig::set_uint64(config, id, 0); break;
                }
            }

            const str_t ssid = str_const(WIFI_SSID);
            const str_t pass = str_const(WIFI_PASSWORD);
            nconfig::set_string(config, nconfig::PARAM_ID_WIFI_SSID, ssid);
            nconfig::set_string(config, nconfig::PARAM_ID_WIFI_PASSWORD, pass);
            char  ap_ssid_buffer[64];
            str_t ap_ssid = str_mutable(ap_ssid_buffer, sizeof(ap_ssid_buffer));
            str_append(ap_ssid, "device-");
            nsystem::get_unique_id(ap_ssid);
            nconfig::set_string(config, nconfig::PARAM_ID_AP_SSID, ap_ssid);
            const str_t ap_pass = str_const("esp32768");
            nconfig::set_string(config, nconfig::PARAM_ID_AP_PASSWORD, ap_pass);
            const str_t remote_server = str_const(SERVER_IP);
            nconfig::set_string(config, nconfig::PARAM_ID_REMOTE_SERVER, remote_server);
            const u16 remote_port = SERVER_PORT;
            nconfig::set_uint16(config, nconfig::PARAM_ID_REMOTE_PORT, remote_port);
        }
    }  // namespace napp
}  // namespace ncore
