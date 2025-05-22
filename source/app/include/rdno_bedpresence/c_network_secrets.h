#ifndef __APP_WIFI_PASSWORD_H__
#define __APP_WIFI_PASSWORD_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
    #pragma once
#endif

#include "rdno_wifi/c_wifi.h"

namespace ncore
{
    const char* WIFI_SSID     = "OBNOSIS8";       // WiFi SSID
    const char* WIFI_PASSWORD = "abcdefghijkl8";  // WiFi password

    const IPAddress_t SERVER_IP   = {10, 0, 0, 100};  // Server IP address
    const u16         SERVER_PORT = 80;               // Server port number
}  // namespace ncore

#endif
