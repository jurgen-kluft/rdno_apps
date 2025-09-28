#ifndef __APP_COMMON_H__
#define __APP_COMMON_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
    #pragma once
#endif

#include "rdno_core/c_config.h"
#include "rdno_core/c_packet.h"

namespace ncore
{
    struct str_t;

    namespace nsensor_type
    {
        typedef u16  type_t;
        const type_t Unknown     = 0x00;
        const type_t Temperature = nconfig::PARAM_ID_T | (npacket::nfieldtype::TypeS8 << 8);     // T    (s8, °C)
        const type_t Humidity    = nconfig::PARAM_ID_H | (npacket::nfieldtype::TypeU8 << 8);     // H    (u8, %)
        const type_t Pressure    = nconfig::PARAM_ID_P | (npacket::nfieldtype::TypeU16 << 8);    // P    (u16, hPa)
        const type_t Light       = nconfig::PARAM_ID_LUX | (npacket::nfieldtype::TypeU16 << 8);  // LUX  (u16, lux)
        const type_t CO2         = nconfig::PARAM_ID_CO2 | (npacket::nfieldtype::TypeU16 << 8);  // CO2  (u16, ppm)
        const type_t VOC         = nconfig::PARAM_ID_VOC | (npacket::nfieldtype::TypeU16 << 8);  // VOC  (u16, ppm)
        const type_t PM1_0       = nconfig::PARAM_ID_PM1 | (npacket::nfieldtype::TypeU16 << 8);  // PM1  (u16, µg/m3)
        const type_t PM2_5       = nconfig::PARAM_ID_PM2 | (npacket::nfieldtype::TypeU16 << 8);  // PM2  (u16, µg/m3)
        const type_t PM10        = nconfig::PARAM_ID_PMA | (npacket::nfieldtype::TypeU16 << 8);  // PMA  (u16, µg/m3)
        const type_t Noise       = nconfig::PARAM_ID_N | (npacket::nfieldtype::TypeU8 << 8);     // DB   (u8, dB)
        const type_t UV          = nconfig::PARAM_ID_UV | (npacket::nfieldtype::TypeU8 << 8);    // UV   (u8, index)
        const type_t CO          = nconfig::PARAM_ID_CO | (npacket::nfieldtype::TypeU8 << 8);    // CO   (u16, ppm/10)
        const type_t Vibration   = nconfig::PARAM_ID_V | (npacket::nfieldtype::TypeU8 << 8);     // V    (u8, 0=none, 1=low, 2=medium, 3=high)
        const type_t State       = nconfig::PARAM_ID_S | (npacket::nfieldtype::TypeU16 << 8);    // S    (u16, state)
        const type_t Battery     = nconfig::PARAM_ID_B | (npacket::nfieldtype::TypeU8 << 8);     // B    (u8, battery level 0-100%)
        const type_t Presence1   = nconfig::PARAM_ID_P1 | (npacket::nfieldtype::TypeU8 << 8);    // P1   (u8, 0=none, 1=trigger up/down, 2=presence)
        const type_t Presence2   = nconfig::PARAM_ID_P2 | (npacket::nfieldtype::TypeU8 << 8);    // P2   (u8, 0=none, 1=trigger up/down, 2=presence)
        const type_t Presence3   = nconfig::PARAM_ID_P3 | (npacket::nfieldtype::TypeU8 << 8);    // P3   (u8, 0=none, 1=trigger up/down, 2=presence)
        const type_t Distance1   = nconfig::PARAM_ID_D1 | (npacket::nfieldtype::TypeU16 << 8);   // D1   (u16, cm)
        const type_t Distance2   = nconfig::PARAM_ID_D2 | (npacket::nfieldtype::TypeU16 << 8);   // D2   (u16, cm)
        const type_t Distance3   = nconfig::PARAM_ID_D3 | (npacket::nfieldtype::TypeU16 << 8);   // D3   (u16, cm)

        inline u8 type(const type_t type) { return type & 0xFF; }
        inline u8 param(const type_t type) { return (type >> 8) & 0xFF; }
    };  // namespace nsensor_type

    s16  key_to_index(str_t const& str);
    void setup_default_config(nconfig::config_t* config);

}  // namespace ncore

#endif
