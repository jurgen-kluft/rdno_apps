#ifndef __APP_COMMON_H__
#define __APP_COMMON_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    struct str_t;

    namespace nvstore
    {
        struct config_t;
    }  // namespace nvstore

    s16 key_to_index(str_t const& str);
    void setup_default_config(nvstore::config_t* config);

}  // namespace ncore

#endif
