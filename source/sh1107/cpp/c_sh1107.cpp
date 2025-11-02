#include "sh1107/c_sh1107.h"

#include "rdno_core/c_malloc.h"
#include "rdno_core/c_gpio.h"
#include "rdno_core/c_timer.h"
#include "rdno_core/c_serial.h"
#include "rdno_core/c_packet.h"
#include "rdno_core/c_str.h"
#include "rdno_core/c_system.h"
#include "rdno_core/c_wire.h"

#include "rdno_wifi/c_tcp.h"
#include "rdno_wifi/c_node.h"

#include "common/c_common.h"

#include "Arduino.h"
#include "rdno_u8g2/U8x8lib.h"
#include "rdno_u8g2/U8g2lib.h"

namespace ncore
{
#define DISPLAY_SDA 7
#define DISPLAY_SCL 6
    U8G2_SH1107_128X128_1_HW_I2C gDisplay(U8G2_R1, U8X8_PIN_NONE, DISPLAY_SCL, DISPLAY_SDA);

    struct state_app_t
    {
    };
    state_app_t gAppState;

    static uint8_t *rotate90(uint8_t *buf)
    {
        static uint8_t rbuf[8];
        uint8_t        i, h;
        uint8_t       *p;
        uint8_t        j;
        for (i = 0; i < 8; i++)
            rbuf[i] = 0;
        for (i = 0; i < 8; i++)
        {
            h = buf[i];
            p = rbuf;
            *p >>= 1;
            *p |= (h & 128);
            h <<= 1;
            p++;
            *p >>= 1;
            *p |= (h & 128);
            h <<= 1;
            p++;
            *p >>= 1;
            *p |= (h & 128);
            h <<= 1;
            p++;
            *p >>= 1;
            *p |= (h & 128);
            h <<= 1;
            p++;
            *p >>= 1;
            *p |= (h & 128);
            h <<= 1;
            p++;
            *p >>= 1;
            *p |= (h & 128);
            h <<= 1;
            p++;
            *p >>= 1;
            *p |= (h & 128);
            h <<= 1;
            p++;
            *p >>= 1;
            *p |= (h & 128);
            h <<= 1;
            p++;
        }
        return rbuf;
    }

    void u8x8_draw_glyph_90(u8x8_t *u8x8, uint8_t x, uint8_t y, uint8_t encoding)
    {
        static uint8_t buf[8];
        u8x8_get_glyph_data(u8x8, encoding, buf, 0);
        u8x8_DrawTile(u8x8, x, y, 1, rotate90(buf));
    }

    void u8x8_draw_string_90(u8x8_t *u8x8, uint8_t x, uint8_t y, const char *s)
    {
        while (*s != '\0')
            u8x8_draw_glyph_90(u8x8, x, y++, *s++);
    }
}  // namespace ncore

namespace ncore
{
    namespace napp
    {
        void presetup()
        {
            // Initialize I2C bus
            nwire::begin(DISPLAY_SDA, DISPLAY_SCL);

            gDisplay.begin();
        }

        void setup(state_t *state) {}

        void tick(state_t *state)
        {
            gDisplay.firstPage();
            do
            {
                gDisplay.setFont(u8g2_font_ncenB14_tr);
                gDisplay.drawStr(0, 15, "Hello World!");
            } while (gDisplay.nextPage());
            delay(1000);
        }

    }  // namespace napp
}  // namespace ncore