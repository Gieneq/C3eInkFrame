#include "epd_fonts.h"


epd_font_t* epd_fonts_get_font(const uint8_t size) {
    epd_font_t* font = &font_8;
    switch (size)
    {
    case 12:
        font = &font_12;
        break;

    case 16:
        font = &font_16;
        break;

    case 20:
        font = &font_20;
        break;
    
    case 24:
        font = &font_24;
        break;
    
    default:
        font = &font_8;
        break;
    }

    return font;
}