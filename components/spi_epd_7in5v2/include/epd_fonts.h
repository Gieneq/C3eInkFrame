#pragma once

#define MAX_HEIGHT_FONT         41
#define MAX_WIDTH_FONT          32
#define OFFSET_BITMAP           

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

//ASCII
typedef struct epd_font_t
{    
  const uint8_t *table;
  uint16_t width;
  uint16_t height;
  
} epd_font_t;

epd_font_t* epd_fonts_get_font(const uint8_t size);

extern epd_font_t font_8;
extern epd_font_t font_12;
extern epd_font_t font_16;
extern epd_font_t font_20;
extern epd_font_t font_24;

#ifdef __cplusplus
}
#endif
  
