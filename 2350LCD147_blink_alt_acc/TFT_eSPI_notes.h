/*
User_Setup.h for tft_eSPI library and RP2350-LCD-1.47-A
// User defined information reported by "Read_User_Setup" test & diagnostics example
#define USER_SETUP_INFO "User_Setup rp2350147LCD"

//rp2350 waveshare lcd1.47

#define ST7789_2_DRIVER    // Minimal configuration option, define additional parameters below for this display

#define TFT_WIDTH  172 // ST7789 172 x 320
#define TFT_HEIGHT 320 // ST7789 240 x 320

#define CGRAM_OFFSET

#define TFT_SCLK 18
#define TFT_MOSI 19
//#define TFT_MISO 4
#define TFT_DC   16  // Data Command control pin
#define TFT_RST  20  // Reset pin (could connect to RST pin)
#define TFT_CS   17  // Chip select control pin
#define TFT_BL   21  // Reset pin (could connect to RST pin)
#define TFT_BACKLIGHT_ON HIGH  // Level to turn ON back-light (HIGH or LOW)
//#define TOUCH_CS -1    // Chip select pin (T_CS) of touch screen

 //#define TFT_INVERSION_ON
 //#define TFT_INVERSION_OFF
 //#define TFT_SPI_PORT 0 // Set to 0 if SPI0 pins are used, or 1 if spi1 pins used

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.

#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

#define SMOOTH_FONT

#define SPI_FREQUENCY       27000000

#define SPI_READ_FREQUENCY  20000000

#define SPI_TOUCH_FREQUENCY  2500000

#define RP2040_PIO_SPI // Leave commented out to use standard RP2040 SPI port interface

*/