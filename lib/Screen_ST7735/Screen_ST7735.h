// Screen_ST7735 - Treiber fuer das Crystalfontz CFAF128128B-0145T (ST7735-Familie)
// auf dem BOOSTXL-EDUMKII Educational BoosterPack.
//
// Gleiche Schnittstelle wie Screen_HX8353E (nutzt dieselbe LCD_screen_font
// Basisklasse aus der EduBPMKII_Screen-Lib), nur mit ST7735-Registerwerten.
// Pins (CS=13/PN2, RST=17/PH3, DC=31/PL3, SPI-Modul 2/PD3+PD1) laut
// SLAU599B Table 2-7 und SPMU365 Table 2-1 (TI Educational BoosterPack MKII
// bzw. EK-TM4C1294XL User's Guide).

#if defined(ENERGIA)
#include "Energia.h"
#else
#error Board not supported
#endif

#ifndef SCREEN_ST7735_RELEASE
#define SCREEN_ST7735_RELEASE 100

#include "LCD_screen_font.h"
#include "SPI.h"

class Screen_ST7735 : public LCD_screen_font {
public:
    Screen_ST7735();
    Screen_ST7735(uint8_t resetPin, uint8_t dataCommandPin, uint8_t chipSelectPin, uint8_t backlightPin);
    void begin();
    String WhoAmI();
    void invert(boolean flag);
    void setBacklight(boolean flag);
    void setDisplay(boolean flag);
    void setOrientation(uint8_t orientation);
private:
    void _setPoint(uint16_t x1, uint16_t y1, uint16_t colour);
    void _setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void _writeCommand(uint8_t command8);
    void _writeData(uint8_t data8);
    void _writeData16(uint16_t data16);
    void _writeData88(uint8_t dataHigh8, uint8_t dataLow8);
    void _fastFill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour);
    void _getRawTouch(uint16_t &x, uint16_t &y, uint16_t &z);
    uint8_t _pinReset;
    uint8_t _pinDataCommand;
    uint8_t _pinChipSelect;
    uint8_t _pinBacklight;
};
#endif
