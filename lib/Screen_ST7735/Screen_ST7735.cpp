#include "Screen_ST7735.h"

#define ST7735_WIDTH  128
#define ST7735_HEIGHT 128

// Col/Row Offset fuer 128x128 "greentab"-Variante (Crystalfontz CFAF128128B-0145T)
#define ST7735_COLSTART 2
#define ST7735_ROWSTART 1

#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_NORON   0x13
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_MADCTL  0x36
#define ST7735_COLMOD  0x3A
#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

Screen_ST7735::Screen_ST7735() {
#if defined(__LM4F120H5QR__) || defined(__MSP430F5529__) || defined(__TM4C123GH6PM__) || defined(__TM4C1294NCPDT__) || defined(__TM4C1294XNCZAD__)
    _pinReset          = 17;
    _pinDataCommand    = 31;
    _pinChipSelect     = 13;
    _pinBacklight      = NULL;
#else
#error Platform not supported
#endif
}

Screen_ST7735::Screen_ST7735(uint8_t resetPin, uint8_t dataCommandPin, uint8_t chipSelectPin, uint8_t backlightPin)
{
    _pinReset = resetPin;
    _pinDataCommand = dataCommandPin;
    _pinChipSelect = chipSelectPin;
    _pinBacklight = backlightPin;
}

void Screen_ST7735::begin()
{
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);

    if (_pinReset != NULL) pinMode(_pinReset, OUTPUT);
    if (_pinBacklight != NULL) pinMode(_pinBacklight, OUTPUT);
    pinMode(_pinDataCommand, OUTPUT);
    pinMode(_pinChipSelect, OUTPUT);

    if (_pinBacklight != NULL) digitalWrite(_pinBacklight, HIGH);

    if (_pinReset != NULL) {
        digitalWrite(_pinReset, HIGH);
        delay(50);
        digitalWrite(_pinReset, LOW);
        delay(50);
        digitalWrite(_pinReset, HIGH);
        delay(150);
    }

    _writeCommand(ST7735_SWRESET);
    delay(150);
    _writeCommand(ST7735_SLPOUT);
    delay(200);

    _writeCommand(ST7735_FRMCTR1);
    _writeData(0x01); _writeData(0x2C); _writeData(0x2D);
    _writeCommand(ST7735_FRMCTR2);
    _writeData(0x01); _writeData(0x2C); _writeData(0x2D);
    _writeCommand(ST7735_FRMCTR3);
    _writeData(0x01); _writeData(0x2C); _writeData(0x2D);
    _writeData(0x01); _writeData(0x2C); _writeData(0x2D);

    _writeCommand(ST7735_INVCTR);
    _writeData(0x07);

    _writeCommand(ST7735_PWCTR1);
    _writeData(0xA2); _writeData(0x02); _writeData(0x84);
    _writeCommand(ST7735_PWCTR2);
    _writeData(0xC5);
    _writeCommand(ST7735_PWCTR3);
    _writeData(0x0A); _writeData(0x00);
    _writeCommand(ST7735_PWCTR4);
    _writeData(0x8A); _writeData(0x2A);
    _writeCommand(ST7735_PWCTR5);
    _writeData(0x8A); _writeData(0xEE);

    _writeCommand(ST7735_VMCTR1);
    _writeData(0x0E);

    _writeCommand(ST7735_INVOFF);

    _writeCommand(ST7735_MADCTL);
    _writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB);

    _writeCommand(ST7735_COLMOD);
    _writeData(0x05);  // 16 bit/pixel

    _writeCommand(ST7735_GMCTRP1);
    _writeData(0x02); _writeData(0x1c); _writeData(0x07); _writeData(0x12);
    _writeData(0x37); _writeData(0x32); _writeData(0x29); _writeData(0x2d);
    _writeData(0x29); _writeData(0x25); _writeData(0x2B); _writeData(0x39);
    _writeData(0x00); _writeData(0x01); _writeData(0x03); _writeData(0x10);
    _writeCommand(ST7735_GMCTRN1);
    _writeData(0x03); _writeData(0x1d); _writeData(0x07); _writeData(0x06);
    _writeData(0x2E); _writeData(0x2C); _writeData(0x29); _writeData(0x2D);
    _writeData(0x2E); _writeData(0x2E); _writeData(0x37); _writeData(0x3F);
    _writeData(0x00); _writeData(0x00); _writeData(0x02); _writeData(0x10);

    _writeCommand(ST7735_NORON);
    delay(10);
    _writeCommand(ST7735_DISPON);
    delay(120);

    _screenWidth  = ST7735_WIDTH;
    _screenHeigth = ST7735_HEIGHT;
    _penSolid  = false;
    _fontSolid = true;
    _flagRead  = false;
    _touchTrim = 0;
    clear();
}

String Screen_ST7735::WhoAmI()
{
    return "EDUMKII ST7735 128x128";
}

void Screen_ST7735::invert(boolean flag)
{
    _writeCommand(flag ? ST7735_INVON : ST7735_INVOFF);
}

void Screen_ST7735::setBacklight(boolean flag)
{
    if (_pinBacklight != NULL) digitalWrite(_pinBacklight, flag);
}

void Screen_ST7735::setDisplay(boolean flag)
{
    if (_pinBacklight != NULL) setBacklight(flag);
}

void Screen_ST7735::setOrientation(uint8_t orientation)
{
    _orientation = orientation % 4;
    _writeCommand(ST7735_MADCTL);
    switch (_orientation) {
        case 0:
            _writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB);
            break;
        case 1:
            _writeData(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_RGB);
            break;
        case 2:
            _writeData(ST7735_MADCTL_RGB);
            break;
        case 3:
            _writeData(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_RGB);
            break;
    }
}

void Screen_ST7735::_fastFill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour)
{
    if (x1 > x2) _swap(x1, x2);
    if (y1 > y2) _swap(y1, y2);
    _setWindow(x1, y1, x2, y2);
    digitalWrite(_pinDataCommand, HIGH);
    digitalWrite(_pinChipSelect, LOW);
    uint8_t hi = highByte(colour);
    uint8_t lo = lowByte(colour);
    for (uint32_t t = (uint32_t)(y2 - y1 + 1) * (x2 - x1 + 1); t > 0; t--) {
        SPI.transfer(hi);
        SPI.transfer(lo);
    }
    digitalWrite(_pinChipSelect, HIGH);
}

void Screen_ST7735::_setPoint(uint16_t x1, uint16_t y1, uint16_t colour)
{
    if ((x1 < 0) || (x1 >= screenSizeX()) || (y1 < 0) || (y1 >= screenSizeY())) return;
    _setWindow(x1, y1, x1 + 1, y1 + 1);
    _writeData16(colour);
}

void Screen_ST7735::_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    x0 += ST7735_COLSTART;
    x1 += ST7735_COLSTART;
    y0 += ST7735_ROWSTART;
    y1 += ST7735_ROWSTART;

    _writeCommand(ST7735_CASET);
    _writeData16(x0);
    _writeData16(x1);
    _writeCommand(ST7735_RASET);
    _writeData16(y0);
    _writeData16(y1);
    _writeCommand(ST7735_RAMWR);
}

void Screen_ST7735::_writeCommand(uint8_t command8)
{
    digitalWrite(_pinDataCommand, LOW);
    digitalWrite(_pinChipSelect, LOW);
    SPI.transfer(command8);
    digitalWrite(_pinChipSelect, HIGH);
}

void Screen_ST7735::_writeData(uint8_t data8)
{
    digitalWrite(_pinDataCommand, HIGH);
    digitalWrite(_pinChipSelect, LOW);
    SPI.transfer(data8);
    digitalWrite(_pinChipSelect, HIGH);
}

void Screen_ST7735::_writeData16(uint16_t data16)
{
    digitalWrite(_pinDataCommand, HIGH);
    digitalWrite(_pinChipSelect, LOW);
    SPI.transfer(highByte(data16));
    SPI.transfer(lowByte(data16));
    digitalWrite(_pinChipSelect, HIGH);
}

void Screen_ST7735::_writeData88(uint8_t dataHigh8, uint8_t dataLow8)
{
    digitalWrite(_pinDataCommand, HIGH);
    digitalWrite(_pinChipSelect, LOW);
    SPI.transfer(dataHigh8);
    SPI.transfer(dataLow8);
    digitalWrite(_pinChipSelect, HIGH);
}

void Screen_ST7735::_getRawTouch(uint16_t &x0, uint16_t &y0, uint16_t &z0)
{
    x0 = 0;
    y0 = 0;
    z0 = 0;
}
