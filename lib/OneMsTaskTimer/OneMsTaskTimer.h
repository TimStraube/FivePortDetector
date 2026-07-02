// Projekt-lokaler Ersatz für die framework-eigene OneMsTaskTimer-Lib.
// Die Original-Lib hat keinen Zweig für __TM4C1294NCPDT__ (nur MSP430,
// CC3200, TM4C123) -> Compile-Fehler. LCD_SharpBoosterPack_SPI braucht
// diesen Header nur für den optionalen Auto-VCOM-Toggle, den wir wegen
// des fehlenden TM4C1294-Zweigs abgeschaltet lassen (autoVCOM=false in
// main.cpp). Deshalb genügt hier ein Stub, der nie tatsächlich einen
// Timer startet (und damit auch nicht mit TIMER0 aus main.cpp kollidiert).

#ifndef OneMsTaskTimer_h
#define OneMsTaskTimer_h

#include <stdint.h>

typedef struct OneMsTaskTimer_t{
    uint32_t msecs;
    void (*func)();
    uint32_t count;
    OneMsTaskTimer_t * nextTask;
} OneMsTaskTimer_t;

namespace OneMsTaskTimer {
    void add(OneMsTaskTimer_t * task);
    void remove(OneMsTaskTimer_t * task);
    void start();
    void start(uint32_t timer_index);
    void stop();
    void set_timer_index(uint32_t timer_index);
    void _ticHandler();
}

#endif
