// Siehe OneMsTaskTimer.h: Stub für TM4C1294, wird nicht benutzt
// (autoVCOM=false bei LCD_SharpBoosterPack_SPI in main.cpp).

#include "OneMsTaskTimer.h"

namespace OneMsTaskTimer {

void add(OneMsTaskTimer_t * task) { (void)task; }
void remove(OneMsTaskTimer_t * task) { (void)task; }
void start() {}
void start(uint32_t timer_index) { (void)timer_index; }
void stop() {}
void set_timer_index(uint32_t timer_index) { (void)timer_index; }
void _ticHandler() {}

}
