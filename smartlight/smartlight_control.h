#ifndef __SMARTLIGHT_CONTROL_H_INCLUDED__
#define __SMARTLIGHT_CONTROL_H_INCLUDED__

#include "smartlight_types.h"

void smartlight_control_init(smartlight_setup* a_setup);
void smartlight_control_setOptimalPhoto(uint16_t photo);
uint16_t smartlight_control_getOptimalPhoto();
void smartlight_control_setWorkmode(smartlight_workmode mode);
smartlight_workmode smartlight_control_getWorkmode();
void smartlight_control_update();
void smartlight_control_SwitchWorkmode();

#endif // __SMARTLIGHT_CONTROL_H_INCLUDED__
