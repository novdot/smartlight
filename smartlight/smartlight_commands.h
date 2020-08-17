#ifndef __SMARTLIGHT_COMMANDS_H_INCLUDED__
#define __SMARTLIGHT_COMMANDS_H_INCLUDED__

#include "smartlight_types.h"

void smartlight_commands_init(
		smartlight_setup* a_setup
		);
void smartlight_commands_update();

int smartlight_commands_sendData(uint8_t*a_pmsg,uint16_t a_size);

#endif //__SMARTLIGHT_COMMANDS_H_INCLUDED__
