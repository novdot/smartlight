#ifndef __SMARTLIGHT_DISPLAY_H_INCLUDED__
#define __SMARTLIGHT_DISPLAY_H_INCLUDED__

#include "smartlight_types.h"

typedef enum{
	_smartlight_display_page_main = 0
	, _smartlight_display_page_menu
	, _smartlight_display_page_setups
	, _smartlight_display_page_about
}smartlight_display_page;

void smartlight_display_init(smartlight_setup* a_setup);
void smartlight_display_update();

#endif // __SMARTLIGHT_DISPLAY_H_INCLUDED__
