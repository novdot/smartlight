#include "smartlight_control.h"
#include "smartlight_display.h"

#define SMARTLIGHT_OPTIMAL_PHOTO 200

smartlight_setup* g_pSetup;

/*****************************************************************************/
void smartlight_control_init(smartlight_setup* a_setup)
{
	g_pSetup = a_setup;
	g_pSetup->mode = _smartlight_workmode_off;
	g_pSetup->sensors.nOptimalPhoto = SMARTLIGHT_OPTIMAL_PHOTO;

	smartlight_display_init(g_pSetup);
}

/*****************************************************************************/
void smartlight_control_setOptimalPhoto(uint16_t photo)
{
	g_pSetup->sensors.nOptimalPhoto = photo;
}
/*****************************************************************************/
void smartlight_control_setWorkmode(smartlight_workmode mode)
{
	g_pSetup->mode = mode;
}
/*****************************************************************************/
#define DELTA 30
#define MULTI_DOWN (100 - DELTA)/100
#define MULTI_UP (100 + DELTA)/100

void smartlight_control_update()
{
	//update sensors data
	g_pSetup->sensors.sensors_handler();

	//check current mode
	switch(g_pSetup->mode) {
	case _smartlight_workmode_off:
		break;

	case _smartlight_workmode_handle:
		//роверим верхний предел
		if( (g_pSetup->sensors.nPotentiometr*16) >= 0xFFFF) goto display;
		g_pSetup->led.led_handler(g_pSetup->sensors.nPotentiometr*16);
		break;

	case _smartlight_workmode_auto:
		//потенциометр может менять предпочитаемую освещенность
		g_pSetup->sensors.nOptimalPhoto = g_pSetup->sensors.nPotentiometr/8;
		//сравниваем предпочитаемую с текущей и добавляем или убавляем свет
		if( (g_pSetup->sensors.nPhoto) > (g_pSetup->sensors.nOptimalPhoto*MULTI_UP) ){
			//проверка на возможность понижения
			if(g_pSetup->led.nLed==0) goto display;
			//нужно понижать
			g_pSetup->led.nLed *= 0.9;
			g_pSetup->led.led_handler(g_pSetup->led.nLed);
		} else if( (g_pSetup->sensors.nPhoto) < (g_pSetup->sensors.nOptimalPhoto*MULTI_DOWN) ){
			//проверка на возможность повышения
			if(g_pSetup->led.nLed >= 0xFFFF) goto display;
			//нужно повышать
			if( (g_pSetup->led.nLed*1.1) == g_pSetup->led.nLed ) g_pSetup->led.nLed += 10;
			else g_pSetup->led.nLed *= 1.1;
			g_pSetup->led.led_handler(g_pSetup->led.nLed);
		} else {
			//нажодимся в пределах +- 10% - ничего не меняем
		}
		break;
	}
display:
	//update display
	smartlight_display_update();
}
/*****************************************************************************/
void smartlight_control_SwitchWorkmode()
{
	g_pSetup->mode++;
	//Защита от выхода из диапазона - зацикливание.
	if(g_pSetup->mode>=_smartlight_workmode_count) {
		g_pSetup->mode = 0;
		g_pSetup->led.led_off();
	} else if(g_pSetup->mode==1)
		g_pSetup->led.led_on();
}


