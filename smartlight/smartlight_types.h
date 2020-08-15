#ifndef __SMARTLIGHT_TYPES_H_INCLUDED__
#define __SMARTLIGHT_TYPES_H_INCLUDED__

#include "stdint.h"

#define SMARTLIGHT_VERSION "ver.1.0"

typedef enum{
	_smartlight_workmode_off = 0
	, _smartlight_workmode_handle
	, _smartlight_workmode_auto
	, _smartlight_workmode_count
}smartlight_workmode;

typedef void (*handlerSet_t)(uint32_t);
typedef void (*handler_t)(void);
typedef void (*handlerCommandsSend_t)(uint8_t*,uint16_t);
typedef void (*handlerCommandsRead_t)(uint8_t*,uint16_t);

typedef struct smartlight_led_controlDef{
	uint32_t nLed;
	handler_t led_on;
	handler_t led_off;
	handlerSet_t led_handler; //< set led power value
}smartlight_led_control;

typedef struct smartlight_com_controlDef{
	handlerCommandsSend_t commands_send;
	handlerCommandsRead_t commands_read;
	uint8_t bRxReady; //< when Rx done flag set to 1
	uint8_t bTxReady; //< if 1 ready for Tx
	uint8_t bAnswerRec; //<
}smartlight_com_control;

typedef struct smartlight_sensors_controlDef{
	uint16_t nPotentiometr; //< current potentiometr value
	uint16_t nPhoto; //< current lumen value
	uint16_t nOptimalPhoto; //< set optimal lumen value
	handler_t sensors_handler;
}smartlight_sensors_control;

typedef struct smartlight_setupDef{
	smartlight_workmode mode;
	smartlight_com_control commands;
	smartlight_led_control led;
	smartlight_sensors_control sensors;
}smartlight_setup;

#endif // __SMARTLIGHT_TYPES_H_INCLUDED__
