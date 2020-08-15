#ifndef __SMARTLIGHT_COMMANDS_H_INCLUDED__
#define __SMARTLIGHT_COMMANDS_H_INCLUDED__

#include "smartlight_types.h"

void smartlight_commands_init(
		smartlight_setup* a_setup
		);
void smartlight_commands_update();

int smartlight_commands_at();
typedef enum{
	_smartlight_commands_stationmode_station =1
	, _smartlight_commands_stationmode_softap = 2
	, _smartlight_commands_stationmode_full =3
}smartlight_commands_stationmode;
int smartlight_commands_setStationMode(
		smartlight_commands_stationmode mode
		);
int smartlight_commands_setWiFiPoint(
		char* name
		, int name_size
		, char* pass
		, int pass_size
		);
int smartlight_commands_setConnectionMode(
		int a_mode
		);
int smartlight_commands_startConnect(
		char*type
		,int type_size
		,char*ip
		,int ip_size
		,int port_dest
		,int port_src
		);
int smartlight_commands_sendData(uint8_t*a_pmsg,uint16_t a_size);

#endif //__SMARTLIGHT_COMMANDS_H_INCLUDED__
