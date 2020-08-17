#ifndef __LIB_AT_COMMANDS_H_INCLUDED__
#define __LIB_AT_COMMANDS_H_INCLUDED__

typedef enum at_commands_stationmodeDef{
	_at_commands_stationmode_station =1
	, _at_commands_stationmode_softap = 2
	, _at_commands_stationmode_full =3
}at_commands_stationmode;

typedef enum at_commands_setup_statusDef{
	_at_commands_setup_status_at = 0
	, _at_commands_setup_status_station_mode = 1
	, _at_commands_setup_status_wifi_point = 2
	, _at_commands_setup_status_conn_mode = 3
	, _at_commands_setup_status_do_connect = 4
	, _at_commands_setup_status_connected = 5
	, _at_commands_setup_status_count //always last item!
}at_commands_setup_status;

int at_commands_init();
int at_commands_send(
		uint8_t* send_buffer
		, uint16_t* len
		, uint8_t*a_pmsg
		, uint16_t a_size
		);
int at_commands_is_ready_to_send();
int at_commands_rec_parse(
		uint8_t* send_buffer
		, uint16_t len
		, uint8_t* a_pmsg
		, uint16_t* a_msg_size
		);

#endif //__LIB_AT_COMMANDS_H_INCLUDED__
