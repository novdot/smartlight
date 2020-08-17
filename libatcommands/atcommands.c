#include "atcommands.h"

//flags
uint8_t g_bIsFailCnt = 0;
uint8_t g_bIsActiveAT = 0;
uint8_t g_bIsSetupStationMode = 0;
uint8_t g_bIsSetupWiFiPoint = 0;
uint8_t g_bIsConnectionMode = 0;
uint8_t g_bIsConnect = 0;

/*****************************************************************************/
int at_commands_init()
{
	g_bIsFailCnt = 0;
	g_bIsActiveAT = 0;
	g_bIsSetupStationMode = 0;
	g_bIsSetupWiFiPoint = 0;
	g_bIsConnectionMode = 0;
	g_bIsConnect = 0;
}
/*****************************************************************************/
int at_commands_ping(uint8_t* send_buffer,uint16_t* len)
{
	sprintf(send_buffer,"AT\r\n");
	*len = 4;
}
/*****************************************************************************/
int at_commands_setStationMode(
		uint8_t* send_buffer
		, uint16_t* len
		, at_commands_stationmode mode
		)
{
	sprintf(send_buffer,"AT+CWMODE=%d\r\n",a_mode);
	*len = 13;
}
/*****************************************************************************/
int at_commands_setWiFiPoint(
		uint8_t* send_buffer
		, uint16_t* len
		, char* name
		, int name_size
		, char* pass
		, int pass_size
		)
{
	sprintf(send_buffer,"AT+CWJAP=\"%s\",\"%s\"",name,pass);
	*len = 15 + name_size + pass_size;
}
/*****************************************************************************/
int at_commands_setConnectionMode(
		uint8_t* send_buffer
		, uint16_t* len
		, int a_mode
		)
{
	sprintf(send_buffer,"AT+CIPMUX=%d",a_mode);
	*len = 13;
}
/*****************************************************************************/
int at_commands_startConnect(
		uint8_t* send_buffer
		, uint16_t* len
		, char*type
		, int type_size
		, char*ip
		, int ip_size
		, int port_dest
		, int port_src
		)
{
	sprintf(send_buffer,"AT+CIPSTART=0,\"%s\",\"%s\",%d,%d,0",type,ip,port_dest,port_src);
	*len = 25 + type_size + ip_size;
}
/*****************************************************************************/
int at_commands_send(
		uint8_t* send_buffer
		, uint16_t* len
		, uint8_t* a_pmsg,
		uint16_t a_msg_size
		)
{
	sprintf(send_buffer,"AT+CIPSEND=0,%d",a_msg_size);
	sprintf(send_buffer,a_pmsg);
	*len = 16 + a_msg_size;
}
/*****************************************************************************/
int at_commands_is_ready_to_send()
{/*
	//check if transmitted in process
	if(g_psetup->commands.bTxReady == 0)
		return;

	//check if get answer for previous command
	if(g_psetup->commands.bAnswerRec == 0)
		return;

	//check if setups done
	if(g_status >= _command_setup_status_count)
		return;

	switch(g_status) {
	case _command_setup_status_at:
		smartlight_commands_at();
		break;
	case _command_setup_status_station_mode:
		smartlight_commands_setStationMode(_smartlight_commands_stationmode_station);
		break;
	case _command_setup_status_wifi_point:
		smartlight_commands_setWiFiPoint("dlink",5,"a5165601",8);
		break;
	case _command_setup_status_conn_mode:
		smartlight_commands_setConnectionMode(1);
		break;
	case _command_setup_status_do_conn:
		smartlight_commands_startConnect("UDP",3,"192.168.0.105",13,1556,1555);
		break;
	default:
		goto fail;
	}
	g_psetup->commands.bTxReady = 0;
	g_psetup->commands.bAnswerRec = 0;
	return;*/
	return 0;
}
/*****************************************************************************/
int at_commands_rec_parse(
		uint8_t* send_buffer
		, uint16_t len
		, uint8_t* a_pmsg
		, uint16_t* a_msg_size
		)
{

}
/*****************************************************************************/

