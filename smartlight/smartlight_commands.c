#include "smartlight_commands.h"

smartlight_setup* g_psetup;
uint8_t rec_buffer;//[64];
uint8_t send_buffer[64];

//flags
uint8_t g_bIsFailCnt = 0;
uint8_t g_bIsActiveAT = 0;
uint8_t g_bIsSetupStationMode = 0;
uint8_t g_bIsSetupWiFiPoint = 0;
uint8_t g_bIsConnectionMode = 0;
uint8_t g_bIsConnect = 0;

/*****************************************************************************/
void smartlight_commands_init(smartlight_setup* a_setup)
{
	g_psetup = a_setup;

	g_bIsFailCnt = 0;
	g_bIsActiveAT = 0;
	g_bIsSetupStationMode = 0;
	g_bIsSetupWiFiPoint = 0;
	g_bIsConnectionMode = 0;
	g_bIsConnect = 0;
}
/*****************************************************************************/
void send_cmd(uint8_t* retbuf,int len)
{
	// CR+LF ASCII 0x0D 0x0A \r \n
	retbuf[len] = 0x0D;
	retbuf[len+1] = 0x0A;
	g_psetup->commands.bTxReady = 0;
	g_psetup->commands.commands_send(retbuf,len+2);
}
/*****************************************************************************/
int smartlight_commands_at()
{
	//send AT
	sprintf(send_buffer,"AT");
	send_cmd(send_buffer,2);
	//search answer
	//g_psetup->commands_read(&rec_buffer,4+4);
	//return strncmp(send_buffer,rec_buffer,4);
	return 0;
}

/*****************************************************************************/
int smartlight_commands_setStationMode(smartlight_commands_stationmode a_mode)
{
	sprintf(send_buffer,"AT+CWMODE=%d",a_mode);
	send_cmd(send_buffer,11);
	return 0;
}
/*****************************************************************************/
int smartlight_commands_setWiFiPoint(
		char* name
		, int name_size
		, char* pass
		, int pass_size
		)
{
	sprintf(send_buffer,"AT+CWJAP=\"%s\",\"%s\"",name,pass);
	send_cmd(send_buffer,13+name_size+pass_size);
	return 0;
}
/*****************************************************************************/
int smartlight_commands_setConnectionMode(int a_mode)
{
	sprintf(send_buffer,"AT+CIPMUX=%d",a_mode);
	send_cmd(send_buffer,11);
	return 0;
}

/*****************************************************************************/
int smartlight_commands_startConnect(
		char*type
		,int type_size
		,char*ip
		,int ip_size
		,int port_dest
		,int port_src
		)
{
	//sprintf(send_buffer,"AT+CIPSTART=0,\"UDP\",\"192.168.0.105\",1556,1555,0%x%x",0x0a,0x0b);
	sprintf(send_buffer,"AT+CIPSTART=0,\"%s\",\"%s\",%d,%d,0",type,ip,port_dest,port_src);
	send_cmd(send_buffer,23+type_size+ip_size);
	return 0;
}
/*****************************************************************************/
int smartlight_commands_sendData(uint8_t*a_pmsg,uint16_t a_size)
{
	sprintf(send_buffer,"AT+CIPSEND=0,%d",20);
	send_cmd(send_buffer,15);

	sprintf(send_buffer,a_pmsg);//"Hello from ESP8266"
	send_cmd(send_buffer,a_size);
	return 0;
}
/*****************************************************************************/
typedef enum command_setup_statusDef{
	_command_setup_status_at = 0
	, _command_setup_status_station_mode = 1
	, _command_setup_status_wifi_point = 2
	, _command_setup_status_conn_mode = 3
	, _command_setup_status_do_conn = 4
	, _command_setup_status_count //always last item!
}command_setup_status;
command_setup_status g_status = _command_setup_status_at;
void commands_send()
{
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
	return;
fail:
	return;
}
/*****************************************************************************/
#define STATUS_MAX 6
void commands_recieve()
{
	static uint8_t _status = 0;
	static uint8_t command_ok_code[STATUS_MAX] = {'\r','\n','O','K','\r','\n'};

	if(g_psetup->commands.bRxReady == 0)
		return;

	if(rec_buffer==command_ok_code[_status])
		_status++;
	else
		goto fail;

	if(_status >= STATUS_MAX)
		goto succsess;

whait:
	return;
succsess:
	g_psetup->commands.bAnswerRec = 1;
	if(g_status<_command_setup_status_count) g_status++;
fail:
	_status = 0;
	return;

}
/*****************************************************************************/
void smartlight_commands_update()
{
	//if(g_bIsFailCnt>=5) return;

	commands_send();
	g_psetup->commands.commands_read(&rec_buffer,1);
	commands_recieve();

	//if connected and setuped - send current data
}


