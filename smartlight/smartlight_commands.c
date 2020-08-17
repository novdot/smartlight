#include "smartlight_commands.h"
#include "atcommands.h"

smartlight_setup* g_psetup;

#define MAX_BUFFER_LENGHT 64

uint8_t rec_buffer[MAX_BUFFER_LENGHT];
uint8_t send_buffer[MAX_BUFFER_LENGHT];

uint8_t g_bIsConnect = 0;

/*****************************************************************************/
void smartlight_commands_init(smartlight_setup* a_setup)
{
	g_psetup = a_setup;
	g_bIsConnect = 0;
}
/*****************************************************************************/
void commands_send()
{
	uint16_t send_size = 0;

	uint8_t packet_buffer[MAX_BUFFER_LENGHT];
	uint16_t packet_size = 0;

	//проверка если интерфейс проинициализирован
	if(at_commands_is_ready_to_send()==0) return;

	//заполняем пакет информацией

	//отправляем текущее сосотояния регистров
	at_commands_send(send_buffer,&send_size,packet_buffer,packet_size);
	g_psetup->commands.commands_send(send_buffer,send_size);
fail:
	return;
}
/*****************************************************************************/

void commands_recieve()
{
	uint8_t packet_buffer[MAX_BUFFER_LENGHT];
	uint16_t packet_size = 0;
	//распарсили то что приняли
	at_commands_rec_parse(rec_buffer,MAX_BUFFER_LENGHT,packet_buffer,&packet_size);

	//выполним команду

	//очистили буффер
	memcpy(rec_buffer,0,MAX_BUFFER_LENGHT);

	return;

}
/*****************************************************************************/
void smartlight_commands_update()
{
	g_psetup->commands.commands_read(rec_buffer,MAX_BUFFER_LENGHT);
	commands_send();
	commands_recieve();
}


