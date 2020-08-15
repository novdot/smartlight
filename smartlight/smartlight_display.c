#include "smartlight_display.h"
#include "smartlight_control.h"
#include "ssd1306.h"

#define SMARTLIGHT_OLED_HEADER_FONT_SIZE_H 18
#define SMARTLIGHT_OLED_HEADER_FONT_SIZE_W 11
#define SMARTLIGHT_OLED_BODY_FONT_SIZE_H 10
#define SMARTLIGHT_OLED_BODY_FONT_SIZE_W 7
#define SMARTLIGHT_OLED_BODY_LINE_MAX 3
#define SMARTLIGHT_OLED_FOOTER_FONT_SIZE_H 8
#define SMARTLIGHT_OLED_FOOTER_FONT_SIZE_W 6

#define SMARTLIGHT_OLED_H 64
#define SMARTLIGHT_OLED_W 128

#define SMARTLIGHT_OLED_LINE_START 2
#define SMARTLIGHT_OLED_LINE_LENGHT_MAX SMARTLIGHT_OLED_W

smartlight_display_page g_CurrentPage;
smartlight_setup* g_pSetup;
/*****************************************************************************/
char* getWorkmodeNameByCode(smartlight_workmode a_mode)
{
	switch(a_mode){
	case _smartlight_workmode_off:
		return ("off");
	case _smartlight_workmode_handle:
		return ("handle");
	case _smartlight_workmode_auto:
		return ("auto");
	default:
		return ("???");
	}
}

/*****************************************************************************/
void smartlight_display_init(smartlight_setup* a_setup)
{
	g_pSetup = a_setup;
	g_CurrentPage = _smartlight_display_page_main;
	//init OLED
	ssd1306_Init();
	smartlight_display_update();
}

/*****************************************************************************/
void smartlight_display_draw_header()
{
	//Font_11x18
	ssd1306_SetCursor(SMARTLIGHT_OLED_LINE_START,0);
	char buf[SMARTLIGHT_OLED_LINE_LENGHT_MAX];
	switch(g_CurrentPage){
	case _smartlight_display_page_main:
		sprintf(buf,"SmartLight");
		break;
	default:
		break;
	}
	ssd1306_WriteString("SmartLight", Font_11x18, Black);

}
/*****************************************************************************/
void smartlight_display_draw_footer()
{
	char buf[SMARTLIGHT_OLED_LINE_LENGHT_MAX];
	//Font_6x8
	ssd1306_SetCursor(SMARTLIGHT_OLED_LINE_START, SMARTLIGHT_OLED_H - SMARTLIGHT_OLED_FOOTER_FONT_SIZE_H);
	switch(g_CurrentPage){
	case _smartlight_display_page_main:
		sprintf(buf,"press to switch mode");
		break;
	default:
		break;
	}
	ssd1306_WriteString(buf, Font_6x8, Black);
}
/*****************************************************************************/
#define LABEL_SIZE (10*SMARTLIGHT_OLED_BODY_FONT_SIZE_W)
#define VALUE_SIZE (SMARTLIGHT_OLED_LINE_LENGHT_MAX - SMARTLIGHT_OLED_LINE_START - LABEL_SIZE)
#if( SMARTLIGHT_OLED_LINE_LENGHT_MAX<=(SMARTLIGHT_OLED_LINE_START+LABEL_SIZE) )
#error "Wrong values! LABEL_SIZE>=SMARTLIGHT_OLED_LINE_LENGHT_MAX ! check this defines!"
#endif

#define DRAW_BODY_LABEL(indx,text) \
		ssd1306_SetCursor(SMARTLIGHT_OLED_LINE_START,SMARTLIGHT_OLED_HEADER_FONT_SIZE_H+indx*SMARTLIGHT_OLED_BODY_FONT_SIZE_H);\
		ssd1306_WriteString(text, Font_7x10, Black);

#define DRAW_BODY_VALUE(indx,text) \
		ssd1306_SetCursor(SMARTLIGHT_OLED_LINE_START+LABEL_SIZE,SMARTLIGHT_OLED_HEADER_FONT_SIZE_H+indx*SMARTLIGHT_OLED_BODY_FONT_SIZE_H);\
		ssd1306_WriteString(text, Font_7x10, Black);

void smartlight_display_draw_boody()
{
	char buf[8];
	//Font_7x10 SMARTLIGHT_OLED_BODY_LINE_MAX - lines maximum
	switch(g_CurrentPage){
	case _smartlight_display_page_main:
		DRAW_BODY_LABEL(0,"workmode");
		DRAW_BODY_VALUE(0,getWorkmodeNameByCode(g_pSetup->mode));
		DRAW_BODY_LABEL(1,"current");
		sprintf(buf,"%d",g_pSetup->sensors.nPhoto);
		DRAW_BODY_VALUE(1,buf);
		DRAW_BODY_LABEL(2,"prefer");
		sprintf(buf,"%d",g_pSetup->sensors.nOptimalPhoto);
		DRAW_BODY_VALUE(2,buf);
		break;

	default:
		break;
	}
}
/*****************************************************************************/
void smartlight_display_update()
{
	//print info to OLED
    ssd1306_Fill(White);
	smartlight_display_draw_header();
	smartlight_display_draw_boody();
	smartlight_display_draw_footer();

    ssd1306_UpdateScreen();
}
