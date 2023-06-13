#include "display.h"



const unsigned char battery_frame[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xf0, 0x00, 0xff, 0xff, 0xff, 0xf8, 0x00, 0xc0, 0x00, 0x00, 
	0x08, 0x00, 0x80, 0x00, 0x00, 0x0c, 0x00, 0x80, 0x00, 0x00, 0x0c, 0x00, 0x80, 0x00, 0x00, 0x0c, 
	0x00, 0x80, 0x00, 0x00, 0x0e, 0x00, 0x80, 0x00, 0x00, 0x0e, 0x00, 0x80, 0x00, 0x00, 0x0e, 0x00, 
	0x80, 0x00, 0x00, 0x0e, 0x00, 0x80, 0x00, 0x00, 0x0e, 0x00, 0x80, 0x00, 0x00, 0x0c, 0x00, 0x80, 
	0x00, 0x00, 0x0c, 0x00, 0x80, 0x00, 0x00, 0x0c, 0x00, 0xc0, 0x00, 0x00, 0x08, 0x00, 0xe0, 0x00, 
	0x00, 0x18, 0x00, 0x3f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
}; 


const unsigned char battery_chargin_frame[]  = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xf0, 0x00, 0xff, 0xff, 0xff, 0xf8, 0x00, 0xc0, 0x00, 0x00, 
	0x08, 0x00, 0x80, 0x00, 0x00, 0x0c, 0x00, 0x80, 0x06, 0x00, 0x0c, 0x00, 0x80, 0x07, 0xc0, 0x0c, 
	0x00, 0x80, 0x07, 0xf8, 0x0e, 0x00, 0x8f, 0x87, 0xff, 0x0e, 0x00, 0x81, 0xff, 0xff, 0x8e, 0x00, 
	0x80, 0x7f, 0xff, 0x8e, 0x00, 0x80, 0x1f, 0xc7, 0x8e, 0x00, 0x80, 0x03, 0x80, 0x0c, 0x00, 0x80, 
	0x00, 0x80, 0x0c, 0x00, 0x80, 0x00, 0x00, 0x0c, 0x00, 0xc0, 0x00, 0x00, 0x08, 0x00, 0xe0, 0x00, 
	0x00, 0x18, 0x00, 0x3f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};




HAL_StatusTypeDef display_init(void){
	
	if(HAL_I2C_IsDeviceReady(&I2C_HANDLER,SSD1306_I2C_ADDR,50,HAL_MAX_DELAY)==HAL_OK){
	SSD1306_Init (); // initialize the display
  SSD1306_Clear();
		
	SSD1306_GotoXY(25, 30); 
	SSD1306_Puts ("Welcome", &Font_11x18, 1); 
	SSD1306_UpdateScreen();	

	return HAL_OK;
	}
	return HAL_ERROR;
}

void set_bms_soc(uint8_t soc, charging_status_t mode){
	
  //clearing space
	SSD1306_DrawFilledRectangle(CHARGING_LOGO_X,CHARGING_LOGO_Y,LOGO_X_PIXEL,LOGO_Y_PIXEL,0);

	switch((int)mode){
		
		case operating :
		SSD1306_DrawBitmap(CHARGING_LOGO_X,CHARGING_LOGO_Y, battery_frame, LOGO_X_PIXEL, LOGO_Y_PIXEL, 1);

	  if(soc > 80) {
			
		// between 4.2 volt to 4 
		SSD1306_DrawLine(FIRST_BLOCK,11,FIRST_BLOCK,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+1,11,FIRST_BLOCK+1,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+2,11,FIRST_BLOCK+2,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+3,11,FIRST_BLOCK+3,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+4,11,FIRST_BLOCK+4,20,1);
	
		SSD1306_DrawLine(SECOND_BLOCK,11,SECOND_BLOCK,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+1,11,SECOND_BLOCK+1,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+2,11,SECOND_BLOCK+2,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+3,11,SECOND_BLOCK+3,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+4,11,SECOND_BLOCK+4,20,1);
		
		SSD1306_DrawLine(THIRD_BLOCK,11,THIRD_BLOCK,20,1);
		SSD1306_DrawLine(THIRD_BLOCK+1,11,THIRD_BLOCK+1,20,1);
		SSD1306_DrawLine(THIRD_BLOCK+2,11,THIRD_BLOCK+2,20,1);
		SSD1306_DrawLine(THIRD_BLOCK+3,11,THIRD_BLOCK+3,20,1);
		SSD1306_DrawLine(THIRD_BLOCK+4,11,THIRD_BLOCK+4,20,1);
		
		SSD1306_DrawLine(FOURTHD_BLOCK,11,FOURTHD_BLOCK,20,1);
		SSD1306_DrawLine(FOURTHD_BLOCK+1,11,FOURTHD_BLOCK+1,20,1);
		SSD1306_DrawLine(FOURTHD_BLOCK+2,11,FOURTHD_BLOCK+2,20,1);
		SSD1306_DrawLine(FOURTHD_BLOCK+3,11,FOURTHD_BLOCK+3,20,1);
		SSD1306_DrawLine(FOURTHD_BLOCK+4,11,FOURTHD_BLOCK+4,20,1);
		
	}else if(soc > 50){
		
		// between 4 volt to 3.7
		SSD1306_DrawLine(FIRST_BLOCK,11,FIRST_BLOCK,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+1,11,FIRST_BLOCK+1,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+2,11,FIRST_BLOCK+2,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+3,11,FIRST_BLOCK+3,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+4,11,FIRST_BLOCK+4,20,1);
	
		SSD1306_DrawLine(SECOND_BLOCK,11,SECOND_BLOCK,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+1,11,SECOND_BLOCK+1,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+2,11,SECOND_BLOCK+2,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+3,11,SECOND_BLOCK+3,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+4,11,SECOND_BLOCK+4,20,1);
		
		SSD1306_DrawLine(THIRD_BLOCK,11,THIRD_BLOCK,20,1);
		SSD1306_DrawLine(THIRD_BLOCK+1,11,THIRD_BLOCK+1,20,1);
		SSD1306_DrawLine(THIRD_BLOCK+2,11,THIRD_BLOCK+2,20,1);
		SSD1306_DrawLine(THIRD_BLOCK+3,11,THIRD_BLOCK+3,20,1);
		SSD1306_DrawLine(THIRD_BLOCK+4,11,THIRD_BLOCK+4,20,1);
	}else if(soc > 30) {

		// between 3.7 volt to 3.5
		
		SSD1306_DrawLine(FIRST_BLOCK,11,FIRST_BLOCK,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+1,11,FIRST_BLOCK+1,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+2,11,FIRST_BLOCK+2,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+3,11,FIRST_BLOCK+3,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+4,11,FIRST_BLOCK+4,20,1);
	
		SSD1306_DrawLine(SECOND_BLOCK,11,SECOND_BLOCK,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+1,11,SECOND_BLOCK+1,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+2,11,SECOND_BLOCK+2,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+3,11,SECOND_BLOCK+3,20,1);
		SSD1306_DrawLine(SECOND_BLOCK+4,11,SECOND_BLOCK+4,20,1);
	}else if(soc > 10 ) {
		// between 3.5 volt to 3.3
		SSD1306_DrawLine(FIRST_BLOCK,11,FIRST_BLOCK,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+1,11,FIRST_BLOCK+1,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+2,11,FIRST_BLOCK+2,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+3,11,FIRST_BLOCK+3,20,1);
		SSD1306_DrawLine(FIRST_BLOCK+4,11,FIRST_BLOCK+4,20,1);
  }
	
		SSD1306_UpdateScreen();
	
		break; 
	
		case charging : 
			
		SSD1306_DrawBitmap(CHARGING_LOGO_X,CHARGING_LOGO_Y, battery_chargin_frame, LOGO_X_PIXEL, LOGO_Y_PIXEL, 1);
		SSD1306_UpdateScreen();
		
		break;
	}
	}

	
	
	void display_clear(void){
		SSD1306_Clear();		
	}
	
	
	
	void display_mode(display_mode_t mode){
		
		
		switch((int)mode){
			
		case display_sleep :
			SSD1306_OFF();
		break;
		
		case display_wake :
			SSD1306_ON();
		break; 
	
		case  display_working :
			SSD1306_DrawFilledRectangle(0,0,40,25,0);
			SSD1306_GotoXY(1, 8); 
			SSD1306_Puts ("on", &Font_11x18, 1); 
			SSD1306_UpdateScreen();
		break;
		
		case dispay_setting :
		SSD1306_DrawFilledRectangle(0,0,40,25,0);	
		SSD1306_GotoXY(1, 8); 
  	SSD1306_Puts ("off", &Font_11x18, 1); 
		SSD1306_UpdateScreen();	
			
		break; 
		
	}
}
	
			
			
			
			
			
	
	
	
	
void set_preasure(uint32_t data){
	
	// clear working area
	SSD1306_DrawFilledRectangle(WORKING_AREA_X,WORKING_AREA_Y,WORKING_WITH_X,WORKING_WITH_Y,0);
	char buff[10]={0};
	if(data <10) {
	sprintf(buff,"  %dmmHg",data);
	}else if(data < 100) {
		sprintf(buff," %dmmHg",data);
	}else{
		sprintf(buff,"%dmmHg",data);
	}
	SSD1306_GotoXY(25, 30); 
	SSD1306_Puts (buff, &Font_11x18, 1); 
	SSD1306_UpdateScreen();
	
}





