/*
  ******************************************************************************
  * @file 			( фаил ):   ST7735.h
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):
  ******************************************************************************
  
 */
 
 
#ifndef _ST7735_H
#define _ST7735_H


/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

// Обязательно нужен #include "main.h" 
// чтоб отдельно не подключать файлы связанные с МК и стандартными библиотеками

#include "main.h"
#include "fonts.h"

#include "stdlib.h"
#include "string.h"

#include "math.h"


//#######  SETUP  ##############################################################################################
		
		//==== выбераем через что будем отправлять через HAL или CMSIS(быстрее) ==================
		//-- нужное оставляем другое коментируем ( важно должно быть только один выбран )---------
		
			// указываем порт SPI для CMSIS ( быстро )-------
			// так как у разных МК разные регистры то в функциях корректируем под свой МК
			// на данный момент есть реализация на серию F1 F4 H7 для выбора серии в функциях
			//	void ST7735_SendCmd(uint8_t Cmd);
			//	void ST7735_SendData(uint8_t Data );
			//	void ST7735_SendDataMASS(uint8_t* buff, size_t buff_size);	
			// комментируем и раскомментируем то что нам нужно, также там же редактируем под свой МК если не работает
			#define 	ST7735_SPI_CMSIS 	SPI1
			//-----------------------------------------------
			
			// указываем порт SPI для HAL ( медлено )--------
			//#define 	ST7735_SPI_HAL 		hspi1
			//-----------------------------------------------
			
		//============================================================================
			
			// выбираем как выводить информацию через буфер кадра или попиксельно ( 1-буфер кадра, 0-попиксельный вывод ) -----
			// через буфер быстре если много информации обнавлять за один раз ( требует много оперативки для массива )
			// по пиксельно рисует онлайн буз буферра если информация обновляеться немного то выгодно испотзовать данный режим
			#define FRAME_BUFFER				0
			//-----------------------------------------------------------------------------------------------------------------
			
		//=== указываем порты ( если в кубе назвали их DC RES CS то тогда нечего указывать не нужно )
		#if defined (DC_GPIO_Port)
		#else
			#define DC_GPIO_Port	GPIOA
			#define DC_Pin			GPIO_PIN_11
		#endif
		
		#if defined (RES_GPIO_Port)
		#else
			#define RES_GPIO_Port   GPIOA
			#define RES_Pin			GPIO_PIN_12
		#endif
		
		//--  Cесли используем порт CS для выбора устройства тогда раскомментировать ------------
		// если у нас одно устройство лучше пин CS притянуть к земле( или на порту подать GND )
		
		#define CS_PORT
		
		//----------------------------------------------------------------------------------------
		#ifdef CS_PORT
			#if defined (CS_GPIO_Port)
			#else
				#define CS_GPIO_Port    GPIOA
				#define CS_Pin			GPIO_PIN_14
			#endif
		#endif
		
		//=============================================================================
		
		//=== выбрать нужный дисплей  ==============================================================
		//-- нужное оставляем другое коментируем ( важно должно быть только один выбран )---------
		
		//	#define  ST7735_IS_160X128_V1		// 1.8" 160 x 128 AliExpress/eBay  display, default orientatio		
		//	#define  ST7735_IS_160X128_V2		// 1.8" 160 x 128 WaveShare ST7735S-based display, default orientation		
		//	#define  ST7735_IS_128X128			// 1.44" 128 x 128 display, default orientation		
			#define  ST7735_IS_160X80			// 0.96" mini 160 x 80 display (it's unlikely you want the default orientation)
		
		//==========================================================================================
		
//##############################################################################################################

#ifdef ST7735_SPI_HAL
	extern SPI_HandleTypeDef ST7735_SPI_HAL;
#endif

extern uint16_t ST7735_Width, ST7735_Height;

extern uint16_t ST7735_X_Start;
extern uint16_t ST7735_Y_Start;

#define RGB565(r, g, b)         (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

#define PI 	3.14159265

//-- готовые цвета -----------------------
	#define   	ST7735_BLACK   	0x0000
	#define   	ST7735_BLUE    	0x001F
	#define   	ST7735_RED     	0xF800
	#define   	ST7735_GREEN   	0x07E0
	#define 	ST7735_CYAN    	0x07FF
	#define 	ST7735_MAGENTA 	0xF81F
	#define 	ST7735_YELLOW  	0xFFE0
	#define 	ST7735_WHITE   	0xFFFF
//----------------------------------------



// Битовые маски настройки цветности ST7735
#define ST7735_ColorMode_12bit  0x03
#define ST7735_ColorMode_16bit  0x05
#define ST7735_ColorMode_18bit  0x06
//----------------------------------------


#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_ML  0x10
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04
//----------------------------------------

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1
//---------------------------------------

#define DELAY 0x80

//---------------------------------------



//===========================================================================================
// 1.8" 160 x 128 AliExpress/eBay  display, default orientation

#ifdef ST7735_IS_160X128_V1
	#define ST7735_WIDTH  128
	#define ST7735_HEIGHT 160
	#define ST7735_XSTART 0
	#define ST7735_YSTART 0
	#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY)
#endif

//===========================================================================================

//===========================================================================================
// 1.8" 160 x 128 WaveShare ST7735S-based display, default orientation

#ifdef ST7735_IS_160X128_V2
	#define ST7735_WIDTH  128
	#define ST7735_HEIGHT 160
	#define ST7735_XSTART 2
	#define ST7735_YSTART 1
	#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB)
#endif

//===========================================================================================

//===========================================================================================
// 1.44" 128 x 128 display, default orientation

#ifdef ST7735_IS_128X128
	#define ST7735_WIDTH  128
	#define ST7735_HEIGHT 128
	#define ST7735_XSTART 2
	#define ST7735_YSTART 3
	#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR)
#endif

//===========================================================================================

//===========================================================================================
// 0.96" mini 160 x 80 display (it's unlikely you want the default orientation)

#ifdef ST7735_IS_160X80 
	#define ST7735_XSTART 26
	#define ST7735_YSTART 1
	#define ST7735_WIDTH  80
	#define ST7735_HEIGHT 160 
	#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR)
#endif

//===========================================================================================

/****************************/

//###########################################################################################
//###########################################################################################



void ST7735_Init(void);
void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);	
void ST7735_HardReset(void);
void ST7735_SleepModeEnter( void );
void ST7735_SleepModeExit( void );
void ST7735_ColorModeSet(uint8_t ColorMode);
void ST7735_MemAccessModeSet(uint8_t Rotation, uint8_t VertMirror, uint8_t HorizMirror, uint8_t IsBGR);
void ST7735_InversionMode(uint8_t Mode);
void ST7735_FillScreen(uint16_t color);
void ST7735_Clear(void);
void ST7735_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ST7735_SetBL(uint8_t Value);
void ST7735_DisplayPower(uint8_t On);
void ST7735_DrawRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7735_DrawRectangleFilled(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t fillcolor);
void ST7735_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7735_DrawLineWithAngle(int16_t x, int16_t y, uint16_t length, double angle_degrees, uint16_t color);
void ST7735_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void ST7735_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void ST7735_DrawPixel(int16_t x, int16_t y, uint16_t color);
void ST7735_DrawCircleFilled(int16_t x0, int16_t y0, int16_t radius, uint16_t fillcolor);
void ST7735_DrawCircle(int16_t x0, int16_t y0, int16_t radius, uint16_t color);
void ST7735_DrawEllipse(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color);
void ST7735_DrawEllipseFilled(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color);
void ST7735_DrawEllipseFilledWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color);
void ST7735_DrawEllipseWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color);
void ST7735_DrawChar(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, unsigned char ch);
void ST7735_DrawCharWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, unsigned char ch);
void ST7735_print(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, char *str);
void ST7735_printWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, char *str);
void ST7735_rotation( uint8_t rotation );
void ST7735_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);
void ST7735_DrawBitmapWithAngle(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color, double angle_degrees);
void ST7735_DrawCircleHelper(int16_t x0, int16_t y0, int16_t radius, int8_t quadrantMask, uint16_t color);
void ST7735_DrawFillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void ST7735_DrawFillRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color);
void ST7735_DrawRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color);
void ST7735_DrawArc(int16_t x0, int16_t y0, int16_t radius, int16_t startAngle, int16_t endAngle, uint16_t color, uint8_t thick);
void ST7735_DrawLineThick(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t thick);
void ST7735_DrawLineThickWithAngle(int16_t x, int16_t y, int16_t length, double angle_degrees, uint16_t color, uint8_t thick);


#if FRAME_BUFFER
	void ST7735_Update(void);
	void ST7735_ClearFrameBuffer(void);
#endif


/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif	/*	_ST7735_H */

/************************ (C) COPYRIGHT GKP *****END OF FILE****/
