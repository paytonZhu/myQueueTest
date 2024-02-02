#include "sys.h"


//Ƕ��ʽ������
//�Ա���վ��http://chinanuedc.taobao.com



//LCD��Ҫ������
typedef struct  
{										    
	u16 width;			//LCD ���
	u16 height;			//LCD �߶�
	u16 id;				//LCD ID
	u8  dir;			//���������������ƣ�0��������1��������	
	u16	 wramcmd;		//��ʼдgramָ��
	u16  setxcmd;		//����x����ָ��
	u16  setycmd;		//����y����ָ��	 
}_lcd_dev; 	

//LCD����
extern _lcd_dev lcddev;	//����LCD��Ҫ����


#define RED  	  0xf800
#define GREEN	  0x07e0
#define BLUE 	  0x001f
#define WHITE	  0xffff
#define BLACK	  0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D   	//��ɫ0 3165 00110 001011 00101
#define GRAY1   0x8410      	//��ɫ1      00000 000000 00000
#define GRAY2   0x4208      	//��ɫ2  1111111111011111

/////////////////////////////////////�û�������///////////////////////////////////	 
//֧�ֺ��������ٶ����л���֧��8/16λģʽ�л�
#define USE_HORIZONTAL  	0	    //�����Ƿ�ʹ�ú��� 		0,��ʹ��.  1,ʹ��.
//ʹ��ģ��SPI��Ϊ����



//Һ�����ƿ���1�������궨��
#define	LCD_SDA_SET  	GPIO_SetBits(GPIOD,GPIO_Pin_6)
#define	LCD_SCL_SET  	GPIO_SetBits(GPIOD,GPIO_Pin_11) 

#define	LCD_CS_SET  	GPIO_SetBits(GPIOC,GPIO_Pin_6)   
#define LCD_RST_SET   GPIO_SetBits(GPIOE,GPIO_Pin_0);//RST�������Ϊ��

#define	LCD_RS_SET  	GPIO_SetBits(GPIOE,GPIO_Pin_4)   
#define	LCD_BLK_SET  	GPIO_SetBits(GPIOE,GPIO_Pin_5)   

//Һ�����ƿ���0�������궨��

#define	LCD_SDA_CLR  	GPIO_ResetBits(GPIOD,GPIO_Pin_6)    
#define	LCD_SCL_CLR  	GPIO_ResetBits(GPIOD,GPIO_Pin_11)  

#define	LCD_CS_CLR  	GPIO_ResetBits(GPIOC,GPIO_Pin_6)  
#define LCD_RST_CLR   GPIO_ResetBits(GPIOE,GPIO_Pin_0);//RST�������Ϊ��

#define	LCD_RS_CLR  	GPIO_ResetBits(GPIOE,GPIO_Pin_4) 
#define	LCD_BLK_CLR  	GPIO_ResetBits(GPIOE,GPIO_Pin_5) 



void LCD_GPIO_Init(void);//��ʼ��IO��
void  SPI_WriteData(u8 Data);//STM32_ģ��SPIдһ���ֽ����ݵײ㺯��
void LCD_WriteData_16Bit(uint16_t Data);//��Һ����дһ��16λ����

void Lcd_WriteIndex(u8 Index);//д�������Ĵ�����ַ
void Lcd_WriteData(u8 Data);//  д�Ĵ�������
void Lcd_WriteReg(u8 Index,u8 Data);////д�Ĵ�������

u16 Lcd_ReadReg(u8 LCD_Reg);

void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(u16 Color);
void Lcd_SetXY(u16 x,u16 y);
void Gui_DrawPoint(u16 x,u16 y,u16 Data);
//unsigned int Lcd_ReadPoint(u16 x,u16 y);

