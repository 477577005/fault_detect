#include <reg52.h>
#include <intrins.h>//延时函数头文件
#include <absacc.h> //绝对地址访问头文件
#define uchar unsigned char
#define u8 unsigned char
#define uint unsigned int
#define uint16_t unsigned  short int
#define PCF8563_addr 0XA2
#define dev24C02_addr 0XA8
#define ZLG7290_addr 0X70

/*****************端口接线************/
/*			ISD1420语音控制		*/
sbit REC =P1^2;	sbit PLAYE=P1^3;
/*     			 iic接线      */
sbit IIC_SCL=P1^1;//SCL
sbit IIC_SDA=P1^0;//SDA
/*	   			TLC549接线		*/
sbit CLK = P1^2; // 时钟信号
sbit DATA = P1^3; // 数据输出
sbit CS = P1^4;  // 片选信号
/*******		TLC561接线********/
sbit DIN  = P1^2;  // 数据输入
sbit SCLK = P1^3;  // 时钟
//sbit CS   = P1^4;  // 片选 与TLC549共用
/*******************adc0809**********/
sbit eoc=P1^2;sbit clk=P1^3;
/*****************单引脚控制器件接线*****************/
sbit one_input_class=P1^7;//测试结果

/*****************定义步进电机和38译码器引脚**************/
sbit A = P1^2;sbit B1 = P1^3;sbit C = P1^4;sbit D = P1^5;
/****************DS18B20接线**************************/
sbit DS18B20_PORT=P1^2;	//DS18B20数据口定义
/*********************串并转换和并串转换***************************************/
// HC165 控制引脚  并入
sbit HC165_SH_LD =P1^2;//#define HC165_SH_LD P1^2  // 移位/装载控制
sbit HC165_CLK  = P1^3;  // 时钟
sbit HC165_DATA  =P1^4;  // 串行数据输入（QH）
// HC164 控制引脚  并出
sbit HC164_DATA  =P1^5; // 串行数据输出（A/B）
sbit HC164_CLK  = P1^6;  // 时钟）
/***************************************串口函数声明及变量定义*******************/
bit rx_flag = 0;                 // 接收完成标志
bit INT0_flag;uchar input_code;uchar key_code1;
#define BUF_SIZE 4   // 接收缓冲区大小
void INT0_Init();//P3^2连接ZLG7290的INT(KEY)
unsigned char rx_buf[BUF_SIZE];  // 接收缓冲区
unsigned char result_buf[22];  // 接收缓冲区
unsigned char rx_count = 0;      // 接收数据数
void UART_Init();
void UART_SendByte(unsigned char dat);
void UART_SendString(char *str);
/*******************************************IIC函数优先声明及变量定义*************/
/*********数码管译码********/
uchar codevalue[10]={0xfc,0x60,0xda,0xf2,0x66,0xb6,0xbe,0xe0,0xfe,0xf6  /*AF*/ };
void DelayMs(int i);
void iic_start(void);
void iic_stop(void);
void iic_ack(void);
void iic_nack(void);
uchar iic_wait_ack(void);
void iic_write_byte(uchar dat);
uchar iic_read_byte(uchar ack);

// 多个相似的I2C读写函数可以合并
void IIC_Write(uchar dev_addr, uchar write_addr, uchar dat);
uchar IIC_Read(uchar dev_addr,uchar read_addr);
/***************ZLG7290函数声明**********************/
void ZLG7290_SetLED(uchar digit, uchar value);
uchar ZLG7290_ReadKey();
/*************TLC549函数声明*************/
void TLC549_Init();		
uchar TLC549_ReadByte();
/*************TLC561函数声明*************/
void TLC5615_Write(uint16_t data1);
/*****************************串并转换并串转换函数声明***************************/
unsigned char HC165_ReadByte();
void HC164_SendByte(unsigned char dat);
/*****************DS18B20函数声明***********************/
void delay_10us(int ten_us);
void delay_ms(int ms);
void ds18b20_reset(void);
unsigned char ds18b20_check(void);
unsigned char ds18b20_read_bit(void);
unsigned char ds18b20_read_byte(void);
void ds18b20_write_byte(u8 dat);
void ds18b20_start(void);
u8 ds18b20_init(void);
float ds18b20_read_temperture(void);
uchar int_to_char(uint16_t dat);
void smg_anjian();
/*********************检测器件声明函数**********/
void test_A2_16_16LED(void);	//用到8255加F5按键 外接8255辅助
void test_B1_ISD1420(void);		//
void test_B2_TLC549(void);		//模数  
void test_B3_DAC0832(void);     //数模：直流电机(高电平)
void test_B4_ADC0809(void);		//模数  串口
void test_C2_TLC561(void);		//数模  直流电机
void test_D1_stepMotor(void);	//P1^2到P1^5
void test_D3_24c02(void);		//串口输出
void test_D3_PCF(void);			//串口显示，键盘检测，PCF时钟	
void test_F8_DS18B20(void);		//光耦（低电平）VCC接地
void test_smgAnjian(void);
void test_8led(void);
void test_8Anjian(void);
void test_8Kaiguan(void);
void test_8led(void);
void test_relay(void);
void test_photocoupling(void);
void test_motor(void);
void test_Buzzer(void);
void test_hc244_hc273(void);
void test_chuanbin(void);
void test_138(void);

/*     五个模块    */
void test_single_IO_module(void);     //低电平：继电器,蜂鸣器
void test_Multi_IO_module(void);//开关八位接JP36，转换出来的jp49接8个LED
void test_8255_module(void);
void test_iic_module(void);
//void test_ADC_DAC_module();
void test_AD_module(void);
void result_display(void);
void show_home_page(void);


/****************主函数*******************/
void main()
{	
	UART_Init();
	input_code=0x11;
	while(1)
	 {		
			switch(input_code) {
		 
				case 0x01: test_single_IO_module(); break;	//单IO控制类：继电器、光耦、电机、鸣器、温度传感			1/
				case 0x02: test_Multi_IO_module(); break;	//多IO控制类：串并并串、简单io、138译码、语言控制、步进电机2/
				case 0x03: test_8255_module(); break;		//8255类：8led,8开关，8按键，16*16led  	 			3/
				case 0x04: test_iic_module(); break;		//IIc控制类：24c02、pcf8563、zlg7290					4/
				case 0x05: test_AD_module(); break;			//数模转换类 ：TLC549									5/
				case 0x0E: result_display(); break;			//结果总览
				case 0x11: show_home_page(); break;			//显示主页
				default: break;
			}
	  }
}

void show_home_page(void){
	UART_SendString("Its home page 	input number begin test\r\n ");
	UART_SendString("1:single_io_module   2:multi_io_module 3:8255_module  \r\n ");
	UART_SendString("4:iic_module	5:adda_module  	a:result_display\r\n");input_code=0;
}
/*					模块实现            */
void test_single_IO_module(void)    //8低电平：共用P1^7（继电器{VCC接地},蜂鸣器）成功继电器通 蜂鸣器响 
{
	UART_SendString("test_single_IO_module\r\n ");input_code=0;
	while(input_code != 0x11) {
        switch(input_code) {
			case 0x00: UART_SendString("input number begin test  1:relay  3:motor 4:buzzer 5:DS18B20 q:quit\r\n  ");input_code=0x12;break;
            case 0x01: test_relay(); break;
			// case 0x02: test_photocoupling(); break;        //实验箱高偶好像器件都损坏了，一上电就自带上拉电阻了
			case 0x03: test_motor(); break;
            case 0x04: test_Buzzer(); break;
			case 0x05: test_F8_DS18B20(); break;
			default: break;
        }
    }
}
void test_Multi_IO_module(void)	//12开关八位接JP36，转换出来的jp49接8个LED	
{
	UART_SendString("test_Multi_IO_module\r\n ");input_code=0;
	while(input_code != 0x11) {
        switch(input_code) {
			case 0x00: UART_SendString("input 1:hc273_244 2:chuanbin 3:138 4:ISD1420 5:stepMotor q:quit\r\n ");input_code=0x12;break;
            case 0x01: test_hc244_hc273(); break;
            case 0x02: test_chuanbin(); break;
			case 0x03: test_138(); break;
            case 0x04: test_B1_ISD1420(); break;
			case 0x05: test_D1_stepMotor(); break;
			default: break;
        }
    }
}
void test_8255_module()
{	//uchar i;
	UART_SendString("test_8255_module\r\n ");input_code=0;
	while(input_code != 0x11) {
        switch(input_code) {
			case 0x00: UART_SendString("input 1:16_16LED 2:8led 3:8Anjian 4:8Kaiguan q:quit\r\n");input_code=0x12;break;
            case 0x01: test_A2_16_16LED(); break;
            case 0x02: test_8led(); break;
			case 0x03: test_8Anjian(); break;
            case 0x04: test_8Kaiguan(); break;
			default: break;
        }
    }
}
void test_iic_module(void)        //11
{	UART_SendString("test_iic_module\r\n ");input_code=0;
	while(input_code != 0x11) {
        switch(input_code) {
			case 0x00: UART_SendString("input 1:24c02  2:PCF8563 3:smg_anjian q:quit\r\n ");input_code=0x12;break;
            case 0x01: test_D3_24c02(); break;
            case 0x02: test_D3_PCF(); break;
			case 0x03: test_smgAnjian(); break;
			default: break;
        }
    }
}
void test_AD_module()
{
	UART_SendString("test_AD_module\r\n ");input_code=0;
	while(input_code != 0x11) {
        switch(input_code) {
			case 0x00: UART_SendString("input 1:TLC549 2:ADC0809 3:TLC561 4:DAC0832 q:quit\r\n ");input_code=0x12;break;
            case 0x01: test_B2_TLC549(); break;
            case 0x02: test_B4_ADC0809(); break;
			case 0x03: test_C2_TLC561(); break;
            case 0x04: test_B3_DAC0832(); break;
			default: break;
        }
    }
}
/******************各器件实现********************/
void test_relay(void){
	UART_SendString("wiring is as follows:C4_CTRL->P1_7;COM1->VCC; COUT1->F3_test. input:1 begin test\r\n ");
	UART_SendString("if green light is on, then device is ok ;else device bad\r\n ");
	input_code=0;one_input_class=1;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x01: input_code=6;UART_SendString("test_relay begin\r\n "); break;
            case 0x06: one_input_class=0; break;
			case 0x0F: result_buf[7]='h';input_code=0x11; break;
            case 0x10: result_buf[7]='n';input_code=0x11; break;
			default: break;
        }
	}
	input_code=0;
}
/*
void test_photocoupling(void){
	UART_SendString("test_photocoupling input:2	D4_SIN1->P1_7  SOUT1->F3_test\r\n ");
	input_code=0;one_input_class=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x02: input_code=6;UART_SendString("test_photocoupling begin\r\n "); break;
            case 0x06: one_input_class=1; break;
			case 0x0F: result_buf[15]='h';input_code=0x11; break;
            case 0x10: result_buf[15]='n';input_code=0x11; break;
			default: break;
        }
	}
	input_code=0;
}*/
void test_motor(void){
	UART_SendString("test_motor input:3	C4_CTRL->P1_7 COM1->VCC COUT1->E1_CTRL\r\n ");one_input_class=1;input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x03: input_code=6;UART_SendString("test_motor begin\r\n "); break;
            case 0x06: one_input_class=0; break;
			case 0x0F: result_buf[16]='h';input_code=0x11; break;
            case 0x10: result_buf[16]='n';input_code=0x11; break;
			default: break;
        }
	}
	input_code=0;
}
void test_Buzzer(void){
	UART_SendString("test_buzzer input:4  F6_Ctrl->P1_7\r\n ");one_input_class=1;input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x04: input_code=6;UART_SendString("test_Buzzer begin\r\n "); break;
            case 0x06: one_input_class=0; break;
			case 0x0F: result_buf[17]='h';input_code=0x11; break;
            case 0x10: result_buf[17]='n';input_code=0x11; break;
			default: break;
        }
	}
	input_code=0;one_input_class=1;
}

void test_hc244_hc273(void){
	uchar i;
	UART_SendString("test_273and244 input:1\r\n ");
	UART_SendString("CS_273->GND CS_244->GND JP66->F5_JP65 JP104->F5_JP80\r\n ");input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x01: input_code=6;UART_SendString("test_hc244_hc273 begin\r\n "); break;
            case 0x06: i=XBYTE[0xffff];XBYTE[0xffff]=i; 	break;
			case 0x0F: result_buf[13]='h';input_code=0x11; break;
            case 0x10: result_buf[13]='n';input_code=0x11; break;
			default: break;
        }
	}

	input_code=0;
}
void test_chuanbin(void){
	uchar data8;
	UART_SendString("test_chuanbin input:2\r\n ");
	UART_SendString("SH_LD->P1_2 CLK->P1_3 QH->P1_4  JP36->F5_JP80\r\n ");
	UART_SendString("A/B->P1_5 CLK->P1_6  JP49->F5_JP65\r\n ");input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x02: input_code=6;UART_SendString("test_chuanbin begin\r\n "); break;
            case 0x06: data8=HC165_ReadByte();  // 从HC165读取数据
					   HC164_SendByte(data8);   // 发送到HC164
			           DelayMs(100); 	break;
			case 0x0F: result_buf[11]='h';input_code=0x11; break;
            case 0x10: result_buf[11]='n';input_code=0x11; break;
			default: break;
        }
	}
	input_code=0;
}
void test_138(void){
	UART_SendString("test_138 input:3\r\n ");
	UART_SendString("A->P1_2 B->P1_3 C->P1_4 G1->VCC G2A/G2B->GND JP35->F5_JP65\r\n ");input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x03: input_code=6;UART_SendString("test_138 begin\r\n "); break;
            case 0x06: 	A=B1=C=0;DelayMs(500);A=B1=C=1;DelayMs(500);	
						A=0;B1=0;C=1;DelayMs(500);A=0;B1=1;C=0;DelayMs(500);A=0;B1=1;C=1;DelayMs(500);
						A=1;B1=0;C=0;DelayMs(500);A=1;B1=0;C=1;DelayMs(500);A=1;B1=1;C=0;DelayMs(500); 	break;
			case 0x0F: result_buf[14]='h';input_code=0x11; break;
            case 0x10: result_buf[14]='n';input_code=0x11; break;
			default: break;
        }
	}
	input_code=0;
}


void test_8led(void){
	//uchar i;
	input_code=0;
	UART_SendString("test_8led input:2  F5_JP65->B6_JP56\r\n ");
	XBYTE[0xf003]=0x80;//
	while(input_code!=0x11){
		switch(input_code) {
            case 0x02: input_code=6;UART_SendString("test_8led begin\r\n "); break;
            case 0x06: 	XBYTE[0xf000]=0xAA;DelayMs(500);XBYTE[0xf000]=0x55;DelayMs(500);	break;
			case 0x0F: result_buf[19]='h';input_code=0x11; break;
            case 0x10: result_buf[19]='n';input_code=0x11; break;
			default: break;
        }
	}
		
			input_code=0;
}
void test_8Anjian(void){
	uchar i;
	input_code=0;
	UART_SendString("test_8Anjian input:3 F5_JP74->B6_JP53 F5_JP65->B6_JP56 \r\n ");
	XBYTE[0xf003]=0x82;//PB输入，PA输出
	while(input_code!=0x11){
		switch(input_code) {
            case 0x03: input_code=6;UART_SendString("test_8Anjian begin\r\n "); break;
            case 0x06: i=XBYTE[0xf001];XBYTE[0xf000]=i;	break;
			case 0x0F: result_buf[20]='h';input_code=0x11; break;
            case 0x10: result_buf[20]='n';input_code=0x11; break;
			default: break;
        }
	}
	
	input_code=0;
}
void test_8Kaiguan(void){
	uchar i;
	input_code=0;
	XBYTE[0xf003]=0x82;//PB输入，PA输出
	UART_SendString("test_8Kaiguan input:4 F5_JP80->B6_JP53 F5_JP65->B6_JP56 \r\n ");
	while(input_code!=0x11){
		switch(input_code) {
            case 0x04: input_code=6;UART_SendString("test_8Kaiguan begin\r\n "); break;
            case 0x06: i=XBYTE[0xf001];XBYTE[0xf000]=i;	break;
			case 0x0F: result_buf[21]='h';input_code=0x11; break;
            case 0x10: result_buf[21]='n';input_code=0x11; break;
			default: break;
        }
	}
		
	input_code=0;
}


void test_smgAnjian(void)		//10
{
	INT0_Init();
	UART_SendString("test_smgAnjian input:3 D3_B->F4_B D3_C->F4_C SCL->P1_1 SDA->P1_0\r\n ");input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x03: input_code=6;UART_SendString("test_smgAnjian begin\r\n "); break;
            case 0x06: smg_anjian();	break;
			case 0x0F: result_buf[18]='h';input_code=0x11; break;
            case 0x10: result_buf[18]='n';input_code=0x11; break;
			default: break;
        }
	}

	IT0 = 0;    // 下降沿触发
    EX0 = 0;    // 使能INT0中断
	input_code=0;
}

void test_A2_16_16LED(void)	// B接jp23、24  高电平点亮
{
	UART_SendString("test_A2_16_16LED input:1	B6:A0->A0 A1->A1\r\n ");
	UART_SendString("JP22->B6_B JP24->B6_C  CS->CS1\r\n ");input_code=0;
	XBYTE[0xf003]=0x80;  //PA输出 PB输入
	XBYTE[0xf000]=0x00;
	XBYTE[0xf001]=0x00;
		while(input_code!=0x11){
		switch(input_code) {
            case 0x01: input_code=6;UART_SendString("test_A2_16_16LED begin\r\n "); break;
            case 0x06:	XBYTE[0xf000]=0x01;DelayMs(500);XBYTE[0xf000]=0x02;DelayMs(500);
						XBYTE[0xf000]=0x04;DelayMs(500);XBYTE[0xf000]=0x08;DelayMs(500);
						XBYTE[0xf000]=0x10;DelayMs(500);XBYTE[0xf000]=0x20;DelayMs(500);
						XBYTE[0xf000]=0x40;DelayMs(500);XBYTE[0xf000]=0x80;DelayMs(500);	break;
			case 0x0F: result_buf[1]='h';input_code=0x11; break;
            case 0x10: result_buf[1]='n';input_code=0x11; break;
			default: break;
        }
	}

	input_code=0;
}

void test_B1_ISD1420(void)	//6	
{	REC=1;
	UART_SendString("test_B1_ISD1420 input:4	REC->P1_2 PLAYE->P1_3\r\n ");input_code=0;
	PLAYE=1;REC=1; //DelayMs(1000);
		while(input_code!=0x11){
		switch(input_code) {
            case 0x04: input_code=6;UART_SendString("test_ISD1420 begin\r\n ");REC=0; break;
            case 0x06: PLAYE=1;DelayMs(2000);REC=1;PLAYE=0;	break;
			case 0x0F: result_buf[2]='h';input_code=0x11; break;
            case 0x10: result_buf[2]='n';input_code=0x11; break;
			default: break;
        }
	}
	
	input_code=0;
}
void test_B2_TLC549(void)	//1	
{	uchar adc_value;
	UART_SendString("test_B2_TLC549 input:1\r\n ");
	UART_SendString("CLK->P1_2 DATA->P1_3 CS->P1_4 AIN->F1\r\n ");input_code=0;
	TLC549_Init();  // 初始化TLC549
	while(input_code!=0x11){
		switch(input_code) {
            case 0x01: input_code=6;UART_SendString("test_TLC549 begin\r\n "); break;
            case 0x06:  adc_value = TLC549_ReadByte();  // 读取ADC值
						adc_value=(adc_value*5)/255;
						UART_SendByte((int_to_char(adc_value)));UART_SendString("\r\n");adc_value=0;
						DelayMs(1000);	break;
			case 0x0F: result_buf[3]='h';input_code=0x11; break;
            case 0x10: result_buf[3]='n';input_code=0x11; break;
			default: break;
        }
	}

	input_code=0;
}
void test_B3_DAC0832(void)	//4	
{
	UART_SendString("test_B3_DAC0832 input:4 CS->CS1  OUT->E1_CTRL\r\n ");
	input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x04: input_code=6;UART_SendString("test_DAC0832 begin\r\n "); break;
            case 0x06:	XBYTE[0xFF00]=0;
						DelayMs(500);
						XBYTE[0xFF00]=0xFF;
						DelayMs(500);	break;
			case 0x0F: result_buf[4]='h';input_code=0x11; break;
            case 0x10: result_buf[4]='n';input_code=0x11; break;
			default: break;
        }
	}

	input_code=0;
}
void test_B4_ADC0809(void)	//2  用了定时器
{	
	uint16_t getdata;unsigned short int i=0;
	ET0=1;
	EA=1;
	TMOD |=0x02;
	TH0=246;
	TL0=246;
	TR0=1;
	UART_SendString("test_B4_ADC0809 input:2\r\n ");input_code=0;
	UART_SendString("EOC->P1_2 CLK->P1_3 CS->CS1 ADDA~C->GND INT0->F1\r\n ");
	while(input_code!=0x11){
		switch(input_code) {
            case 0x02: input_code=6;UART_SendString("test_ADC0809 begin\r\n "); break;
            case 0x06:  XBYTE[0X8000]=0;
						while(eoc==0){ i++;if(i>30000){i=0;break;} ;}
						getdata=XBYTE[0X8000];
						getdata=(getdata*5)/255;
						UART_SendByte((int_to_char(getdata)));UART_SendString("\r\n");DelayMs(500);getdata=0;break;
			case 0x0F: result_buf[5]='h';input_code=0x11; break;
            case 0x10: result_buf[5]='n';input_code=0x11; break;
			default: break;
        }
	}

	TR0=0;input_code=0;
}
void test_C2_TLC561(void)	//3		用到直流电机
{
	UART_SendString("test_C2_TLC561 input:3\r\n ");
	UART_SendString("DIN->P1_2 SCLK->P1_3 CS->P1_4 OUT->E1_CTRL\r\n ");input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x03: input_code=6;UART_SendString("test_TLC561 begin\r\n "); break;
            case 0x06: 	TLC5615_Write(10);
						DelayMs(1000);
						//TLC5615_Write(100);// 
						TLC5615_Write(7774);// 1111 1111 1100  高10位全为1
						DelayMs(1000); 	break;
			case 0x0F: result_buf[6]='h';input_code=0x11; break;
            case 0x10: result_buf[6]='n';input_code=0x11; break;
			default: break;
        }
	}

	input_code=0;
}

void test_D1_stepMotor(void)	//7
{
	UART_SendString("test_D1_stepMotor input:5	A->P1_2 B->P1_3 C->P1_4 D->P1_5\r\n ");
	input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x05: input_code=6;UART_SendString("test_stepMotor begin\r\n "); break;
            case 0x06: 	A = 1; B1 = 0; C = 0; D = 0; DelayMs(100);
						A = 0; B1 = 1; C = 0; D = 0; DelayMs(100);
						A = 0; B1 = 0; C = 1; D = 0; DelayMs(100);
						A = 0; B1 = 0; C = 0; D = 1; DelayMs(100);	break;
			case 0x0F: result_buf[8]='h';input_code=0x11; break;
            case 0x10: result_buf[8]='n';input_code=0x11; break;
			default: break;
        }
	}

	input_code=0;
}
void test_D3_24c02(void)	//1
{
	
	UART_SendString("test_D3_24c02 input:1	SCL->P1_1 SDA->P1_0\r\n ");
	//IIC24c02_write_one_byte(10,22);
	IIC_Write(dev24C02_addr,10,22);
	input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x01: input_code=6;UART_SendString("test_24c02 begin\r\n "); break;
            case 0x06:  if(IIC_Read(dev24C02_addr,10) == 22 )	UART_SendString("24c02 good\r\n ");
						else									UART_SendString("24c02 error\r\n ");
						DelayMs(1000);	break;
			case 0x0F: result_buf[9]='h';input_code=0x11; break;
            case 0x10: result_buf[9]='n';input_code=0x11; break;
			default: break;
        }
	}

	input_code=0;
}
void test_D3_PCF(void)	//2	用到8255和LED辅助测试 和TLC549一样借助8255
{
	uchar i;input_code=0;
	UART_SendString("test_D3_PCF input:2  SCL->P1_1 SDA->P1_0\r\n ");
	while(input_code!=0x11){
		switch(input_code) {
            case 0x02: input_code=6;UART_SendString("test_PCF begin\r\n "); break;
            case 0x06:	i=IIC_Read(PCF8563_addr,0X02);
						i=(( (i >> 4)*10)+(i&0x0F)); //BCD转十进制
						//XBYTE[0xf000]=PCF_read_one_byte(0X02);
						UART_SendByte((int_to_char(i/10)));
						UART_SendByte(int_to_char(i%10));UART_SendString("\r\n");
						DelayMs(1000);i=0;	break;
			case 0x0F: result_buf[10]='h';input_code=0x11; break;
            case 0x10: result_buf[10]='n';input_code=0x11; break;
			default: break;
        }
	}

	input_code=0;
}


void test_F8_DS18B20(void)		//8    
{	uint16_t value_temp;
	UART_SendString("test_F8_DS18B20 input:5  TOUT->P1_2\r\n ");
	ds18b20_init();input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x05: input_code=6;UART_SendString("test_DS18B20 begin\r\n "); break;
            case 0x06:  value_temp=ds18b20_read_temperture();
						UART_SendByte((int_to_char(value_temp/10)));
						UART_SendByte(int_to_char((value_temp%10)));UART_SendString("\r\n");
						DelayMs(500);	break;
			case 0x0F: result_buf[12]='h';input_code=0x11; break;
            case 0x10: result_buf[12]='n';input_code=0x11; break;
			default: break;
        }
	}

	input_code=0;
}
void result_display(void)		//14
{
	UART_SendString("result_display\r\n ");
	//UART_SendString("lcd12864:");	UART_SendByte(result_buf[0]);
	UART_SendString("relay:");	UART_SendByte(result_buf[7]);
	//UART_SendString("	photocoupling:");	UART_SendByte(result_buf[15]);
	UART_SendString("	motor:");	UART_SendByte(result_buf[16]);
	UART_SendString("	buzzer:");	UART_SendByte(result_buf[17]);
	UART_SendString("	DS18B20:");	UART_SendByte(result_buf[12]);
	UART_SendString("\r\n ");
	UART_SendString("HC273_hc244:");	UART_SendByte(result_buf[13]);
	UART_SendString("	chuanbin:");	UART_SendByte(result_buf[11]);
	UART_SendString("	138:");	UART_SendByte(result_buf[14]);
	UART_SendString("	ISD1420:");	UART_SendByte(result_buf[2]);
	UART_SendString("	stepMotor:");UART_SendByte(result_buf[8]);
	UART_SendString("\r\n ");
	UART_SendString("16_16LED:");	UART_SendByte(result_buf[1]);
	UART_SendString("	8led:");	UART_SendByte(result_buf[19]);
	UART_SendString("	8Anjian:");	UART_SendByte(result_buf[20]);
	UART_SendString("	8Kaiguan:");	UART_SendByte(result_buf[21]);
	UART_SendString("\r\n ");
	UART_SendString("24c02:");	UART_SendByte(result_buf[9]);
	UART_SendString("	PCF:");		UART_SendByte(result_buf[10]);
	UART_SendString("	smgAnjian:");		UART_SendByte(result_buf[18]);
	UART_SendString("\r\n ");
	UART_SendString("TLC549:");	UART_SendByte(result_buf[3]);
	UART_SendString("	ADC0809:");	UART_SendByte(result_buf[5]);
	UART_SendString("	TLC561:");	UART_SendByte(result_buf[6]);
	UART_SendString("	DAC0832:");	UART_SendByte(result_buf[4]);
	UART_SendString("\r\n ");
	input_code=0x11;UART_SendString("\r\n ");
	//UART_SendString("Its home page\r\n ");
}

/******************************************TLC549函数实现**********************************/
// 初始化TLC549
void TLC549_Init() {
	CS = 1;  // 片选信号置高
	CLK = 0; // 时钟信号置低
}
// 从TLC549读取一个字节
uchar TLC549_ReadByte() {
	uchar i, dat = 0;
	CS = 0;  // 片选信号置低，启动转换
	for (i = 0; i < 8; i++) {
		dat <<= 1;          // 左移一位
		CLK = 1;           // 时钟信号置高
		if (DATA) dat |= 1; // 读取数据位
		CLK = 0;           // 时钟信号置低
	}
	 CS = 1;  // 片选信号置高，结束转换
	return dat;
}
/*************DAC0809实现  定时中断函数********************/
void T0x(void ) interrupt 1 using 0        
	{
		clk=~clk;	
	}
/******************	TLC561函数实现		****************/
void TLC5615_Write(uint16_t data1) {
    uchar i;
    CS = 0;          // 使能芯片
    
    // TLC5615需要12bit数据（10bit数据 + 2bit填充）
    data1 <<= 2;      // 左移2位补零
    
    for(i=0; i<12; i++) {
        SCLK = 0;
        DIN = (data1 & 0x800) ? 1 : 0;  // 取最高位
        data1 <<= 1;
        SCLK = 1;     // 上升沿发送数据
        _nop_();      // 短暂延时
    }
    SCLK = 0;
    CS = 1;           // 禁用芯片
}
/**********************************************串并并串转 *****************************************/
// 从HC165读取1字节（并行→串行）
unsigned char HC165_ReadByte() {
    unsigned char i;
	unsigned char dat = 0;
    HC165_SH_LD=0;//HC165_SH_LD = 0; // 装载并行数据（低电平有效）
    _nop_();          // 短暂延时（确保稳定）
    HC165_SH_LD = 1;  // 开始移位（高电平）
    
    for (i = 0; i < 8; i++) {
        dat <<= 1;            // 左移1位（MSB First）
        if (HC165_DATA) dat |=0x01;  // 读取数据位
        
        HC165_CLK = 0;  // 时钟下降沿（准备）
        _nop_();
        HC165_CLK = 1;  // 时钟上升沿（移位）
        _nop_();
    }
    return dat;
}
// 向HC164发送1字节（串行→并行）
void HC164_SendByte(unsigned char dat) {
    unsigned char i;
    for (i = 0; i < 8; i++) {
        HC164_DATA = (dat >> (7 - i)) & 0x01;  // 从高位到低位发送
        HC164_CLK = 0;  // 时钟下降沿（准备）
        _nop_();
        HC164_CLK = 1;  // 时钟上升沿（锁存）
        _nop_();
    }
}
/****************************************延时函数实现*******************************************/
void delay_10us(int ten_us)
{
	while(ten_us--);	
}
void DelayMs(int i)
{
	
	for(i;i>0;i--)
	{
		unsigned char ii, j;
		_nop_();
		ii = 2;
		j = 199;
		do
		{
			while (--j);
		} while (--ii);
	}
}
/*******************************************************************************
* 函 数 名         : ds18b20_reset
* 函数功能		   : 复位DS18B20  
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void ds18b20_reset(void)
{
	DS18B20_PORT=0;	//拉低DQ
	delay_10us(75);	//拉低750us
	DS18B20_PORT=1;	//DQ=1
	delay_10us(2);	//20US
}
/*******************************************************************************
* 函 数 名         : ds18b20_check
* 函数功能		   : 检测DS18B20是否存在
* 输    入         : 无
* 输    出         : 1:未检测到DS18B20的存在，0:存在
*******************************************************************************/
unsigned char ds18b20_check(void)
{
	unsigned char time_temp=0;

	while(DS18B20_PORT&&time_temp<20)	//等待DQ为低电平
	{
		time_temp++;
		delay_10us(1);	
	}
	if(time_temp>=20)return 1;	//如果超时则强制返回1
	else time_temp=0;
	while((!DS18B20_PORT)&&time_temp<20)	//等待DQ为高电平
	{
		time_temp++;
		delay_10us(1);
	}
	if(time_temp>=20)return 1;	//如果超时则强制返回1
	return 0;
}

/*******************************************************************************
* 函 数 名         : ds18b20_read_bit
* 函数功能		   : 从DS18B20读取一个位
* 输    入         : 无
* 输    出         : 1/0
*******************************************************************************/
unsigned char ds18b20_read_bit(void)
{
	unsigned char dat=0;
	
	DS18B20_PORT=0;
	_nop_();_nop_();
	DS18B20_PORT=1;	
	_nop_();_nop_(); //该段时间不能过长，必须在15us内读取数据
	if(DS18B20_PORT)dat=1;	//如果总线上为1则数据dat为1，否则为0
	else dat=0;
	delay_10us(5);
	return dat;
} 

/*******************************************************************************
* 函 数 名         : ds18b20_read_byte
* 函数功能		   : 从DS18B20读取一个字节
* 输    入         : 无
* 输    出         : 一个字节数据
*******************************************************************************/
unsigned char ds18b20_read_byte(void)
{
	unsigned char i=0;
	u8 dat=0;
	u8 temp=0;

	for(i=0;i<8;i++)//循环8次，每次读取一位，且先读低位再读高位
	{
		temp=ds18b20_read_bit();
		dat=(temp<<7)|(dat>>1);
	}
	return dat;	
}
/*******************************************************************************
* 函 数 名         : ds18b20_write_byte
* 函数功能		   : 写一个字节到DS18B20
* 输    入         : dat：要写入的字节
* 输    出         : 无
*******************************************************************************/
void ds18b20_write_byte(u8 dat)
{
	u8 i=0;
	u8 temp=0;
	for(i=0;i<8;i++)//循环8次，每次写一位，且先写低位再写高位
	{
		temp=dat&0x01;//选择低位准备写入
		dat>>=1;//将次高位移到低位
		if(temp){
			DS18B20_PORT=0;
			_nop_();_nop_();
			DS18B20_PORT=1;	
			delay_10us(6);
		}
		else{
			DS18B20_PORT=0;
			delay_10us(6);
			DS18B20_PORT=1;
			_nop_();_nop_();	
		}	
	}	
}

/*******************************************************************************
* 函 数 名         : ds18b20_start
* 函数功能		   : 开始温度转换
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void ds18b20_start(void)
{
	ds18b20_reset();//复位
	ds18b20_check();//检查DS18B20
	ds18b20_write_byte(0xcc);//SKIP ROM
    ds18b20_write_byte(0x44);//转换命令	
}

/*******************************************************************************
* 函 数 名         : ds18b20_init
* 函数功能		   : 初始化DS18B20的IO口 DQ 同时检测DS的存在
* 输    入         : 无
* 输    出         : 1:不存在，0:存在
*******************************************************************************/ 
u8 ds18b20_init(void)
{
	ds18b20_reset();
	return ds18b20_check();	
}

/*******************************************************************************
* 函 数 名         : ds18b20_read_temperture
* 函数功能		   : 从ds18b20得到温度值
* 输    入         : 无
* 输    出         : 温度数据
*******************************************************************************/
float ds18b20_read_temperture(void)
{
	float temp;
	u8 dath=0;
	u8 datl=0;
	uint16_t value=0;
	
	ds18b20_start();//开始转换
	ds18b20_reset();//复位
	ds18b20_check();
	ds18b20_write_byte(0xcc);//SKIP ROM
    ds18b20_write_byte(0xbe);//读存储器

	datl=ds18b20_read_byte();//低字节
	dath=ds18b20_read_byte();//高字节
	value=(dath<<8)+datl;//合并为16位数据

	if((value&0xf800)==0xf800)//判断符号位，负温度
	{
		value=(~value)+1; //数据取反再加1
		temp=value*(-0.0625);//乘以精度	
	}
	else //正温度
	{
		temp=value*0.0625;	
	}
	return temp;
	//return dath;
}


/**********************************************串口的实现*********************/
/**********************************************************************/

void UART_Init() {
    SCON = 0x50;        // 模式1（8位UART），允许接收（REN=1）
    TMOD |= 0x20;       // 定时器1模式2（8位自动重装）
    TH1 = 0xFD;         // 波特率9600（12MHz晶振）
    TL1 = 0xFD;
    TR1 = 1;            // 启动定时器1
    EA = 1;             // 开启总中断
    ES = 1;             // 开启串口中断
}

/**
 * @brief 发送一个字节
 * @param dat 要发送的数据
 */
void UART_SendByte(unsigned char dat) {
    SBUF = dat;         // 数据写入发送缓冲区
    while (!TI);        // 等待发送完成
    TI = 0;             // 清除发送中断标志
}
/**
 * @brief 发送字符串
 * @param str 要发送的字符串（以'\0'结尾）
 */

void UART_SendString(char *str) {
    while (*str) {
        UART_SendByte(*str++);
    }
}
/**
 * @brief 串口中断服务函数  接收函数
 */
void UART_ISR() interrupt 4 {
    if (RI) {                       // 接收中断
        RI = 0;                     // 清除接收标志
        rx_buf[0] = SBUF;  // 存储接收到的数据
		
        // 将串口输入字符作为对应数值
		if(rx_buf[0]=='1')input_code=0x01;
		else if(rx_buf[0]=='2')input_code=0x02;
		else if(rx_buf[0]=='3')input_code=0x03;
		else if(rx_buf[0]=='4')input_code=0x04;
		else if(rx_buf[0]=='5')input_code=0x05;
		else if(rx_buf[0]=='6')input_code=0x06;
		else if(rx_buf[0]=='7')input_code=0x07;
		else if(rx_buf[0]=='8')input_code=0x08;
		else if(rx_buf[0]=='9')input_code=0x09;
		else if(rx_buf[0]=='a')input_code=0x0E;
		else if(rx_buf[0]=='g')input_code=0x0F;
		else if(rx_buf[0]=='b')input_code=0x10;
		else if(rx_buf[0]=='q')input_code=0x11;
    }
	
}
/*************IIc的实现**********************/
/************************************************************************************/


void iic_start(void)
{
	IIC_SDA=1;//如果把该条语句放在SCL后面，第二次读写会出现问
	delay_10us(1);
	IIC_SCL=1;
	delay_10us(1);
	IIC_SDA=0;	//当SCL为高电平时，SDA由高变为
	delay_10us(1);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
	delay_10us(1);
}
/*******************************************************************************
*         : iic_stop
* 函数功能		   : 产生IIC停止信号   
*            : 
*            : 
*******************************************************************************/
void iic_stop(void)
{	
	IIC_SDA=0;//如果把该条语句放在SCL后面，第二次读写会出现问
	delay_10us(1);
	IIC_SCL=1;
	delay_10us(1);
	IIC_SDA=1;	//当SCL为高电平时，SDA由低变为
	delay_10us(1);			
}
/*******************************************************************************
*         : iic_ack
* 函数功能		   : 产生ACK应答  
*            : 
*            : 
*******************************************************************************/
void iic_ack(void)
{
	IIC_SCL=0;
	IIC_SDA=0;	//SDA为低电平
	delay_10us(1);
   	IIC_SCL=1;
	delay_10us(1);
	IIC_SCL=0;
}
/*******************************************************************************
*         : iic_nack
* 函数功能		   : 产生NACK非应 
*            : 
*            : 
*******************************************************************************/
void iic_nack(void)
{
	IIC_SCL=0;
	IIC_SDA=1;	//SDA为高电平
	delay_10us(1);
   	IIC_SCL=1;
	delay_10us(1);
	IIC_SCL=0;	
}
/*******************************************************************************
*         : iic_wait_ack
* 函数功能		   : 等待应答信号到来   
*            : 
*            : 1，接收应答失
        			 0，接收应答成
*******************************************************************************/
uchar iic_wait_ack(void)
{
	uchar time_temp=0;
	
	IIC_SCL=1;
	delay_10us(1);
	while(IIC_SDA)	//等待SDA为低电平
	{
		time_temp++;
		if(time_temp>100)//超时则强制结束IIC通信
		{	
			iic_stop();
			return 1;	
		}			
	}
	IIC_SCL=0;
	return 0;	
}
/*******************************************************************************
*         : iic_write_byte
* 函数功能		   : IIC发一个字
*            : dat：发送一个字
*            : 
*******************************************************************************/
void iic_write_byte(uchar dat)
{                        
    uchar i=0; 
	   	    
    IIC_SCL=0;
    for(i=0;i<8;i++)	//循环8次将个字节传出，先传高再传低
    {              
        if((dat&0x80)>0) 
			IIC_SDA=1;
		else
			IIC_SDA=0;
        dat<<=1; 	  
		delay_10us(1);  
		IIC_SCL=1;
		delay_10us(1); 
		IIC_SCL=0;	
		delay_10us(1);
    }	 
}
/*******************************************************************************
*         : iic_read_byte
* 函数功能		   : IIC读一个字
*            : ack=1时，发ACK，ack=0，发送nACK 
*            : 应答或非应答
*******************************************************************************/
uchar iic_read_byte(uchar ack)
{
	uchar i=0,receive=0;
   	
    for(i=0;i<8;i++ )	//循环8次将个字节读出，先读高再传低
	{
        IIC_SCL=0; 
        delay_10us(1);
		IIC_SCL=1;
        receive<<=1;
        if(IIC_SDA)receive++;   
		delay_10us(1); 
    }					 
    if (!ack)
        iic_nack();
    else
        iic_ack();  
		  
    return receive;
}

void ZLG7290_SetLED(uchar digit, uchar value)  
{
    //zlg7290_write_one_byte(0x10 + digit, value); 
	 IIC_Write(ZLG7290_addr, 0X10+digit,value);
	// IIC_Read(ZLG7290_addr,0x01);
}
uchar ZLG7290_ReadKey()
{
    //return zlg7290_read_one_byte(0x01); 
	return IIC_Read(ZLG7290_addr,0x01);
}

/*****************************优化后的IIC 写函数  ************/
// 多个相似的I2C读写函数可以合并

void IIC_Write(uchar dev_addr, uchar write_addr, uchar dat) {
    iic_start();
    iic_write_byte(dev_addr);
    iic_wait_ack();
    iic_write_byte(write_addr);
    iic_wait_ack();
    iic_write_byte(dat);
    iic_wait_ack();
    iic_stop();
    DelayMs(10);
}
uchar IIC_Read(uchar dev_addr,uchar read_addr)
{				  
	uchar temp=0;		  	    																 
    iic_start();  	
	iic_write_byte(dev_addr);	   //发写命令
	iic_wait_ack(); 
    iic_write_byte(read_addr); 	//发写地址  
	iic_wait_ack();	    
	iic_start();  	 	    	 
	iic_write_byte(dev_addr+1);        		//进入接收模式	   
	iic_wait_ack();	 
    temp=iic_read_byte(0);	//读取字节		   
    iic_stop();				//产生个停止条   
	return temp;			//返回读取的数
}
/*********************中断T0读取按键**********************/
// 初始化中断
void INT0_Init() {
    IT0 = 1;    // 下降沿触发
    EX0 = 1;    // 使能INT0中断
    EA = 1;     // 开启总中断
}


// 中断服务函数
void INT0_ISR() interrupt 0 {
     key_code1 = ZLG7290_ReadKey(); // 读取键值寄存器
}
//将整形数转为对应的字符，由于串口发送
uchar int_to_char(uint16_t dat)
{
	if(dat==0) return '0';else if(dat==1) return '1';else if(dat==2) return '2';
	else if(dat==3) return '3';else if(dat==4) return '4';else if(dat==5) return '5';
	else if(dat==6) return '6';else if(dat==7) return '7';else if(dat==8) return '8';
	else if(dat==9) return '9';
	return 0;
}

//
void smg_anjian()
{   if(key_code1<=8){ZLG7290_SetLED(key_code1-1,codevalue[(key_code1-1)]);}
	else if(key_code1>8 && key_code1<17){key_code1=(key_code1-8);ZLG7290_SetLED(key_code1,codevalue[(8-key_code1)]);key_code1=0;}
}

