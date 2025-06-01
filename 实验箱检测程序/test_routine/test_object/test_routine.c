#include <reg52.h>
#include <intrins.h>//��ʱ����ͷ�ļ�
#include <absacc.h> //���Ե�ַ����ͷ�ļ�
#define uchar unsigned char
#define u8 unsigned char
#define uint unsigned int
#define uint16_t unsigned  short int
#define PCF8563_addr 0XA2
#define dev24C02_addr 0XA8
#define ZLG7290_addr 0X70

/*****************�˿ڽ���************/
/*			ISD1420��������		*/
sbit REC =P1^2;	sbit PLAYE=P1^3;
/*     			 iic����      */
sbit IIC_SCL=P1^1;//SCL
sbit IIC_SDA=P1^0;//SDA
/*	   			TLC549����		*/
sbit CLK = P1^2; // ʱ���ź�
sbit DATA = P1^3; // �������
sbit CS = P1^4;  // Ƭѡ�ź�
/*******		TLC561����********/
sbit DIN  = P1^2;  // ��������
sbit SCLK = P1^3;  // ʱ��
//sbit CS   = P1^4;  // Ƭѡ ��TLC549����
/*******************adc0809**********/
sbit eoc=P1^2;sbit clk=P1^3;
/*****************�����ſ�����������*****************/
sbit one_input_class=P1^7;//���Խ��

/*****************���岽�������38����������**************/
sbit A = P1^2;sbit B1 = P1^3;sbit C = P1^4;sbit D = P1^5;
/****************DS18B20����**************************/
sbit DS18B20_PORT=P1^2;	//DS18B20���ݿڶ���
/*********************����ת���Ͳ���ת��***************************************/
// HC165 ��������  ����
sbit HC165_SH_LD =P1^2;//#define HC165_SH_LD P1^2  // ��λ/װ�ؿ���
sbit HC165_CLK  = P1^3;  // ʱ��
sbit HC165_DATA  =P1^4;  // �����������루QH��
// HC164 ��������  ����
sbit HC164_DATA  =P1^5; // �������������A/B��
sbit HC164_CLK  = P1^6;  // ʱ�ӣ�
/***************************************���ں�����������������*******************/
bit rx_flag = 0;                 // ������ɱ�־
bit INT0_flag;uchar input_code;uchar key_code1;
#define BUF_SIZE 4   // ���ջ�������С
void INT0_Init();//P3^2����ZLG7290��INT(KEY)
unsigned char rx_buf[BUF_SIZE];  // ���ջ�����
unsigned char result_buf[22];  // ���ջ�����
unsigned char rx_count = 0;      // ����������
void UART_Init();
void UART_SendByte(unsigned char dat);
void UART_SendString(char *str);
/*******************************************IIC����������������������*************/
/*********���������********/
uchar codevalue[10]={0xfc,0x60,0xda,0xf2,0x66,0xb6,0xbe,0xe0,0xfe,0xf6  /*AF*/ };
void DelayMs(int i);
void iic_start(void);
void iic_stop(void);
void iic_ack(void);
void iic_nack(void);
uchar iic_wait_ack(void);
void iic_write_byte(uchar dat);
uchar iic_read_byte(uchar ack);

// ������Ƶ�I2C��д�������Ժϲ�
void IIC_Write(uchar dev_addr, uchar write_addr, uchar dat);
uchar IIC_Read(uchar dev_addr,uchar read_addr);
/***************ZLG7290��������**********************/
void ZLG7290_SetLED(uchar digit, uchar value);
uchar ZLG7290_ReadKey();
/*************TLC549��������*************/
void TLC549_Init();		
uchar TLC549_ReadByte();
/*************TLC561��������*************/
void TLC5615_Write(uint16_t data1);
/*****************************����ת������ת����������***************************/
unsigned char HC165_ReadByte();
void HC164_SendByte(unsigned char dat);
/*****************DS18B20��������***********************/
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
/*********************���������������**********/
void test_A2_16_16LED(void);	//�õ�8255��F5���� ���8255����
void test_B1_ISD1420(void);		//
void test_B2_TLC549(void);		//ģ��  
void test_B3_DAC0832(void);     //��ģ��ֱ�����(�ߵ�ƽ)
void test_B4_ADC0809(void);		//ģ��  ����
void test_C2_TLC561(void);		//��ģ  ֱ�����
void test_D1_stepMotor(void);	//P1^2��P1^5
void test_D3_24c02(void);		//�������
void test_D3_PCF(void);			//������ʾ�����̼�⣬PCFʱ��	
void test_F8_DS18B20(void);		//����͵�ƽ��VCC�ӵ�
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

/*     ���ģ��    */
void test_single_IO_module(void);     //�͵�ƽ���̵���,������
void test_Multi_IO_module(void);//���ذ�λ��JP36��ת��������jp49��8��LED
void test_8255_module(void);
void test_iic_module(void);
//void test_ADC_DAC_module();
void test_AD_module(void);
void result_display(void);
void show_home_page(void);


/****************������*******************/
void main()
{	
	UART_Init();
	input_code=0x11;
	while(1)
	 {		
			switch(input_code) {
		 
				case 0x01: test_single_IO_module(); break;	//��IO�����ࣺ�̵��������������������¶ȴ���			1/
				case 0x02: test_Multi_IO_module(); break;	//��IO�����ࣺ������������io��138���롢���Կ��ơ��������2/
				case 0x03: test_8255_module(); break;		//8255�ࣺ8led,8���أ�8������16*16led  	 			3/
				case 0x04: test_iic_module(); break;		//IIc�����ࣺ24c02��pcf8563��zlg7290					4/
				case 0x05: test_AD_module(); break;			//��ģת���� ��TLC549									5/
				case 0x0E: result_display(); break;			//�������
				case 0x11: show_home_page(); break;			//��ʾ��ҳ
				default: break;
			}
	  }
}

void show_home_page(void){
	UART_SendString("Its home page 	input number begin test\r\n ");
	UART_SendString("1:single_io_module   2:multi_io_module 3:8255_module  \r\n ");
	UART_SendString("4:iic_module	5:adda_module  	a:result_display\r\n");input_code=0;
}
/*					ģ��ʵ��            */
void test_single_IO_module(void)    //8�͵�ƽ������P1^7���̵���{VCC�ӵ�},���������ɹ��̵���ͨ �������� 
{
	UART_SendString("test_single_IO_module\r\n ");input_code=0;
	while(input_code != 0x11) {
        switch(input_code) {
			case 0x00: UART_SendString("input number begin test  1:relay  3:motor 4:buzzer 5:DS18B20 q:quit\r\n  ");input_code=0x12;break;
            case 0x01: test_relay(); break;
			// case 0x02: test_photocoupling(); break;        //ʵ�����ż�������������ˣ�һ�ϵ���Դ�����������
			case 0x03: test_motor(); break;
            case 0x04: test_Buzzer(); break;
			case 0x05: test_F8_DS18B20(); break;
			default: break;
        }
    }
}
void test_Multi_IO_module(void)	//12���ذ�λ��JP36��ת��������jp49��8��LED	
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
/******************������ʵ��********************/
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
            case 0x06: data8=HC165_ReadByte();  // ��HC165��ȡ����
					   HC164_SendByte(data8);   // ���͵�HC164
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
	XBYTE[0xf003]=0x82;//PB���룬PA���
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
	XBYTE[0xf003]=0x82;//PB���룬PA���
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

	IT0 = 0;    // �½��ش���
    EX0 = 0;    // ʹ��INT0�ж�
	input_code=0;
}

void test_A2_16_16LED(void)	// B��jp23��24  �ߵ�ƽ����
{
	UART_SendString("test_A2_16_16LED input:1	B6:A0->A0 A1->A1\r\n ");
	UART_SendString("JP22->B6_B JP24->B6_C  CS->CS1\r\n ");input_code=0;
	XBYTE[0xf003]=0x80;  //PA��� PB����
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
	TLC549_Init();  // ��ʼ��TLC549
	while(input_code!=0x11){
		switch(input_code) {
            case 0x01: input_code=6;UART_SendString("test_TLC549 begin\r\n "); break;
            case 0x06:  adc_value = TLC549_ReadByte();  // ��ȡADCֵ
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
void test_B4_ADC0809(void)	//2  ���˶�ʱ��
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
void test_C2_TLC561(void)	//3		�õ�ֱ�����
{
	UART_SendString("test_C2_TLC561 input:3\r\n ");
	UART_SendString("DIN->P1_2 SCLK->P1_3 CS->P1_4 OUT->E1_CTRL\r\n ");input_code=0;
	while(input_code!=0x11){
		switch(input_code) {
            case 0x03: input_code=6;UART_SendString("test_TLC561 begin\r\n "); break;
            case 0x06: 	TLC5615_Write(10);
						DelayMs(1000);
						//TLC5615_Write(100);// 
						TLC5615_Write(7774);// 1111 1111 1100  ��10λȫΪ1
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
void test_D3_PCF(void)	//2	�õ�8255��LED�������� ��TLC549һ������8255
{
	uchar i;input_code=0;
	UART_SendString("test_D3_PCF input:2  SCL->P1_1 SDA->P1_0\r\n ");
	while(input_code!=0x11){
		switch(input_code) {
            case 0x02: input_code=6;UART_SendString("test_PCF begin\r\n "); break;
            case 0x06:	i=IIC_Read(PCF8563_addr,0X02);
						i=(( (i >> 4)*10)+(i&0x0F)); //BCDתʮ����
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

/******************************************TLC549����ʵ��**********************************/
// ��ʼ��TLC549
void TLC549_Init() {
	CS = 1;  // Ƭѡ�ź��ø�
	CLK = 0; // ʱ���ź��õ�
}
// ��TLC549��ȡһ���ֽ�
uchar TLC549_ReadByte() {
	uchar i, dat = 0;
	CS = 0;  // Ƭѡ�ź��õͣ�����ת��
	for (i = 0; i < 8; i++) {
		dat <<= 1;          // ����һλ
		CLK = 1;           // ʱ���ź��ø�
		if (DATA) dat |= 1; // ��ȡ����λ
		CLK = 0;           // ʱ���ź��õ�
	}
	 CS = 1;  // Ƭѡ�ź��øߣ�����ת��
	return dat;
}
/*************DAC0809ʵ��  ��ʱ�жϺ���********************/
void T0x(void ) interrupt 1 using 0        
	{
		clk=~clk;	
	}
/******************	TLC561����ʵ��		****************/
void TLC5615_Write(uint16_t data1) {
    uchar i;
    CS = 0;          // ʹ��оƬ
    
    // TLC5615��Ҫ12bit���ݣ�10bit���� + 2bit��䣩
    data1 <<= 2;      // ����2λ����
    
    for(i=0; i<12; i++) {
        SCLK = 0;
        DIN = (data1 & 0x800) ? 1 : 0;  // ȡ���λ
        data1 <<= 1;
        SCLK = 1;     // �����ط�������
        _nop_();      // ������ʱ
    }
    SCLK = 0;
    CS = 1;           // ����оƬ
}
/**********************************************��������ת *****************************************/
// ��HC165��ȡ1�ֽڣ����С����У�
unsigned char HC165_ReadByte() {
    unsigned char i;
	unsigned char dat = 0;
    HC165_SH_LD=0;//HC165_SH_LD = 0; // װ�ز������ݣ��͵�ƽ��Ч��
    _nop_();          // ������ʱ��ȷ���ȶ���
    HC165_SH_LD = 1;  // ��ʼ��λ���ߵ�ƽ��
    
    for (i = 0; i < 8; i++) {
        dat <<= 1;            // ����1λ��MSB First��
        if (HC165_DATA) dat |=0x01;  // ��ȡ����λ
        
        HC165_CLK = 0;  // ʱ���½��أ�׼����
        _nop_();
        HC165_CLK = 1;  // ʱ�������أ���λ��
        _nop_();
    }
    return dat;
}
// ��HC164����1�ֽڣ����С����У�
void HC164_SendByte(unsigned char dat) {
    unsigned char i;
    for (i = 0; i < 8; i++) {
        HC164_DATA = (dat >> (7 - i)) & 0x01;  // �Ӹ�λ����λ����
        HC164_CLK = 0;  // ʱ���½��أ�׼����
        _nop_();
        HC164_CLK = 1;  // ʱ�������أ����棩
        _nop_();
    }
}
/****************************************��ʱ����ʵ��*******************************************/
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
* �� �� ��         : ds18b20_reset
* ��������		   : ��λDS18B20  
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void ds18b20_reset(void)
{
	DS18B20_PORT=0;	//����DQ
	delay_10us(75);	//����750us
	DS18B20_PORT=1;	//DQ=1
	delay_10us(2);	//20US
}
/*******************************************************************************
* �� �� ��         : ds18b20_check
* ��������		   : ���DS18B20�Ƿ����
* ��    ��         : ��
* ��    ��         : 1:δ��⵽DS18B20�Ĵ��ڣ�0:����
*******************************************************************************/
unsigned char ds18b20_check(void)
{
	unsigned char time_temp=0;

	while(DS18B20_PORT&&time_temp<20)	//�ȴ�DQΪ�͵�ƽ
	{
		time_temp++;
		delay_10us(1);	
	}
	if(time_temp>=20)return 1;	//�����ʱ��ǿ�Ʒ���1
	else time_temp=0;
	while((!DS18B20_PORT)&&time_temp<20)	//�ȴ�DQΪ�ߵ�ƽ
	{
		time_temp++;
		delay_10us(1);
	}
	if(time_temp>=20)return 1;	//�����ʱ��ǿ�Ʒ���1
	return 0;
}

/*******************************************************************************
* �� �� ��         : ds18b20_read_bit
* ��������		   : ��DS18B20��ȡһ��λ
* ��    ��         : ��
* ��    ��         : 1/0
*******************************************************************************/
unsigned char ds18b20_read_bit(void)
{
	unsigned char dat=0;
	
	DS18B20_PORT=0;
	_nop_();_nop_();
	DS18B20_PORT=1;	
	_nop_();_nop_(); //�ö�ʱ�䲻�ܹ�����������15us�ڶ�ȡ����
	if(DS18B20_PORT)dat=1;	//���������Ϊ1������datΪ1������Ϊ0
	else dat=0;
	delay_10us(5);
	return dat;
} 

/*******************************************************************************
* �� �� ��         : ds18b20_read_byte
* ��������		   : ��DS18B20��ȡһ���ֽ�
* ��    ��         : ��
* ��    ��         : һ���ֽ�����
*******************************************************************************/
unsigned char ds18b20_read_byte(void)
{
	unsigned char i=0;
	u8 dat=0;
	u8 temp=0;

	for(i=0;i<8;i++)//ѭ��8�Σ�ÿ�ζ�ȡһλ�����ȶ���λ�ٶ���λ
	{
		temp=ds18b20_read_bit();
		dat=(temp<<7)|(dat>>1);
	}
	return dat;	
}
/*******************************************************************************
* �� �� ��         : ds18b20_write_byte
* ��������		   : дһ���ֽڵ�DS18B20
* ��    ��         : dat��Ҫд����ֽ�
* ��    ��         : ��
*******************************************************************************/
void ds18b20_write_byte(u8 dat)
{
	u8 i=0;
	u8 temp=0;
	for(i=0;i<8;i++)//ѭ��8�Σ�ÿ��дһλ������д��λ��д��λ
	{
		temp=dat&0x01;//ѡ���λ׼��д��
		dat>>=1;//���θ�λ�Ƶ���λ
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
* �� �� ��         : ds18b20_start
* ��������		   : ��ʼ�¶�ת��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void ds18b20_start(void)
{
	ds18b20_reset();//��λ
	ds18b20_check();//���DS18B20
	ds18b20_write_byte(0xcc);//SKIP ROM
    ds18b20_write_byte(0x44);//ת������	
}

/*******************************************************************************
* �� �� ��         : ds18b20_init
* ��������		   : ��ʼ��DS18B20��IO�� DQ ͬʱ���DS�Ĵ���
* ��    ��         : ��
* ��    ��         : 1:�����ڣ�0:����
*******************************************************************************/ 
u8 ds18b20_init(void)
{
	ds18b20_reset();
	return ds18b20_check();	
}

/*******************************************************************************
* �� �� ��         : ds18b20_read_temperture
* ��������		   : ��ds18b20�õ��¶�ֵ
* ��    ��         : ��
* ��    ��         : �¶�����
*******************************************************************************/
float ds18b20_read_temperture(void)
{
	float temp;
	u8 dath=0;
	u8 datl=0;
	uint16_t value=0;
	
	ds18b20_start();//��ʼת��
	ds18b20_reset();//��λ
	ds18b20_check();
	ds18b20_write_byte(0xcc);//SKIP ROM
    ds18b20_write_byte(0xbe);//���洢��

	datl=ds18b20_read_byte();//���ֽ�
	dath=ds18b20_read_byte();//���ֽ�
	value=(dath<<8)+datl;//�ϲ�Ϊ16λ����

	if((value&0xf800)==0xf800)//�жϷ���λ�����¶�
	{
		value=(~value)+1; //����ȡ���ټ�1
		temp=value*(-0.0625);//���Ծ���	
	}
	else //���¶�
	{
		temp=value*0.0625;	
	}
	return temp;
	//return dath;
}


/**********************************************���ڵ�ʵ��*********************/
/**********************************************************************/

void UART_Init() {
    SCON = 0x50;        // ģʽ1��8λUART����������գ�REN=1��
    TMOD |= 0x20;       // ��ʱ��1ģʽ2��8λ�Զ���װ��
    TH1 = 0xFD;         // ������9600��12MHz����
    TL1 = 0xFD;
    TR1 = 1;            // ������ʱ��1
    EA = 1;             // �������ж�
    ES = 1;             // ���������ж�
}

/**
 * @brief ����һ���ֽ�
 * @param dat Ҫ���͵�����
 */
void UART_SendByte(unsigned char dat) {
    SBUF = dat;         // ����д�뷢�ͻ�����
    while (!TI);        // �ȴ��������
    TI = 0;             // ��������жϱ�־
}
/**
 * @brief �����ַ���
 * @param str Ҫ���͵��ַ�������'\0'��β��
 */

void UART_SendString(char *str) {
    while (*str) {
        UART_SendByte(*str++);
    }
}
/**
 * @brief �����жϷ�����  ���պ���
 */
void UART_ISR() interrupt 4 {
    if (RI) {                       // �����ж�
        RI = 0;                     // ������ձ�־
        rx_buf[0] = SBUF;  // �洢���յ�������
		
        // �����������ַ���Ϊ��Ӧ��ֵ
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
/*************IIc��ʵ��**********************/
/************************************************************************************/


void iic_start(void)
{
	IIC_SDA=1;//����Ѹ���������SCL���棬�ڶ��ζ�д�������
	delay_10us(1);
	IIC_SCL=1;
	delay_10us(1);
	IIC_SDA=0;	//��SCLΪ�ߵ�ƽʱ��SDA�ɸ߱�Ϊ
	delay_10us(1);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ��������
	delay_10us(1);
}
/*******************************************************************************
*         : iic_stop
* ��������		   : ����IICֹͣ�ź�   
*            : 
*            : 
*******************************************************************************/
void iic_stop(void)
{	
	IIC_SDA=0;//����Ѹ���������SCL���棬�ڶ��ζ�д�������
	delay_10us(1);
	IIC_SCL=1;
	delay_10us(1);
	IIC_SDA=1;	//��SCLΪ�ߵ�ƽʱ��SDA�ɵͱ�Ϊ
	delay_10us(1);			
}
/*******************************************************************************
*         : iic_ack
* ��������		   : ����ACKӦ��  
*            : 
*            : 
*******************************************************************************/
void iic_ack(void)
{
	IIC_SCL=0;
	IIC_SDA=0;	//SDAΪ�͵�ƽ
	delay_10us(1);
   	IIC_SCL=1;
	delay_10us(1);
	IIC_SCL=0;
}
/*******************************************************************************
*         : iic_nack
* ��������		   : ����NACK��Ӧ 
*            : 
*            : 
*******************************************************************************/
void iic_nack(void)
{
	IIC_SCL=0;
	IIC_SDA=1;	//SDAΪ�ߵ�ƽ
	delay_10us(1);
   	IIC_SCL=1;
	delay_10us(1);
	IIC_SCL=0;	
}
/*******************************************************************************
*         : iic_wait_ack
* ��������		   : �ȴ�Ӧ���źŵ���   
*            : 
*            : 1������Ӧ��ʧ
        			 0������Ӧ���
*******************************************************************************/
uchar iic_wait_ack(void)
{
	uchar time_temp=0;
	
	IIC_SCL=1;
	delay_10us(1);
	while(IIC_SDA)	//�ȴ�SDAΪ�͵�ƽ
	{
		time_temp++;
		if(time_temp>100)//��ʱ��ǿ�ƽ���IICͨ��
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
* ��������		   : IIC��һ����
*            : dat������һ����
*            : 
*******************************************************************************/
void iic_write_byte(uchar dat)
{                        
    uchar i=0; 
	   	    
    IIC_SCL=0;
    for(i=0;i<8;i++)	//ѭ��8�ν����ֽڴ������ȴ����ٴ���
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
* ��������		   : IIC��һ����
*            : ack=1ʱ����ACK��ack=0������nACK 
*            : Ӧ����Ӧ��
*******************************************************************************/
uchar iic_read_byte(uchar ack)
{
	uchar i=0,receive=0;
   	
    for(i=0;i<8;i++ )	//ѭ��8�ν����ֽڶ������ȶ����ٴ���
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

/*****************************�Ż����IIC д����  ************/
// ������Ƶ�I2C��д�������Ժϲ�

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
	iic_write_byte(dev_addr);	   //��д����
	iic_wait_ack(); 
    iic_write_byte(read_addr); 	//��д��ַ  
	iic_wait_ack();	    
	iic_start();  	 	    	 
	iic_write_byte(dev_addr+1);        		//�������ģʽ	   
	iic_wait_ack();	 
    temp=iic_read_byte(0);	//��ȡ�ֽ�		   
    iic_stop();				//������ֹͣ��   
	return temp;			//���ض�ȡ����
}
/*********************�ж�T0��ȡ����**********************/
// ��ʼ���ж�
void INT0_Init() {
    IT0 = 1;    // �½��ش���
    EX0 = 1;    // ʹ��INT0�ж�
    EA = 1;     // �������ж�
}


// �жϷ�����
void INT0_ISR() interrupt 0 {
     key_code1 = ZLG7290_ReadKey(); // ��ȡ��ֵ�Ĵ���
}
//��������תΪ��Ӧ���ַ������ڴ��ڷ���
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

