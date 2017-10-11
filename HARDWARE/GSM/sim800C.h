#ifndef __ESP8266_H__
#define __ESP8266_H__

#include "sys.h"

typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;

extern volatile unsigned long sys_tick;
extern uint8_t sim_csq;	//Ϊ���ϴ�csq���ݸ�������

//for gprs
//#define RF_TEST 1
#define TRANS_MODE 1
//#define SSL_MODE 1
#define HOST_NAME   "47.92.81.9"
#ifdef SSL_MODE
#define HOST_PORT   1883
#else
#define HOST_PORT   8883
#endif
#define MQTT_RECVBUF_SIZE 256
#define MQTT_SEND_SIZE 1024
#define POWERKEY PBout(0)
#define VBAT PAout(1)

#define SEND_BUF_SIZE  512
#define RECV_BUF_SIZE  512//������յĻ��棬�����ܵĴ󣬷�ֹ���
#define TIME_OUT 100

#define AI_LINK 0
#define ESP_TOUCH 1
#define AIR_LINK 2
#define ESP_AIR 3

#define STATUS_GETIP 2 //��ȡ��IP
#define STATUS_GETLINK 3 //��������
#define STATUS_LOSTLINK 4 //ʧȥ����
#define STATUS_LOSTIP 5 //δ��ȡ��IP

#define GAODE_API_KEY "9eced1b11c8c7ffd5447eb0ba28748d8"

//function
void AutoLink(void);
int GSMInit(const char *addr, uint32_t port, char *http_data);
int ConectTest(void);
int CheckState(void);
int TCPInit(const char *addr, uint32_t port);
int LSB_API_data(void);
int HttpInit(char * http_recv_data);
void timer1msINT(void);
unsigned long millis(void);
void delay(unsigned int ms);
int SetBaud(uint32_t baud);
void GSM_restart(void);
int setOprToStationSoftAP(void);
int smartLink(uint8_t  type,char *link_msg);
int stopSmartLink(void);
int getSystemStatus(void);
int disableMUX(void);
int createTCP(const char *addr, uint32_t port);
void closeTCP(void);
void rx_empty(void);

int eATCSQ(uint8_t *csq);	//Ϊ���ϴ�csq���ݸ�������
int eATCSQ_TRANS(uint8_t *csq);	//��͸��ģʽ��Ϊ���ϴ�csq���ݸ�������

#endif
