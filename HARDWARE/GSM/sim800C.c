//ע�⣺����wifi�ʹ��ڶ�ռ���˽ϴ���ڴ�ռ䣬Ŀǰ�Ѿ������ܼ����ڴ��������������쳣��������Զ���뼸��Ȼ�����ص���Ƭ����
//tip�������ܲ�Ҫ�ں��������ϴ���ڴ���䣬����ֱ���õ�������ȫ�ֱ����ķ�ʽ����

//�� arduinoESP8266�ⲿ�ֺ�����C++��ֲ��C����������51��ARM��Cƽ̨����
//�Է���ֵ��ԭ����true or false ��Ϊ����int�� 0��ʾʧ�� ������ʾ�ɹ�������ԭ�򷵻�
#include <stdlib.h>
#include "serialportAPI.h"
#include "sim800C.h"
#include "stringAPIext.h"
#include "uart.h"
#include "string.h"
#include "stdio.h"

#define AT_DELAY 1000

volatile unsigned long sys_tick = 0;

char data_rec[RECV_BUF_SIZE];
uint8_t sim_csq = 0;
char sim_ip[15];
char sim_imei[16];
unsigned int sim_loc;
unsigned int sim_ci;

uint32_t recvPkg(uint8_t *buffer, uint32_t buffer_size, uint32_t *data_len, uint32_t timeout, uint8_t *coming_mux_id);
int eATUART(uint32_t baud);
int eATRST(void);

/******  ��ѯsimģ��Ĺ̼��汾��  ******/
int eATGSV(char * recv_buf);
/******  GPRSģ���ʼ������TCP  ******/
int sATIPR(unsigned int baudrate);
int eAT(void);
int qATCPIN(char *CODE);
int eATCSQ(uint8_t *csq);
int eATCSQ_TRANS(uint8_t *csq);
int qATCREG0(uint8_t *n, uint8_t *stat);
int qATCGATT(uint8_t *attach);
int sATCIPMODE(uint8_t mode);
int sATCSTT(char *apn);
int eATCIICR(void);
int eATCIFSR(char * ip_addr);
int sATCIPHEAD(uint8_t mode);
int eATCIPCLOSE(void);
/******  ��վ��λAPI��Ҫ������  ******/
int eATGSN(char * imei);
int sATCREG(uint8_t n);
int qATCREG2(unsigned int *lac, unsigned int *ci);
unsigned int a16toi(char * s);
/******  ��ʼ��HTTP������ȡ��վ��λ������  ******/
int sATSAPBR1(uint8_t cmd_type, uint8_t cid, char * tag, char * value);
int sATSAPBR2(uint8_t cmd_type, uint8_t cid);
int eATHTTPINIT(void);
int eATHTTPTERM(void);
int sATHTTPPARA(char * tag, char * value);
int sATHTTPACTION(uint8_t method, char * status);
int qATHTTPREAD(char * buf);
int sATCPOWD(uint8_t mode);
/******  �ļ�ϵͳ����  ******/
int sATFSDRIVE(u8 n);
int sATFSLS(char * filepath, char * list_file);
int sATFSCREATE(char * filename);
int sATFSDEL(char * filename);
int sATFSFLSIZE(char * filename, u32 * size);
int sATFSWRITE(char * filename, u8 mode, int size, int timeout, const char *buffer);
int sATFSREAD(char * filename, u8 mode, int size, int position, char * read_buf);
/******  SSL����  ******/
int qATSSLSETCERT(char * read_buf);
int sATSSLSETCERT(char * file_name, char * pass_word);
int qATCIPSSL(u8 *n);
int sATCIPSSL(u8 n);
/******  ͸��ģʽ�л���������ģʽ  ******/
int eEXIT_TRANS(void);
int eATO(void);
/******  ����  ******/
int sATD(int number);
int sATS0(int number);

int eATCWSTARTSMART(uint8_t type, char *link_msg);
int eATCWSTOPSMART(void);
int qATCWMODE(uint8_t *mode);
int sATCWMODE(uint8_t mode);
int sATCIPMUX(uint8_t mode);
int sATCWAUTOCONN(uint8_t mode);

int recvFindAndFilter(const char *target, const char *begin, const char *end, char *data_rec, uint32_t timeout);
int recvFind(const char *target, uint32_t timeout);
int recvString(char *rec_data, const char *target, uint32_t timeout);
int recvString2(char *rec_data, const char *target1, const char *target2, uint32_t timeout);
int eATCIPSTATUS(const char * status);
int sATCIPSTARTSingle(const char *type, const char *addr, uint32_t port);
int sATCIPSENDSingle(const uint8_t *buffer, uint32_t len);
int recvString3(char *rec_data, const char *target1, const char *target2, const char *target3, uint32_t timeout);

/*******************************************************************************
  �� �� �� ��AutoLink
  �������� ���Զ����ӣ�ǰ10s�Զ����ӣ�������ʧ�������smartlinkģʽ30s������Ȼʧ��
             ���ٴλص��Զ����ӣ�ֱ�����ӳɹ�
  ��    �� ����
  ��    �� ����
*******************************************************************************/
void AutoLink(void)
{
  int status = STATUS_LOSTIP;
  while (status != STATUS_GETIP)
  {
    uint32_t start_time = millis();
    printf("start auto link");
    //10s�Զ�����ʱ��
    while ((millis() - start_time < 10000) && status != STATUS_GETIP)
    {
      status = getSystemStatus();
      delay(1000);
    }

    //����ʧ�ܽ���smartlinkģʽ 30s
    if (status != STATUS_GETIP)
    {
      char link_msg[RECV_BUF_SIZE];
      printf("start smartlink");
      stopSmartLink();

      if (1 == smartLink((uint8_t)ESP_AIR, link_msg))
      {
        stopSmartLink(); //���������Ƿ�ɹ�������Ҫ�ͷſ�����ռ���ڴ�
        printf(link_msg);
        start_time = millis();//�ȴ���ȡIP
        while ((millis() - start_time < 5000) && status != STATUS_GETIP)
        {
          status = getSystemStatus();
          delay(500);
        }
        sATCWAUTOCONN(1); //�����Զ�����ģʽ
      }
      else
      {
        stopSmartLink();
        delay(500);
        printf("link AP fail");
        //restart();
      }
    }
  }
  printf("link AP OK");
  //sATCWAUTOCONN(0); //�����Զ�����ģʽ
}

int GSMInit(const char *addr, uint32_t port, char *http_data)
{	
	
	VBAT = 0;
	//open the GSM
	delay(1000);
	//GPIO_SetBits(GPIOB, GPIO_Pin_0);
	POWERKEY = 1;
	delay(1000);
	//GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	POWERKEY = 0;
	delay(1000);
	
	//if(!sATIPR(115200)) goto RESTART; //���ò�����
	if(!ConectTest())	return 0;
	if(!CheckState()) return 0;
	//if(!FSInit()) return 0;
	//if(!LSB_API_data()) return 0;
	//if(!HttpInit(http_data)) return 0;
	
	//if(!TCPInit(addr, port)) return 0;
	//ClearRxBuf(); //������������л��������
	return 1;
}

int ConectTest(void){
	unsigned char i = 0;
	for(i = 0; i < 40; i++){
		if(!eAT()) delay(AT_DELAY);
		else break;
	}
	if(i >= 40) return 0;
	printf("AT is OK!\r\n");
	if(recvFind("SMS Ready", 5000)) printf("SMS Ready!\r\n"); //�ȴ�30S
//	if(!recvFind("SMS Ready", 30000)) return 0; //�ȴ�30S
//	printf("SMS Ready!\n");
//	delay(AT_DELAY);
	
	return 1;
}

int CheckState(void)
{
	unsigned char i=0;
	char CODE[10];
	uint8_t CREG_n, CREG_stat, attach;
	
	for(i=0; i<30; i++){
		if(!qATCPIN(CODE)) return 0;
		if(strcmp(CODE,"READY") == 0) break;
		else delay(AT_DELAY);
	}
	if(i >= 30) return 0;
	printf("CPIN=READY!\r\n");
	delay(AT_DELAY);
	
	for(i = 0; i < 30; i++)
	{
		if(!eATCSQ(&sim_csq)) return 0;
		if(sim_csq != 0) break;
		else delay(AT_DELAY);
	}
	if(i >= 30) return 0;
	printf("csq = %d\r\n", sim_csq);
	delay(AT_DELAY);
	
	for(i=0; i<30; i++){
		if(!qATCREG0(&CREG_n, &CREG_stat)) return 0;
		if(CREG_stat == 1 || CREG_stat == 5) break;
		else delay(AT_DELAY);
	}
	if(i >= 30) return 0;
	printf("CREG_stat=1!\r\n");
	delay(AT_DELAY);
	
	for(i=0; i<30; i++){
		if(!qATCGATT(&attach)) return 0;
		if(attach == 1) break;
		else delay(AT_DELAY);
	}
	if(i >= 30) return 0;
	printf("CGATT = 1!\r\n");
	
#ifdef RF_TEST
	delay(5000);
//	if(!sATD(112)) return 0;
//	printf("sATD(112) is OK");
	if(!sATS0(1)) return 0;
	printf("sATS0(1) is OK!\r\n");
	while(1);
#endif
	
#ifdef TRANS_MODE
	if(!sATCIPMODE(1)) return 0;
	printf("sATCIPMODE = 1 is OK!\r\n");
#endif
	if(!sATCSTT("CMNET")) return 0;
	printf("CSTT = CMNET is OK!\r\n");
	if(!eATCIICR()) return 0;
	printf("CIICR is OK!\r\n");
	if(!eATCIFSR(sim_ip)) return 0;
	printf("sim_ip = %s\r\n", sim_ip);
	return 1;
}

int TCPInit(const char *addr, uint32_t port)
{
#ifdef SSL_MODE
	if(!sATCIPSSL(1)) return 0;
	printf("CIPSSL = 1 OK\r\n");
#endif
	if(!sATCIPHEAD(1)) return 0;
	printf("CIPHEAD = 1 OK\r\n");
	if(!createTCP(addr, port)) return 0;
	printf("create tcp ok! server_ip = %s, port = %d\r\n", addr, port);
#ifndef TRANS_MODE
	if(!eATCIPSTATUS("CONNECT OK")) return 0;
	printf("STATUS: CONNECT OK!\r\n");
#endif
	
	return 1;
}

int LSB_API_data(void)
{
	if(!eATGSN(sim_imei)) return 0;
	printf("sim_imei = %s\r\n", sim_imei);
	if(!sATCREG(2)) return 0;
	printf("AT+CREG = 2 is OK!\r\n");
	if(!qATCREG2(&sim_loc, &sim_ci)) return 0;
	printf("sim_loc = %d, sim_ci = %d\r\n", sim_loc, sim_ci);
	if(!sATCREG(0)) return 0;
	printf("AT+CREG = 0 is OK!\r\n");
		
	return 1;
}

int HttpInit(char * http_data)
{
	char * http_action_status;
	char http_send_data[SEND_BUF_SIZE];
	char http_recv_data[RECV_BUF_SIZE];
	
	/*http init*/
	if(!sATSAPBR1(3,1,"CONTYPE","GPRS")) return 0;
	printf("http 1 OK!\r\n");
	if(!sATSAPBR1(3,1,"APN","CMNET")) return 0;
	printf("http 2 OK!\r\n");
	if(!sATSAPBR2(1,1)) return 0;
	printf("http 3 OK!\r\n");
	if(!sATSAPBR2(2,1)) return 0;
	printf("http 4 OK!\r\n");
	if(!eATHTTPINIT()) return 0;
	printf("http 5 OK!\r\n");
	if(!sATHTTPPARA("CID","1")) return 0;
	printf("http 6 OK!\r\n");
	sprintf(http_send_data, "http://apilocate.amap.com/position?accesstype=0&imei=%s&cdma=0&bts=460,00,%d,%d,%d&serverip=%s&output=josn&key=%s",sim_imei,sim_loc,sim_ci,(sim_csq*2-113),sim_ip,GAODE_API_KEY);
	printf("%s\r\n", http_send_data);
	if(!sATHTTPPARA("URL",http_send_data)) return 0;
	//if(!sATHTTPPARA("URL","http://apilocate.amap.com/position?accesstype=0&imei=862631037454138&cdma=0&bts=460,00,4566,6758,-101&nearbts=460,00,4566,8458,-110|460,00,4566,31248,-107&serverip=10.76.18.147&output=josn&key=9eced1b11c8c7ffd5447eb0ba28748d8")) return 0;
	printf("http 7 OK!\r\n");
	if(!sATHTTPACTION(0, http_action_status)) return 0;
	printf("http 8 OK!\r\n");
	if(!qATHTTPREAD(http_recv_data)) return 0;
	printf("{%s\r\n", http_recv_data);
	printf("http 9 OK!\r\n");
	if(!eATHTTPTERM()) return 0;
	printf("http 10 OK!\r\n");
	
	return 1;
}

int FtpInit(const char *addr)
{
	
	/*http init*/
	if(!sATSAPBR1(3,1,"CONTYPE","GPRS")) return 0;
	printf("http 1 OK!\r\n");
	if(!sATSAPBR1(3,1,"APN","CMNET")) return 0;
	printf("http 2 OK!\r\n");
	if(!sATSAPBR2(1,1)) return 0;
	printf("http 3 OK!\r\n");
	
	
	return 1;
}


void ClearRecBuf(void)
{
	//memset(data_rec, '\0', sizeof(data_rec));
  data_rec[0] = '\0';
}

//��1ms��ʱ�����ã��ö�ʱ����Ҫ�нϸߵ����ȼ�
void timer1msINT(void)
{
  sys_tick++;
}

unsigned long millis(void)
{
  return sys_tick;
}

//ms ��ʱ���������� msֵ > 10
void delay(unsigned int ms)
{
  unsigned long start = millis();
  while(millis() - start <= ms);
}

int SetBaud(uint32_t baud)
{
  return eATUART(baud);
}

void GSM_restart(void)
{
	if(sATCPOWD(1)) printf("GSM restart!\r\n");
	else printf("GSM restart FAILED!\r\n");
	VBAT = 1;
	delay(1000);
	VBAT = 0;
}

//�л���AP+stationģʽ
int setOprToStationSoftAP(void)
{
//  uint8_t mode = 0xff;
//  if (!qATCWMODE(&mode))
//  {
//    return 0;
//  }
//  if (mode == 1)
//  {
//    return 1;
//  }
//  else
//  {
//    if (sATCWMODE(1) && restart())
//    {
//      return 1;
//    }
//    else
//    {
//      return 0;
//    }
//  }
	return 1;
}

//ע��link_msg�Ĵ�СҪ�㹻����ֹ�ڴ����,����SSID��pwd��Ϣ
int smartLink(uint8_t  type, char *link_msg)
{
  return eATCWSTARTSMART(type, link_msg);
}

int stopSmartLink(void)
{
  return eATCWSTOPSMART();
}

int getSystemStatus(void)
{
  if(!eATCIPSTATUS("CONNECT OK")) return 0;
	else return 1;
}

int disableMUX(void)
{
  return sATCIPMUX(0);
}

int createTCP(const char *addr, uint32_t port)
{
  return sATCIPSTARTSingle("TCP", addr, port);
}

void closeTCP(void)
{
	if(eATCIPCLOSE()) printf("close TCP is OK!\r\n");
	else printf("close TCP is FAILED!\r\n");
}

/* +IPD,<id>,<len>:<data> */
/* +IPD,<len>:<data> */

uint32_t recvPkg(uint8_t *buffer, uint32_t buffer_size, uint32_t *data_len, uint32_t timeout, uint8_t *coming_mux_id)
{
#ifdef TRANS_MODE
	unsigned long start;
	int ret = 0;
	char a;
	
	start = millis();
	while (millis() - start < timeout)
  {
		if(SerialAvailable() > 0)
    {
      a = SerialRead();
      buffer[ret++] = a;
    }
	}
	
	return ret;
#else
  char a;
  int32_t index_PIPDcomma = 0;
  int32_t index_colon = 0; /* : */
  int32_t index_comma = 0; /* , */
  int32_t len = 0;
  int8_t id = 0;
  int has_data = 0;
  uint32_t ret;
  unsigned long start;
  uint32_t i;
  ClearRecBuf();
  if (buffer == NULL)
  {
    return 0;
  }

  start = millis();
  while (millis() - start < timeout)
  {
    if(SerialAvailable() > 0)
    {
      a = SerialRead();
      StringAddchar(data_rec, a);
    }

    index_PIPDcomma = StringIndex(data_rec, "+IPD,");
    if (index_PIPDcomma != -1)
    {
      index_colon = StringIndexCharOffset(data_rec, ':', index_PIPDcomma + 5);
      if (index_colon != -1)
      {
        index_comma = StringIndexCharOffset(data_rec, ',', index_PIPDcomma + 5);
        /* +IPD,id,len:data */
        if (index_comma != -1 && index_comma < index_colon)
        {
          char str_temp[5];
          StringSubstring(str_temp, data_rec, index_PIPDcomma + 5, index_comma);
          id = atoi(str_temp);
          if (id < 0 || id > 4)
          {
            return 0;
          }
          StringSubstring(str_temp, data_rec, index_comma + 1, index_colon);
          len = atoi(str_temp);
          if (len <= 0)
          {
            return 0;
          }
        }
        else     /* +IPD,len:data */
        {
          char str_temp[5];
          StringSubstring(str_temp, data_rec, index_PIPDcomma + 5, index_colon);
          len = atoi(str_temp);
          if (len <= 0)
          {
            return 0;
          }
        }
        has_data = 1;
        break;
      }
    }
  }

  if (has_data)
  {
    i = 0;
    ret = len > buffer_size ? buffer_size : len;
    start = millis();
    while (millis() - start < 3000)
    {
      while(SerialAvailable() > 0 && i < ret)
      {
        a = SerialRead();
        buffer[i++] = a;
      }
      if (i == ret)
      {
        rx_empty();
        if (data_len)
        {
          *data_len = len;
        }
        if (index_comma != 0 && coming_mux_id)
        {
          *coming_mux_id = id;
        }
        return ret;
      }
    }
  }
  return 0;
#endif
}

void rx_empty(void)
{
//    while(SerialAvailable() > 0) {
//        SerialRead();
//    }
  ClearRxBuf();
}

int eATUART(uint32_t baud)
{
  int int_baud = baud;
  rx_empty();
  SerialPrint("AT+UART=", STRING_TYPE);
  SerialPrint(&int_baud, INT_TYPE);
  SerialPrintln(",8,1,0,0", STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

int eATRST(void)
{
  rx_empty();
  SerialPrintln("AT+RST", STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

/******  ��ѯsimģ��Ĺ̼��汾��  ******/
int eATGSV(char * recv_buf)
{
	char buf[SEND_BUF_SIZE];
	int ret;
	
	rx_empty();
	sprintf(buf, "AT+GSV");
	SerialPrintln(buf, STRING_TYPE);
	ret = recvFindAndFilter("\r\nOK", buf, "\r\nOK", recv_buf, TIME_OUT);
	if(ret != 1) return 0;
	return 1;
}

/******  GPRSģ���ʼ������TCP  ******/
int sATIPR(unsigned int baudrate)
{
  int int_baudrate = baudrate;
  rx_empty();
  SerialPrint("AT+IPR=", STRING_TYPE);
  SerialPrintln(&int_baudrate, INT_TYPE);
  return recvFind("OK", TIME_OUT);
}

int eAT(void)
{
  rx_empty();
  SerialPrintln("AT", STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

int qATCPIN(char *CODE)
{
  char str_CODE[10];
  int ret;
  if (!CODE)
  {
    return 0;
  }
  rx_empty();
  SerialPrintln("AT+CPIN?", STRING_TYPE);

  ret = recvFindAndFilter("OK", "+CPIN: ", "\r\n\r\nOK", str_CODE, TIME_OUT);
  if (ret != 0)
  {
		strcpy(CODE,str_CODE);
    //CODE = str_CODE;
    return 1;
  }
  return 0;
}

int eATCSQ(uint8_t *csq)
{
  char str_csq[5];
  int ret;
  if (!csq)
  {
    return 0;
  }
  rx_empty();
  SerialPrintln("AT+CSQ", STRING_TYPE);

  ret = recvFindAndFilter("OK", "+CSQ: ", ",", str_csq, TIME_OUT);
  if (ret != 0)
  {
    *csq = (uint8_t)atoi(str_csq);
    return 1;
  }
  return 0;
}

int eATCSQ_TRANS(uint8_t *csq)
{
	if(!eEXIT_TRANS()) return 0;
	printf("eEXIT_TRANS is OK!\r\n");
	if(!eATCSQ(csq)) return 0;
	if(!eATO()) return 0;
	printf("eATO is OK!\r\n");
	return 1;
}

int qATCREG0(uint8_t *n, uint8_t *stat)
{

	char str_stat[2];
  int ret_stat;
  if (!n || !stat)
  {
    return 0;
  }
  rx_empty();
  SerialPrintln("AT+CREG?", STRING_TYPE);

	ret_stat = recvFindAndFilter("OK", ",", "\r\n\r\nOK", str_stat, TIME_OUT);
  if (ret_stat != 0)
  {
    *n = 0;
		*stat = (uint8_t)atoi(str_stat);
    return 1;
  }
  return 0;
}

int qATCGATT(uint8_t *attach)
{
  char str_attach[5];
  int ret;
  if (!attach)
  {
    return 0;
  }
  rx_empty();
  SerialPrintln("AT+CGATT?", STRING_TYPE);

  ret = recvFindAndFilter("OK", "+CGATT: ", "\r\n\r\nOK", str_attach, TIME_OUT);
  if (ret != 0)
  {
    *attach = (uint8_t)atoi(str_attach);
    return 1;
  }
  return 0;
}

int sATCIPMODE(uint8_t mode)
{
  int int_mode = mode;
  rx_empty();
  SerialPrint("AT+CIPMODE=", STRING_TYPE);
  SerialPrintln(&int_mode, INT_TYPE);
  return recvFind("OK", TIME_OUT);
}

int sATCSTT(char *apn)
{
  //int int_apn = apn;
  rx_empty();
  SerialPrint("AT+CSTT=\"", STRING_TYPE);
  SerialPrint(apn, STRING_TYPE);
	SerialPrintln("\"", STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

int eATCIICR(void)
{
  rx_empty();
  SerialPrintln("AT+CIICR", STRING_TYPE);
  return recvFind("OK", 10000);
}

int eATCIFSR(char * ip_addr)
{
	int32_t index1;
	int32_t index2;
  rx_empty();
  SerialPrintln("AT+CIFSR", STRING_TYPE);
	recvString(data_rec, "*", TIME_OUT);
	index1 = StringIndex(data_rec, "AT+CIFSR");
	if (index1 != -1)
	{
		index1 += StringLenth("AT+CIFSR") + 3;	//����������0xd��һ��0xa,���+3
		index2 = strlen(data_rec) - 3;  //��Ϊ��ĩβ��\n\r�����ַ����ڣ����Ҫ��3
		StringSubstring(ip_addr, data_rec, index1, index2);
		return 1;
	}
  return 0;
}

int sATCIPHEAD(uint8_t mode)
{
  int int_mode = mode;
  rx_empty();
  SerialPrint("AT+CIPHEAD=", STRING_TYPE);
  SerialPrintln(&int_mode, INT_TYPE);
  return recvFind("OK", TIME_OUT);
}

int eATCIPCLOSE(void)
{
	rx_empty();
  SerialPrintln("AT+CIPCLOSE", STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

int eATGSN(char * imei)
{
  int ret;
  if (!imei)
  {
    return 0;
  }
  rx_empty();
  SerialPrintln("AT+GSN", STRING_TYPE);

  ret = recvFindAndFilter("OK", "AT+GSN\r\r\n", "\r\n\r\nOK", imei, TIME_OUT);
  if (ret != 1) return 0;
  return 1;
}

int sATCREG(uint8_t n)
{
	char buf[32];
  rx_empty();
	sprintf(buf,"AT+CREG=%d",n);
	SerialPrintln(buf, STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

int qATCREG2(unsigned int *lac, unsigned int *ci)
{
	int ret;
  char str_lac_ci[16];
	char str_lac[5];
	char str_ci[5];
	
  if (!lac || !ci)
  {
    return 0;
  }
  rx_empty();
  SerialPrintln("AT+CREG?", STRING_TYPE);

  ret = recvFindAndFilter("OK", "+CREG: 2,1,", "\r\n\r\nOK", str_lac_ci, TIME_OUT);
  if (ret != 0)
  {
		int32_t index1 = 1;
		int32_t index2 = StringIndex(str_lac_ci, "\",\"") - 1;
    int32_t index3 = index2 + 4;
		int32_t index4 = strlen(str_lac_ci)-2;
    if (index2 != -1 && index4 != -1)
    {
      StringSubstring(str_lac, str_lac_ci, index1, index2);
			StringSubstring(str_ci, str_lac_ci, index3, index4);
			*lac = a16toi(str_lac);
			*ci = a16toi(str_ci);
			return 1;
		}
  }
  return 0;
}

unsigned int a16toi(char * s)
{
	int i =0;
	unsigned int a=0;
	unsigned int b=0;
	for(i=0;s[i]!='\0';i++)
	{
		if(s[i]>='a'&&s[i]<='f')b=s[i]-'a'+10;
		if(s[i]>='A'&&s[i]<='F')b=s[i]-'A'+10;
		if(s[i]>='0'&&s[i]<='9')b=s[i]-'0';
		a=a*16+b;
	}
	return a;
}

int sATSAPBR1(uint8_t cmd_type, uint8_t cid, char * tag, char * value)
{
	char buf[32];
  rx_empty();
	sprintf(buf,"AT+SAPBR=%d,%d,\"%s\",\"%s\"",cmd_type,cid,tag,value);
	SerialPrintln(buf, STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

int sATSAPBR2(uint8_t cmd_type, uint8_t cid)
{
	char buf[32];
  rx_empty();
	sprintf(buf,"AT+SAPBR=%d,%d",cmd_type,cid);
	SerialPrintln(buf, STRING_TYPE);
  return recvFind("OK", 5000);
}

int eATHTTPINIT(void)
{
	rx_empty();
  SerialPrintln("AT+HTTPINIT", STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

int eATHTTPTERM(void)
{
	rx_empty();
  SerialPrintln("AT+HTTPTERM", STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

int sATHTTPPARA(char * tag, char * value)
{
	char buf[SEND_BUF_SIZE];
  rx_empty();
	sprintf(buf,"AT+HTTPPARA=\"%s\",\"%s\"", tag, value);
	SerialPrintln(buf, STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

int sATHTTPACTION(uint8_t method, char * status)
{
	char buf[32];
	//int index1, index2;
	
  rx_empty();
	sprintf(buf,"AT+HTTPACTION=%d",method);
	SerialPrintln(buf, STRING_TYPE);
  //if(!recvFind("OK", TIME_OUT)) return 0;
	if(!recvFind("200", 10000)) return 0;
	return 1;
}

int qATHTTPREAD(char * buf)
{
  int ret;
  if (!buf)
  {
    return 0;
  }
  rx_empty();
  SerialPrintln("AT+HTTPREAD", STRING_TYPE);

  ret = recvFindAndFilter("\r\nOK", "{", "\r\nOK", buf, TIME_OUT);
  if (ret != 1) return 0;
  return 1;
}

int sATCPOWD(uint8_t mode)
{
  int int_mode = mode;
  rx_empty();
  SerialPrint("AT+CPOWD=", STRING_TYPE);
  SerialPrintln(&int_mode, INT_TYPE);
  return recvFind("NORMAL POWER DOWN", 5000);
}

/******  �ļ�ϵͳ����  ******/
int sATFSDRIVE(u8 n)
{
	char buf[SEND_BUF_SIZE];
  rx_empty();
	sprintf(buf, "AT+FSDRIVE=%d", n);
	SerialPrintln(buf, STRING_TYPE);
	if(n == 0) return recvFind("+FSDRIVE: C\r\n\r\nOK", TIME_OUT);
	else return 0;
}

int sATFSLS(char * filepath, char * list_file)
{
	int ret;
	char buf[SEND_BUF_SIZE];
	
	if (filepath == NULL)
	{
		return 0;
	}
	
  rx_empty();
	sprintf(buf, "AT+FSLS=%s", filepath);
	SerialPrintln(buf, STRING_TYPE);
	sprintf(buf, "%s\r\r\n", buf);
	ret = recvFindAndFilter("\r\nOK", buf, "\r\n\r\nOK", list_file, TIME_OUT);
	if(ret != 1) return 0;
	return 1;
}

int sATFSCREATE(char * filename)
{
	char buf[SEND_BUF_SIZE];
	
	if (filename == NULL)
	{
		return 0;
	}
	
  rx_empty();
	sprintf(buf, "AT+FSCREATE=%s", filename);
	SerialPrintln(buf, STRING_TYPE);
	return recvFind("\r\nOK", TIME_OUT);
}

int sATFSDEL(char * filename)
{
	char buf[SEND_BUF_SIZE];
	
	if (filename == NULL)
	{
		return 0;
	}
	
  rx_empty();
	sprintf(buf, "AT+FSDEL=%s", filename);
	SerialPrintln(buf, STRING_TYPE);
	return recvFind("\r\nOK", TIME_OUT);
}

int sATFSFLSIZE(char * filename, u32 * size)
{
	int ret;
	char buf[SEND_BUF_SIZE];
	
	if (filename == NULL)
	{
		return 0;
	}
	
  rx_empty();
	sprintf(buf, "AT+FSFLSIZE=%s", filename);
	SerialPrintln(buf, STRING_TYPE);
	ret = recvFindAndFilter("\r\nOK", "+FSFLSIZE: ", "\r\n\r\nOK", buf, TIME_OUT);	
	if(ret != 1) return 0;
	*size = atoi(buf);
	return 1;
}

int sATFSWRITE(char * filename, u8 mode, int size, int timeout, const char *buffer)
{
	int i;
	char buf[SEND_BUF_SIZE];
	
	rx_empty();
	sprintf(buf, "AT+FSWRITE=%s, %d, %d, %d", filename, mode, size, timeout);
	SerialPrintln(buf, STRING_TYPE);
	if (-1 != recvFind(">", 1000))  //5000
  {
    rx_empty();
    for (i = 0; i < size; i++)
    {
      SerialWrite(buffer[i]);
    }
    return recvFind("OK", 3000);//10000
  }
	return 0;
}

int sATFSREAD(char * filename, u8 mode, int size, int position, char * read_buf)
{
	char buf[SEND_BUF_SIZE];
	int ret;
	
	rx_empty();
	sprintf(buf, "AT+FSREAD=%s, %d, %d, %d", filename, mode, size, position);
	SerialPrintln(buf, STRING_TYPE);
	ret = recvFindAndFilter("\r\nOK", "\r\r\n", "\r\nOK", read_buf, TIME_OUT);	
	if(ret != 1) return 0;
	return 1;
}
/******  �ļ�ϵͳ����  ******/

/******  SSL����  ******/
int qATSSLSETCERT(char * read_buf)
{
	char buf[SEND_BUF_SIZE];
	int ret;
	
	rx_empty();
	sprintf(buf, "AT+SSLSETCERT=?");
	SerialPrintln(buf, STRING_TYPE);
	ret = recvFindAndFilter("\r\nOK", buf, "\r\nOK", read_buf, TIME_OUT);
	if(ret != 1) return 0;
	return 1;
}

int sATSSLSETCERT(char * file_name, char * pass_word)
{
	char buf[SEND_BUF_SIZE];
	
	rx_empty();
	if(pass_word == NULL)	sprintf(buf, "AT+SSLSETCERT=\"%s\"", file_name);
	else sprintf(buf, "AT+SSLSETCERT=\"%s\", \"%s\"", file_name, pass_word);
	SerialPrintln(buf, STRING_TYPE);
	return recvFind("+SSLSETCERT: 0", TIME_OUT);
}

int qATCIPSSL(u8 *n)
{
	char buf[SEND_BUF_SIZE];
	int ret;
	
	rx_empty();
	sprintf(buf, "AT+CIPSSL?");
	SerialPrintln(buf, STRING_TYPE);
	ret = recvFindAndFilter("\r\nOK", "+CIPSSL: ", "\r\n\r\nOK", buf, TIME_OUT);
	if(ret != 1) return 0;
	*n = atoi(buf);
	return 1;
}

int sATCIPSSL(u8 n)
{
	char buf[SEND_BUF_SIZE];
	
	rx_empty();
	sprintf(buf, "AT+CIPSSL=%d", n);
	SerialPrintln(buf, STRING_TYPE);
	return recvFind("\r\nOK", TIME_OUT);
}
/******  SSL����  ******/

/******  ͸��ģʽ�л���������ģʽ  ******/
int eEXIT_TRANS(void)
{
	rx_empty();
	SerialPrint("+++", STRING_TYPE);
	return recvFind("OK\r\n", 1000);
}

int eATO(void)
{
	rx_empty();
	SerialPrintln("ATO", STRING_TYPE);
	return recvFind("CONNECT\r\n", TIME_OUT);
}
/******  ͸��ģʽ�л���������ģʽ  ******/

/******  ����  ******/
int sATD(int number)
{
	char buf[SEND_BUF_SIZE];
	rx_empty();
	sprintf(buf, "ATD%d;", number);
	SerialPrintln(buf, STRING_TYPE);
	return recvFind("OK\r\n", 1000);
}

int sATS0(int number)
{
	char buf[SEND_BUF_SIZE];
	rx_empty();
	sprintf(buf, "ATS0=%d", number);
	SerialPrintln(buf, STRING_TYPE);
	return recvFind("OK\r\n", 1000);
}
/******  ����  ******/
	

/*******************************************************************************
* �� �� �� ��eATCWSTARTSMART
* �������� ������smartlinkģʽ����Ҫ��30s������//add by LC 2016.01.05 16:27
* ��    �� ��type ������ʽ 0 -AL-LINK    1 - ESP-TOUCH    2 - AIR-KISS
			link_msg ���ص�SSID��PSD
* ��    �� ����
*******************************************************************************/
int eATCWSTARTSMART(uint8_t type, char *link_msg)
{
  int flag;
  int int_type = type;
  rx_empty();
  SerialPrint("AT+CWSTARTSMART=", STRING_TYPE);
  SerialPrintln(&int_type, INT_TYPE);
  flag = recvFind("OK", TIME_OUT);
  printf("AT+CWSTARTSMART=3 is OK!\n");
  if(flag == 0) return flag;
  delay(50);//��ʱ֮��ȴ��Զ�����
  rx_empty();
  //return recvFindAndFilter("OK", "SMART SUCCESS", "\r\n\r\nOK", link_msg,30000);
  return recvFindAndFilter("smartconfig connected wifi", "Smart get wifi info", "WIFI CONNECTED", link_msg, 60000);
}

//add by LC 2016.01.05 16:27
int eATCWSTOPSMART(void)
{
  rx_empty();
  SerialPrintln("AT+CWSTOPSMART", STRING_TYPE);
  return recvFind("OK", TIME_OUT);
}

int qATCWMODE(uint8_t *mode)
{
  char str_mode[5];
  int ret;
  if (!mode)
  {
    return 0;
  }
  rx_empty();
  SerialPrintln("AT+CWMODE?", STRING_TYPE);

  ret = recvFindAndFilter("OK", "+CWMODE:", "\r\n\r\nOK", str_mode, TIME_OUT);
  if (ret != 0)
  {
    *mode = (uint8_t)atoi(str_mode);
    return 1;
  }
  else
  {
    return 0;
  }
}

int sATCWMODE(uint8_t mode)
{
  int int_mode = mode;
  ClearRecBuf();
  rx_empty();
  SerialPrint("AT+CWMODE=", STRING_TYPE);
  SerialPrintln(&int_mode, INT_TYPE);

  recvString2(data_rec, "OK", "no change", TIME_OUT);
  if (StringIndex(data_rec, "OK") != -1 || StringIndex(data_rec, "no change") != -1)
  {
    return 0;
  }
  return 1;
}

int sATCWAUTOCONN(uint8_t mode)
{
  int int_mode = mode;
  rx_empty();
  SerialPrint("AT+CWAUTOCONN=", STRING_TYPE);
  SerialPrintln(&int_mode, INT_TYPE);
  return recvFind("OK", TIME_OUT);
}

int eATCIPSTATUS(const char * status)
{
  //delay(100);//ȥ����ʱ modfied by LC 2016.01.06 23:46
  rx_empty();
  SerialPrintln("AT+CIPSTATUS", STRING_TYPE);
  return recvFind(status, TIME_OUT);
}

//�ڽ��յ��ַ����в��� target ����ȡ begin �� end �м���Ӵ�д��data
int recvFindAndFilter(const char *target, const char *begin, const char *end, char *data_get, uint32_t timeout)
{
  ClearRecBuf();
  recvString(data_rec, target, timeout);
  if (StringIndex(data_rec, target) != -1)
  {
    int32_t index1 = StringIndex(data_rec, begin);
    int32_t index2 = StringIndex(data_rec, end);
    if (index1 != -1 && index2 != -1)
    {
      index1 += StringLenth(begin);
      StringSubstring(data_get, data_rec, index1, index2 - 1);
      return 1;
    }
  }
  ClearRecBuf();
  return 0;
}

int sATCIPMUX(uint8_t mode)
{
  int int_mode = mode;

  rx_empty();
  SerialPrint("AT+CIPMUX=", STRING_TYPE);
  SerialPrintln(&int_mode, INT_TYPE);
  ClearRecBuf();
  recvString2(data_rec, "OK", "Link is builded", TIME_OUT);
  if (StringIndex(data_rec, "OK") != -1)
  {
    return 1;
  }
  return 0;
}

int sATCIPSTARTSingle(const char *type, const char *addr, uint32_t port)
{
  int int_port = port;
  ClearRecBuf();
  rx_empty();
  SerialPrint("AT+CIPSTART=\"", STRING_TYPE);
  SerialPrint(type, STRING_TYPE);
  SerialPrint("\",\"", STRING_TYPE);
  SerialPrint(addr, STRING_TYPE);
  SerialPrint("\",\"", STRING_TYPE);
  SerialPrint(&int_port, INT_TYPE);
	SerialPrintln("\"", STRING_TYPE);

//	ClearRecBuf();
//  //recvString3(data_rec, "OK", "ERROR", "CONNECT OK", 10000);
//	recvString(data_rec, "CONNECT OK", 10000);
//  if (StringIndex(data_rec, "CONNECT OK") != -1)
//  {
//    return 1;
//  }
#ifdef TRANS_MODE
	if(recvFind("CONNECT", 10000)) return 1;
#else
	if(recvFind("CONNECT OK", 10000)) return 1;
#endif
	//if(recvFind("CONNECT", 10000)) return 1;
  return 0;
}

int sATCIPSENDSingle(const uint8_t *buffer, uint32_t len)
{
#ifdef TRANS_MODE
	int i;
	for (i = 0; i < len; i++)
	{
		SerialWrite(buffer[i]);
	}
#else
  int i;
  int int_len = len;
  rx_empty();
  SerialPrint("AT+CIPSEND=", STRING_TYPE);
  SerialPrintln(&int_len, INT_TYPE);
  if (-1 != recvFind(">", 1000))  //5000
  {
    rx_empty();
    for (i = 0; i < len; i++)
    {
      SerialWrite(buffer[i]);
    }
    return recvFind("SEND OK", 3000);//10000
  }
#endif
  return 0;
}



int recvFind(const char *target, uint32_t timeout)
{
  ClearRecBuf();
  recvString(data_rec, target, timeout);
  if (StringIndex(data_rec, target) != -1)
  {
    return 1;
  }
  return 0;
}

//���ܣ��ڴ��ڽ��յ�����rec_data�������Ƿ���target�ַ�������ʱʱ��Ϊtimeout
int recvString(char *rec_data, const char *target, uint32_t timeout)
{
  char a;
  unsigned long start;
  start = millis();
  while (millis() - start < timeout)
  {
    while(SerialAvailable() > 0)
    {
      a = SerialRead();
      if(a == '\0') continue; //��Ϊ��StringAddchar()��������ĩβ���'\0'�Ĳ���
      StringAddchar(rec_data, a); //�����ڶ�����������ӵ�rec_data��
    }
    if (StringIndex(rec_data, target) != -1)  //������һ��target�Ƿ���rec_data��
    {
      break;
    }
  }
  return 1;
}

int recvString2(char *rec_data, const char *target1, const char *target2, uint32_t timeout)
{
  char a;
  unsigned long start = millis();
  while (millis() - start < timeout)
  {
    while(SerialAvailable() > 0)
    {
      a = SerialRead();
      if(a == '\0') continue;
      StringAddchar(rec_data, a);
    }
    if (StringIndex(rec_data, target1) != -1)
    {
      break;
    }
    else if (StringIndex(rec_data, target2) != -1)
    {
      break;
    }
  }
  return 1;
}

int recvString3(char *rec_data, const char *target1, const char *target2, const char *target3, uint32_t timeout)
{
  char a;
  unsigned long start = millis();
  while (millis() - start < timeout)
  {
    while(SerialAvailable() > 0)
    {
      a = SerialRead();
      if(a == '\0') continue;
      StringAddchar(rec_data, a);
    }
    if (StringIndex(rec_data, target1) != -1)
    {
      break;
    }
    else if (StringIndex(rec_data, target2) != -1)
    {
      break;
    }
    else if (StringIndex(rec_data, target3) != -1)
    {
      break;
    }
  }
  return 1;
}












