#include "string.h"

#define LED_ON digitalWrite(13,HIGH)
#define LED_OFF digitalWrite(13,LOW)
#define DE_HIGH digitalWrite(2,HIGH);
#define DE_LOW digitalWrite(2,LOW);

#define SLAVE_ADDRESS 0x04
#define READ_HOLDING_REGISTERS 0x03
#define SEND_BYTE_NUMBERS 54
#define READ_ADDRESS1 0x00
#define READ_ADDRESS2 0x03
#define DATA_LENGTH 0x1B

#define RX_MAX_SIZE 10
#define TX_MAX_SIZE 100

typedef enum
{
  SLAVE_ADD,
  FUNCTION_CODE,
  ADD_BYTE1,
  ADD_BYTE2,
  BLANK,
  DATA_LEN,
  CRC1,
  CRC2,
}CHECK_INPUT;


//rx related
uint8_t rxBuffer[20];
uint8_t rxIndex=0;
uint8_t rxChar;
bool rxError=false;
uint8_t rxComplete=0;
/* modbus related variables*/
unsigned int TxCrc;
uint8_t modbusTxBuff[100];
uint8_t uartTxBuffer[100];
uint8_t modbusTxbufferSize=0;

// ADC related variables
uint8_t adcResult[2];
char we;
String inString="";
float weight=0;

void setup() {
  // put your setup code here, to run once:
Serial1.begin(2400);
Serial2.begin(9600);
Serial.begin(9600);
pinMode(13,OUTPUT);
pinMode(2,OUTPUT);
DE_LOW;
}

void loop() {
  // put your main code here, to run repeatedly:
    LED_OFF;
    
    if(rxComplete==1)
    { 
      DE_HIGH;
      Modbus_TxRTU();
      DE_LOW;
    
        memcpy(rxBuffer,"",RX_MAX_SIZE);
        memcpy(modbusTxBuff,"",TX_MAX_SIZE);
        memcpy(uartTxBuffer,"",TX_MAX_SIZE);
        rxIndex=0;
        rxComplete=0;

    }
    
while(Serial1.available()>0)
    {
      we=Serial1.read();
      Serial.write(we);
      if(we !='\n')
      {
        inString+=(char)we;
        
        }else {
         
          String a=""; 
          a=inString.substring(1);
          weight=a.toFloat();
          if(inString[0]=='+')weight=weight*1;
          else if(inString[0]=='-')weight=weight*-1;
          Serial.print("weight: ");
          Serial.println(weight);
          uint16_t finalWeight=(uint16_t)(weight*10);
          adcResult[0]=(uint8_t)((finalWeight&0xFF00)>>8);
          adcResult[1]=(uint8_t)(finalWeight&0x00FF);
          inString="";
         // for(int j=2;j<27;j++)adcResult[j]=0;
          }
      
      }
   
  
    
}




void serialEvent2()
{
  while(Serial2.available()>0)
  {
    rxChar=(char)Serial2.read();
    
  switch(rxIndex)
  {
    case SLAVE_ADD:
      if(rxChar==SLAVE_ADDRESS){
        rxIndex=1;
        //debg=3;
      rxBuffer[0]=rxChar;}
      else rxIndex=0;
      break;
    
    case FUNCTION_CODE:
      if(rxChar==READ_HOLDING_REGISTERS){
        rxIndex=2;
        //debg=4;
        rxBuffer[1]=rxChar;}
      else rxIndex=0;
      break;
      
    case ADD_BYTE1:
        if(rxChar==READ_ADDRESS1){
          rxIndex=3;
          //debg=4;
          rxBuffer[2]=rxChar;}
        else rxIndex=0;
        break;
          
    case ADD_BYTE2:
        if(rxChar==READ_ADDRESS2){
          rxIndex=4;
          
          rxBuffer[3]=rxChar;}
        else rxIndex=0;
          break;
          
    case BLANK:
        if(rxChar==00){
          rxIndex=5;
          
          rxBuffer[4]=rxChar;}
        else rxIndex=0;
          break;
      
    case DATA_LEN:
      if(rxChar==DATA_LENGTH){
        rxIndex=6;
      
        
        rxBuffer[5]=rxChar;}
      break;
    
    case CRC1:
        rxIndex=7;
        
        rxBuffer[6]=rxChar;
      break;
    
    case CRC2:
        rxIndex=8;
        rxBuffer[7]=rxChar;
        
        rxComplete=1;
      break;
    
    default:
        rxIndex=8;
    
    break;
  }
  }
}


void modbus_CRC16(const unsigned char Data, unsigned int* crc)
{
    unsigned int i;

    *crc = *crc ^(unsigned int) Data;
    for (i = 0; i < 8; ++i)
    {
        if (*crc & 0x0001)
            *crc = (*crc >> 1) ^ 0xA001;
        else
            *crc >>= 1;
    }
}

bool  Modbus_send(uint8_t *data,uint16_t len)
{
  LED_ON;
  if(len <= TX_MAX_SIZE)
  {
    memcpy(uartTxBuffer,data,len);
    for(int i=0;i<len;i++)  Serial2.write(data[i]);
    delay(100);
    LED_OFF;
    return true;
  }
  else
    return false;
}

void Modbus_TxRTU(void)
{
    TxCrc                =0xFFFF;
    modbusTxbufferSize             =0;
    modbusTxBuff[modbusTxbufferSize++]   =SLAVE_ADDRESS;
    //modbus_CRC16(SLAVE_ADDRESS, &TxCrc);
    modbusTxBuff[modbusTxbufferSize++]   =READ_HOLDING_REGISTERS;
   // modbus_CRC16(READ_HOLDING_REGISTERS, &TxCrc);
    modbusTxBuff[modbusTxbufferSize++]   =SEND_BYTE_NUMBERS;
 //   modbus_CRC16(READ_HOLDING_REGISTERS, &TxCrc);

    for(uint8_t i=0; i < 2; i++)
    {
        modbusTxBuff[modbusTxbufferSize++]=adcResult[i];
       
    }
    for(uint8_t i=2; i <SEND_BYTE_NUMBERS ; i++)
    {
        modbusTxBuff[modbusTxbufferSize++]=0;
       
    }
    uint8_t ss=modbusTxbufferSize+3;
    for(uint8_t j=0;j<57;j++) modbus_CRC16(modbusTxBuff[j], &TxCrc);
    
      modbusTxBuff[modbusTxbufferSize++]  = TxCrc & 0x00FF;
      modbusTxBuff[modbusTxbufferSize++] =(TxCrc & 0xFF00) >> 8;
    
      Modbus_send(modbusTxBuff,modbusTxbufferSize);
    
}
