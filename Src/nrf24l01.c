#include "nrf24l01.h"

#define SENDER 1
#define REVEICER 2
#define ROLE   REVEICER

#if ROLE==SENDER
uint8_t  RX_ADDRESS[TX_ADR_WIDTH]= {0x20,0x30,0x40,0x40,0x30}; 
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0x20,0x30,0x40,0x40,0x40}; 
#else
uint8_t  RX_ADDRESS[TX_ADR_WIDTH]= {0x20,0x30,0x40,0x40,0x40}; 
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0x20,0x30,0x40,0x40,0x30}; 
#endif

uint8_t  NRF_TX_Data[TX_PLOAD_WIDTH]={1,2,3,4,5,6,7,8,9,10};
uint8_t  NRF_RX_Data[RX_PLOAD_WIDTH]={0};


static void nrf_read_bytes(NRF_Dev *dev,uint8_t reg,uint8_t * rx_buffer,uint8_t size);
static void nrf_write_bytes(NRF_Dev *dev,uint8_t reg,uint8_t * rx_buffer,uint8_t size);
static void nrf_write_reg(NRF_Dev * dev,uint8_t reg,uint8_t tx);
static uint8_t nrf_read_reg(NRF_Dev * dev,uint8_t reg);

NRF_Dev NRF24l01;
static void NRF_Set_GPIO(NRF_GPIO io,NRF_GPIO_Level level);
static void NRF_Read_Write(uint8_t * txData,uint8_t * rxData,uint16_t len);
static void NRF_Analize_Message(uint8_t * data,uint16_t len);
static void NRF_Write(uint8_t * txData,uint16_t len);
static void NRF_Read(uint8_t * rxData,uint16_t len);

//extern void Delay_Us(uint32_t nus);


void NRF_Init(NRF_Dev * dev){
  dev->set_gpio=NRF_Set_GPIO;
  dev->delay_ms=HAL_Delay;
  dev->spi_write_read=NRF_Read_Write;
  dev->tx_data=NRF_TX_Data;
  dev->rx_data=NRF_RX_Data; 
  dev->tx_len=TX_PLOAD_WIDTH;
  dev->rx_len=RX_PLOAD_WIDTH;
  dev->rx_analize=NRF_Analize_Message;
  dev->spi_write=NRF_Write;
  dev->spi_read=NRF_Read;
  dev->delay_us=HAL_Delay;
  
  nrf_write_reg(dev,NRF_CONFIG,0x02);
  nrf_write_reg(dev,EN_AA,0X00); //禁止自动ACK
  nrf_write_reg(dev,SETUP_RETR,0x00); //禁止重发
  nrf_write_reg(dev,RF_CH,0); //2.4ghz
  nrf_write_reg(dev,RX_PW_P0,dev->rx_len); 
  nrf_write_reg(dev,RF_SETUP,0x26);//0x07); //2mhz 0db
  nrf_write_reg(dev,EN_RXADDR,0x01);//允许通道1接收
  nrf_write_reg(dev,NRF_CONFIG,0x03);  
  
  nrf_write_bytes(dev,TX_ADDR,TX_ADDRESS,5);
  nrf_write_bytes(dev,RX_ADDR_P0,RX_ADDRESS,5);
  

  
  dev->set_gpio(CE,LOW);
  nrf_write_reg(dev,FLUSH_RX,0); 
  //dev->set_gpio(CE,HIGH);
}


static void NRF_Set_GPIO(NRF_GPIO io,NRF_GPIO_Level level){
  if(io==CSN){
    HAL_GPIO_WritePin(GPIOA,CS_Pin,level);
  }else if(io==CE){
    HAL_GPIO_WritePin(GPIOA,CE_Pin,level);
  } 
}
//extern SPI_HandleTypeDef hspi2;

static void NRF_Write(uint8_t * txData,uint16_t len){
  HAL_SPI_Transmit(&hspi1,txData,len,50);
}
static void NRF_Read(uint8_t * rxData,uint16_t len){
  HAL_SPI_Receive(&hspi1,rxData,len,50);
}



static void NRF_Read_Write(uint8_t * txData,uint8_t * rxData,uint16_t len){
  HAL_SPI_TransmitReceive(&hspi1,txData,rxData,len,50);
}

float nrf_command,nrf_thrust,nrf_direction,nrf_ele,nrf_aile;
uint32_t nrf_watch_dog;
extern void uprintf(char *fmt, ...);

float TO_PERCENT(uint16_t data){
  return data/65535.0f*100;
}

//***********************************callback函数**************************************/
static void NRF_Analize_Message(uint8_t * data,uint16_t len){

    /*for(int i = 0; i < 32; i++)
        uprintf("%d ",data[i]);
    uprintf("\r\n");*/
    //if(crc_check(data,(int)len - 2))
    //if(crc_check(data,30))
    if(1)
    {
        uint8_t can_data[8] = {0};
        int cnt = 0;
        if(data[0] == 'R')
        {
            for(int i = 0; i < 6; i++)
            {
                can_data[i] = data[i + 1];
            }
            can_send_msg(324, can_data, 6);
        }
        else
        {
            for(int i = 0; i < len - 2; i++)
            {
                if(data[i] == 0) break;
                can_data[i] = data[i];
                cnt ++;
            }
            can_send_msg(325, can_data, cnt);
        }
    }
  nrf_watch_dog=0;
}



void nrf_receive2(NRF_Dev *dev){
	uint8_t t[32];
	int i=0;
    
  if(dev->mode==NRF_RX){
    dev->mode=NRF_WAIT;
    if(nrf_read_reg(dev,STATUS)<0x40)         //未接收到数据
      return ;  
    nrf_write_reg(dev,STATUS,0xFF); 
	dev->set_gpio(CSN,LOW);
	dev->delay_us(10);
	t[0]=R_RX_PAYLOAD;
    dev->spi_write_read(t,t,1);
	for(i=0;i<RX_PLOAD_WIDTH;i++){
		t[i]=NOP;
	}
    dev->spi_write_read(t,dev->rx_data,dev->rx_len);
    
    dev->set_gpio(CE,LOW);
	dev->set_gpio(CSN,HIGH);
    dev->rx_analize(dev->rx_data,dev->rx_len);  
    nrf_write_reg(dev,FLUSH_RX,0);
    nrf_write_reg(dev,NRF_CONFIG,NRF_WAIT);
  }else{   //mode==wait
    dev->mode=NRF_RX;
    nrf_write_reg(dev,FLUSH_RX,0xFF);
    dev->set_gpio(CE,HIGH);
	nrf_write_reg(dev,NRF_CONFIG,NRF_RX);
    nrf_write_reg(dev,FLUSH_RX,0);
  }
}

void nrf_receive(NRF_Dev * dev){
    
	uint8_t t[32];
	int i=0;
    nrf_write_reg(dev,FLUSH_RX,0xFF);
    dev->set_gpio(CE,HIGH);
	nrf_write_reg(dev,NRF_CONFIG,0x03);
	dev->delay_ms(1);
	if(nrf_read_reg(dev,STATUS)<0x40)         //未接收到数据
		return ;
    nrf_write_reg(dev,STATUS,0xFF); 
	dev->set_gpio(CSN,LOW);
	dev->delay_ms(1);
	t[0]=R_RX_PAYLOAD;
    dev->spi_write_read(t,t,1);
	for(i=0;i<RX_PLOAD_WIDTH;i++){
		t[i]=NOP;
	}
    dev->spi_write_read(t,dev->rx_data,dev->rx_len);
    
    dev->set_gpio(CE,LOW);
	dev->set_gpio(CSN,HIGH);
    dev->rx_analize(dev->rx_data,dev->rx_len);  
    nrf_write_reg(dev,FLUSH_RX,0); 
}
void nrf_send_message(NRF_Dev * dev){
	uint8_t r[32],t,rt;
    
    if(dev->tx_len>32)
      return ;
    
	nrf_write_reg(dev,NRF_CONFIG,0x02);  
	t=W_RX_PAYLOAD;
    dev->set_gpio(CSN,LOW);
    dev->spi_write_read(&t,&rt,1);
    
    dev->spi_write_read(dev->tx_data,r,dev->tx_len);
    
	dev->set_gpio(CSN,HIGH);
	dev->set_gpio(CE,HIGH);
	dev->delay_ms(1);
	do{
		t=nrf_read_reg(dev,STATUS);
        if((t&0x10)==0x10){
          nrf_write_reg(dev,STATUS,0x10);
        }
	}while((t&0x20)==0);
	nrf_write_reg(dev,STATUS,0x20);
	dev->set_gpio(CE,LOW);
	nrf_write_reg(dev,NRF_CONFIG,0x03);
}


static void nrf_write_reg(NRF_Dev * dev,uint8_t reg,uint8_t tx){
	uint8_t r,t;
	REDO:
	dev->set_gpio(CSN,LOW);
    //dev->delay_us(1);
	t=W_REGISTER+reg;
    dev->spi_write(&t,1);
	t=tx;
	//dev->delay_us(10);
	dev->spi_write(&t,1);
	dev->set_gpio(CSN,HIGH);
	//dev->delay_ms(1);
	if(reg==STATUS||reg==FLUSH_RX)
      return ;
	if(nrf_read_reg(dev,reg)!=tx){
		goto REDO;
	}
}

static void nrf_write_bytes(NRF_Dev * dev,uint8_t reg,uint8_t * data,uint8_t size){
	uint8_t t,r,rx_buffer[10];
	int i;
	if(size>10)
		return ;
	INIT:
    dev->set_gpio(CSN,LOW);
    //dev->delay_ms(1);
	t=W_REGISTER+reg;
    dev->spi_write_read(&t,&r,1);
    dev->spi_write_read(data,rx_buffer,size);
	dev->set_gpio(CSN,HIGH);
	//dev->delay_ms(1);
	
	nrf_read_bytes(dev,reg,rx_buffer,size);
	for(i=0;i<size;++i){
		if(rx_buffer[i]!=data[i]){
			goto INIT;
		}
	}
	
}

static void nrf_read_bytes(NRF_Dev * dev,uint8_t reg,uint8_t * rx_buffer,uint8_t size){
	uint8_t r;
    uint8_t t[10]={0xFF};
	if(size>10)
		return ;
    dev->set_gpio(CSN,LOW);
	t[0]=reg;
    dev->spi_write_read(t,&r,1);
	t[0]=0xFF;
    dev->spi_write_read(t,rx_buffer,size);
    dev->set_gpio(CSN,HIGH);
}

static uint8_t nrf_read_reg(NRF_Dev * dev,uint8_t reg){
	uint8_t t,r; 
    t=reg;
	dev->set_gpio(CSN,LOW);
    dev->spi_write(&t,1);  
	t=NOP;
    dev->spi_read(&r,1);
	dev->set_gpio(CSN,HIGH);
	return r;
}

static const char aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,

    0x00, 0xC1, 0x81, 0x40
};

static const char aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,

0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,

    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,

    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,

    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,

    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,

    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,

    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,

    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,

    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,

    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,

    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,

    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,

    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,

    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,

    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,

    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,

    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,

    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,

    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,

    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,

    0x41, 0x81, 0x80, 0x40
};

unsigned short usMBCRC16( unsigned char * pucFrame, unsigned short usLen )
{
    unsigned char ucCRCHi = 0xFF;
    unsigned char ucCRCLo = 0xFF;
    int iIndex;
    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( unsigned char )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( unsigned short )( ucCRCHi << 8 | ucCRCLo );
}

//在sendstr后面添加两位crc位 
void add_crc_check(unsigned char *sendstr, int datalen)
{
	unsigned short check = usMBCRC16(sendstr,datalen);
	sendstr[datalen] = (unsigned char)(check / 256);
	sendstr[datalen + 1] = (unsigned char)(check % 256);
} 
//校验rcvstr是否正确,datalen是实际数据长度，不包括校验位 
int crc_check(unsigned char *rcvstr, int datalen)
{
    if(datalen < 0) return -1;
	unsigned short b = usMBCRC16(rcvstr,datalen);
	unsigned short c = rcvstr[datalen];
	c = c*256 + rcvstr[datalen + 1];
	if(c == b) return 1;
	else return 0;
}

//datalen最长为30,每次发送32个字节，不足部分用\0填充,31,32位为校验位。
void nrf_send(uint8_t *sendstr, int datalen)
{
    if(datalen > 30|| datalen < 0) return;
    unsigned char send[32];
    
    for(int i = 0; i < datalen; i++)
    {
        send[i] = sendstr[i];
    }
    for(int i = datalen; i < 32; i++)
    {
        send[i] = 0;
    }
      add_crc_check((unsigned char *)send, 30);
      NRF24l01.tx_data = send;
      NRF24l01.tx_len = 32;
      nrf_send_message(&NRF24l01);
}