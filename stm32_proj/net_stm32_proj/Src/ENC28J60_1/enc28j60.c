#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"


#include "enc28j60.h"


#define MIN(a,b) (a) < (b) ? (a) : (b)

#define CSN_1		HAL_GPIO_WritePin(GPIOA,SPI_NSS_Pin,GPIO_PIN_SET)
#define CSN_0		HAL_GPIO_WritePin(GPIOA,SPI_NSS_Pin,GPIO_PIN_RESET)


uint8_t Enc28j60Bank;
uint16_t NextPacketPtr;
extern SPI_HandleTypeDef hspi1;

extern uint8_t *uip_appdata;
extern uint8_t uip_buf[2048 + 2];   /* The packet buffer that contains
				    incoming packets. */
extern uint16_t uip_len, uip_slen;
//struct uip_eth_addr uip_ethaddr = {{0,0,0,0,0,0}};
void WriteByte(uint8_t temp)
{
	uint8_t datr;	    

	HAL_SPI_TransmitReceive(&hspi1,&temp,&datr,1,10);

}

uint8_t ReadByte(void)
{
	uint8_t dat=0xff,datr;	 
//	ENC28J60_CS=0;	
//	HAL_GPIO_WritePin(GPIOA,SPI_NSS_Pin,GPIO_PIN_RESET);	
//	dat=op|(addr&ADDR_MASK);
	HAL_SPI_TransmitReceive(&hspi1,&dat,&datr,1,10);
//	dat=0xff;
//	HAL_SPI_TransmitReceive(&hspi1,&dat,&datr,1,10);
//	HAL_SPI_Transmit(&hspi1,&dat,1,10);
//	HAL_SPI_Receive(&hspi1,&dat,1,10);
//	SPI2_ReadWriteByte(dat);
//	dat=SPI2_ReadWriteByte(0xFF);
	//如果是读取MAC/MII寄存器,则第二次读到的数据才是正确的,见手册29页
// 	if(addr&0x80){HAL_SPI_TransmitReceive(&hspi1,&dat,&datr,1,10);}
//	ENC28J60_CS=1;
//	HAL_GPIO_WritePin(GPIOA,SPI_NSS_Pin,GPIO_PIN_SET);	
	return datr;
}


/// ENC28J60 
void delay_us(int t1)
{
	while(t1--);
}

void delay_ms(int t1)
{ 
	int i; 
	while(t1--) 
	{
		i=100;
		while(i--);
	}
}

uint8_t enc28j60ReadOp(uint8_t op, uint8_t address)
{
	uint8_t dat1;
	// activate CS	
	CSN_0;
	// issue read command
	WriteByte(op | (address & ADDR_MASK));	
	dat1 = ReadByte();
	// do dummy read if needed (for mac and mii, see datasheet page 29)
	if(address & 0x80) 	dat1 = ReadByte();
	// release CS
	CSN_1;
	return(dat1);
}

void enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t mydat)
{
	CSN_0;
	// issue write command
	WriteByte( op | (address & ADDR_MASK));
	// write data
	WriteByte(mydat);
	CSN_1;
}

void enc28j60SetBank(uint8_t address)
{
	if((address & BANK_MASK) != Enc28j60Bank)
	{
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
		Enc28j60Bank = (address & BANK_MASK);
	}
}

uint8_t enc28j60Read(uint8_t address)
{
	enc28j60SetBank(address);	
	return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}

void enc28j60Write(uint8_t address, uint8_t mydat)
{
	enc28j60SetBank(address);
	enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, mydat);
}

uint16_t enc28j60_read_phyreg(uint8_t address)
{
	uint16_t mydat;
	enc28j60Write(MIREGADR, address);
	enc28j60Write(MICMD, MICMD_MIIRD);
	
	// Loop to wait until the PHY register has been read through the MII
	// This requires 10.24us
	while( (enc28j60Read(MISTAT) & MISTAT_BUSY) );
	
	// Stop reading
	enc28j60Write(MICMD, MICMD_MIIRD);
	
	// Obtain results and return
	mydat = enc28j60Read ( MIRDL );
	mydat |= enc28j60Read ( MIRDH );

	return mydat;
}

void enc28j60PhyWrite(uint8_t address, uint16_t mydat)
{
	// set the PHY register address
	enc28j60Write(MIREGADR, address);
	// write the PHY data
	enc28j60Write(MIWRL, mydat & 0x00ff);
	enc28j60Write(MIWRH, mydat >> 8);
	// wait until the PHY write completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY)
	{
		delay_us(15);
	}
}

void enc28j60ReadBuffer(uint16_t len, uint8_t* dat)
{

	CSN_0;
	WriteByte(ENC28J60_READ_BUF_MEM);
	while(len--)
	{
		*dat++ = ReadByte();
	}	
	CSN_1;
}

void enc28j60WriteBuffer(uint16_t len, uint8_t* dat)
{
	CSN_0;
	WriteByte(ENC28J60_WRITE_BUF_MEM);
	while(len--)
	{
		WriteByte(*dat++);
	}	
	CSN_1;
}

#define ETHERNET_MIN_PACKET_LENGTH	0x3C
#define ETHERNET_HEADER_LENGTH		0x0E

#define IP_TCP_HEADER_LENGTH 40
#define TOTAL_HEADER_LENGTH (IP_TCP_HEADER_LENGTH+ETHERNET_HEADER_LENGTH)

void enc28j60PacketSend(uint16_t len, uint8_t* packet)
{
	// Set the write pointer to start of transmit buffer area
	enc28j60Write(EWRPTL, TXSTART_INIT);
	enc28j60Write(EWRPTH, TXSTART_INIT>>8);

	// Set the TXND pointer to correspond to the packet size given
	enc28j60Write(ETXNDL, (TXSTART_INIT+len));
	enc28j60Write(ETXNDH, (TXSTART_INIT+len)>>8);

	// write per-packet control byte
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);

	// TODO, fix this up

	if( uip_len <= TOTAL_HEADER_LENGTH )
	{
		// copy the packet into the transmit buffer
		enc28j60WriteBuffer(len, packet);
	}
	else
	{
		len -= TOTAL_HEADER_LENGTH;
		enc28j60WriteBuffer(TOTAL_HEADER_LENGTH, packet);
		enc28j60WriteBuffer(len, (unsigned char *)uip_appdata);
	}
	// send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}




uint16_t enc28j60PacketReceive(uint16_t maxlen, uint8_t* packet)
{
	uint16_t rxstat;
	uint16_t len;
	uint16_t rs,re;

	// check if a packet has been received and buffered
	if( !(enc28j60Read(EIR) & EIR_PKTIF) )
	{
		// Errata workaround #6, PKTIF is not reliable
		// double check by looking at EPKTCNT
		if (enc28j60Read(EPKTCNT) == 0)
			return 0;
	}

	// Set the read pointer to the start of the received packet
	enc28j60Write(ERDPTL, (NextPacketPtr));
	enc28j60Write(ERDPTH, (NextPacketPtr)>>8);

	// read the next packet pointer
	NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	NextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

	// read the packet length
	len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

	// read the receive status
	rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	rxstat |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

	// limit retrieve length
	// (we reduce the MAC-reported length by 4 to remove the CRC)
	len = MIN(len, maxlen);

	// copy the packet from the receive buffer
	enc28j60ReadBuffer(len, packet);
	rs = enc28j60Read(ERXSTH);
	rs <<= 8;
	rs |= enc28j60Read(ERXSTL);
	re = enc28j60Read(ERXNDH);
	re <<= 8;
	re |= enc28j60Read(ERXNDL);
	if (NextPacketPtr - 1 < rs || NextPacketPtr - 1 > re)
	{
		enc28j60Write(ERXRDPTL, (re));
		enc28j60Write(ERXRDPTH, (re)>>8);
	}
	else
	{
		enc28j60Write(ERXRDPTL, (NextPacketPtr-1));
		enc28j60Write(ERXRDPTH, (NextPacketPtr-1)>>8);
	}

	// decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);

	return len;
}

void dev_init(void)
{
	enc28j60_init();
}

void dev_send(void)
{
	enc28j60PacketSend(uip_len, uip_buf);
}

uint16_t dev_poll(void)
{
	return enc28j60PacketReceive(UIP_BUFSIZE, uip_buf);
}

void enc28j60_init(void)
{
	// perform system reset
	enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
	delay_ms(5);

	// set receive buffer start address
	NextPacketPtr = RXSTART_INIT;
	enc28j60Write(ERXSTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXSTH, RXSTART_INIT>>8);

	// set receive pointer address
	enc28j60Write(ERXRDPTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXRDPTH, RXSTART_INIT>>8);

	// set receive buffer end
	// ERXND defaults to 0x1FFF (end of ram)
	enc28j60Write(ERXNDL, RXSTOP_INIT&0xFF);
	enc28j60Write(ERXNDH, RXSTOP_INIT>>8);

	// set transmit buffer start
	// ETXST defaults to 0x0000 (beginnging of ram)
	enc28j60Write(ETXSTL, TXSTART_INIT&0xFF);
	enc28j60Write(ETXSTH, TXSTART_INIT>>8);

	// do bank 2 stuff
	// enable MAC receive
	enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);

	// bring MAC out of reset
	enc28j60Write(MACON2, 0x00);

	// enable automatic padding and CRC operations
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);

	// set inter-frame gap (non-back-to-back)
	enc28j60Write(MAIPGL, 0x12);
	enc28j60Write(MAIPGH, 0x0C);
	// set inter-frame gap (back-to-back)
	enc28j60Write(MABBIPG, 0x12);
	// Set the maximum packet size which the controller will accept
	enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);	
	enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);

	// do bank 3 stuff
	// write MAC address
	// NOTE: MAC address in ENC28J60 is byte-backward
	enc28j60Write(MAADR5, UIP_ETHADDR0);
	enc28j60Write(MAADR4, UIP_ETHADDR1);
	enc28j60Write(MAADR3, UIP_ETHADDR2);
	enc28j60Write(MAADR2, UIP_ETHADDR3);
	enc28j60Write(MAADR1, UIP_ETHADDR4);
	enc28j60Write(MAADR0, UIP_ETHADDR5);

	// no loopback of transmitted frames
	enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);

	// switch to bank 0
	enc28j60SetBank(ECON1);

	// enable interrutps
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);

	// enable packet reception
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}


