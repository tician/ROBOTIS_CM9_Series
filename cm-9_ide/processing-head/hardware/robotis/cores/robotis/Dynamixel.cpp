/*
 * Dynamixel.cpp
 *
 *  Created on: 2013. 4. 18.
 *      Author: in2storm
 */

#include "Dynamixel.h"

#include "dxl.h"


Dynamixel Dxl; //[ROBOTIS] declare dynamixel instance in advance, it is used by sketch code.


Dynamixel::Dynamixel() {
	// TODO Auto-generated constructor stub

}

Dynamixel::~Dynamixel() {
	// TODO Auto-generated destructor stub
}

void Dynamixel::begin(int baud) {
	uint32 Baudrate = 0;
	//TxDString("[DXL]start begin\r\n");

	//change GPIO D9(PA9),D10(PA10) -> D20(PB6), D21(PB7) in USART1 using AFIO
	afio_remap(AFIO_REMAP_USART1);
	//must be declare as SWJ_NO_NJRST because dxl bus use PB5 as DXL_DIR in Half Duplex USART and also use SWJ
	afio_cfg_debug_ports(AFIO_DEBUG_FULL_SWJ_NO_NJRST);
#ifdef BOARD_CM900  //Engineering version case

	gpio_set_mode(PORT_ENABLE_TXD, PIN_ENABLE_TXD, GPIO_OUTPUT_PP);
	gpio_set_mode(PORT_ENABLE_RXD, PIN_ENABLE_RXD, GPIO_OUTPUT_PP);
	gpio_write_bit(PORT_ENABLE_TXD, PIN_ENABLE_TXD, 0 );// TX Disable
	gpio_write_bit(PORT_ENABLE_RXD, PIN_ENABLE_RXD, 1 );// RX Enable
#else
	gpio_set_mode(PORT_TXRX_DIRECTION, PIN_TXRX_DIRECTION, GPIO_OUTPUT_PP);
	gpio_write_bit(PORT_TXRX_DIRECTION, PIN_TXRX_DIRECTION, 0 );// RX Enable
#endif

	// initialize GPIO D20(PB6), D21(PB7) as DXL TX, RX respectively
	gpio_set_mode(PORT_DXL_TXD, PIN_DXL_TXD, GPIO_AF_OUTPUT_PP);
	gpio_set_mode(PORT_DXL_RXD, PIN_DXL_RXD, GPIO_INPUT_FLOATING);


	//Initialize USART 1 device
	usart_init(USART1);


	//Calculate baudrate, refer to ROBOTIS support page.
	Baudrate = 2000000 / (baud + 1);

	usart_set_baud_rate(USART1, STM32_PCLK2, Baudrate);
	nvic_irq_set_priority(USART1->irq_num, 0);//[ROBOTIS][ADD] 2013-04-10 set to priority 0
	usart_enable(USART1);

	gbIsDynmixelUsed = 1;  //[ROBOTIS]2012-12-13 to notify end of using dynamixel SDK to uart.c
	clearBuffer256();
	mCommStatus= 0;
	//dxl_initialize(0, baud);
	setLibStatusReturnLevel(2);
	setLibNumberTxRxAttempts(2);
}


void Dynamixel::end(void){
	gbBusUsed = 0; //[ROBOTIS]2012-12-13 to notify end of using dynamixel SDK to uart.c
	//will be removed by ROBOTIS,.LTD. there maybe not be used...
}


/*
 *  [ROBOTIS][ADD][START] 2013-04-09 support read and write on dxl bus
 * */

byte Dynamixel::readRaw(void){

	/*byte bData=0;
	bData = RxByteFromDXL();
	clearBuffer256();*/
	return RxByteFromDXL();
}
void Dynamixel::writeRaw(uint8 value){

	DXL_TXD

	TxByteToDXL(value);

	DXL_RXD

}
/*
 * @brief : if data coming from dxl bus, returns 1, or if not, returns 0.
 *
 */
byte Dynamixel::available(void){

	return checkNewArrive();
}

byte Dynamixel::readByte(byte bID, byte bAddress){
	gbpParameter[0] = bAddress;
	gbpParameter[1] = 1;
	if(txrx_Packet(bID, INST_READ, 2)){
		mCommStatus = 1;
		return(gbpRxBuffer[5]);
	}
	else{
		mCommStatus = 0;
		return 0xff;
	}
}

word Dynamixel::readWord(byte bID, byte bAddress){
	gbpParameter[0] = bAddress;
	gbpParameter[1] = 2;
	if(txrx_Packet(bID, INST_READ, 2))
	{
		mCommStatus = 1;
		return( (((word)gbpRxBuffer[6])<<8)+ gbpRxBuffer[5] );
	}
	else{
		mCommStatus = 0;
		return 0xffff;
	}
}

byte Dynamixel::writeByte(byte bID, byte bAddress, byte bData){
	gbpParameter[0] = bAddress;
	gbpParameter[1] = bData;
	mCommStatus = txrx_Packet(bID, INST_WRITE, 2);
	return mCommStatus;
}

byte Dynamixel::writeWord(byte bID, byte bAddress, word wData){
    gbpParameter[0] = bAddress;
    gbpParameter[1] = (byte)(wData&0xff);
    gbpParameter[2] = (byte)((wData>>8)&0xff);
    mCommStatus = txrx_Packet(bID, INST_WRITE, 3);
    return mCommStatus;
}
/*
 * @brief Sets the target position and speed of the specified servo
 * @author Made by Martin S. Mason(Professor @Mt. San Antonio College)
 * @change 2013-04-17 changed by ROBOTIS,.LTD.
 * */
byte Dynamixel::setPosition(byte ServoID, int Position, int Speed){
	gbpParameter[0] = (unsigned char)30;
	gbpParameter[1] = (unsigned char)getLowByte(Position);
	gbpParameter[2] = (unsigned char)getHighByte(Position);
	gbpParameter[3] = (unsigned char)getLowByte(Speed);
	gbpParameter[4] = (unsigned char)getHighByte(Speed);
    return(txrx_Packet(ServoID, INST_WRITE, 5));
}
byte Dynamixel::getLowByte( word wData ){

	return (byte)(wData & 0xff);
}

byte Dynamixel::getHighByte( word wData ){

	return ((wData & 0xff00) >> 8);
}
uint16  Dynamixel::makeWord( byte lowbyte, byte highbyte ){
	uint16 wData;

	wData = ((highbyte & 0x00ff) << 8);
	return (wData + lowbyte);
}

byte  Dynamixel::ping( byte  bID ){
	return(txrx_Packet(bID, INST_PING, 0));
}
byte  Dynamixel::reset( byte  bID ){

	return(txrx_Packet(bID, INST_RESET, 0));
}

byte  Dynamixel::getResult(void){
//	return mCommStatus;
	return getTxRxStatus();
}

byte Dynamixel::setLibStatusReturnLevel(byte num)
{
	return setDxlLibStatRtnLvl(num);
}

byte Dynamixel::setLibNumberTxRxAttempts(byte num)
{
	return setDxlLibNumTries(num);
}

/*
 * @brief initialize parameter and get ID, instruction for making packet
 * */
void Dynamixel::initPacket(byte bID, byte bInst){
	mbLengthForPacketMaking = 0;
	mbIDForPacketMaking = bID;
	mbInstructionForPacketMaking = bInst;
	mCommStatus = 0;
}
/*
 * @brief just push parameters, individual ID or moving data, individual data length
 * */
void Dynamixel::pushByte(byte value){
	//packet length is not above the maximum 143 bytes because size of buffer receiver has only 143 bytes capacity.
	//please refer to ROBOTIS e-manual (support.robotis.com)
	if(mbLengthForPacketMaking > 140)//prevent violation of memory access
		return;
	gbpParameter[mbLengthForPacketMaking++] = value;
}
/*
 * @brief transfers packets to dynamixel bus
 * */
byte Dynamixel::flushPacket(void){

	//TxDString("\r\n");
	//TxD_Dec_U8(gbLengthForPacketMaking);
	mCommStatus = txrx_Packet(mbIDForPacketMaking, mbInstructionForPacketMaking, mbLengthForPacketMaking);
	return mCommStatus;
}
/*
 * @brief return current the total packet length
 * */
byte Dynamixel::getPacketLength(void){
	return mbLengthForPacketMaking;
}



void Dynamixel::setTxPacketId(byte id){
	mbIDForPacketMaking = id;

}
void Dynamixel::setTxPacketInstruction(byte instruction){
	mbInstructionForPacketMaking = instruction;

}
void Dynamixel::setTxPacketParameter( byte index, byte value ){
	gbpParameter[index] = value;

}
void Dynamixel::setTxPacketLength( byte length ){
	mbLengthForPacketMaking = length;

}
byte Dynamixel::txrxPacket(void){
	mCommStatus = txrx_Packet(mbIDForPacketMaking, mbInstructionForPacketMaking, mbLengthForPacketMaking);
	return mCommStatus;
}

int Dynamixel::getRxPacketParameter( int index ){
	//return dxl_get_rxpacket_parameter( index );
	return gbpRxBuffer[5 + index];
}
int Dynamixel::getRxPacketLength(void){
	//return dxl_get_rxpacket_length();
	return gbpRxBuffer[3]; //length index is 3 in status packet
}
/*
 *  ERROR Bit table is below.
#define ERRBIT_VOLTAGE		(1)
#define ERRBIT_ANGLE		(2)
#define ERRBIT_OVERHEAT		(4)
#define ERRBIT_RANGE		(8)
#define ERRBIT_CHECKSUM		(16)
#define ERRBIT_OVERLOAD		(32)
#define ERRBIT_INSTRUCTION	(64)
 * */
int Dynamixel::getRxPacketError( byte errbit ){
	//return dxl_get_rxpacket_error( errbit );
	if( gbpRxBuffer[4] & errbit ){ //error bit index is 4 in status packet
		return 1;
	}
	return 0;

}
