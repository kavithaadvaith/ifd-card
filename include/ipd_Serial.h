/********************************************************
 *		     Integra Micro Software Services (P) Ltd
 *			#4, 12th KM, Bellary Road, 
 *			Jakkur, Bangalore 560064
 *		
 *		� Copyright 2006, IMSSPL, Bangalore, Karnataka
 *			All Rights Reserved
 *
 * @file	<ipd_Serial>.h
 * @ingroup     <Serial>
 *
 * @author	<ChandraShekhar>  (chandrasrk@integramicro.com)
 * @date	<10 OCT 06>
 * @brief 	<  Prototypes, structures and definitions  
                              describing the Serial Interface  > 
 
 ********************************************************
 * Revision History
 *------------------
 * Revision <version no> <Date> <Time> <modified by>
 * Brief Revision Description
 */
/******** File Guard ************************************/
#ifndef _SERIAL_H_
#define _SERIAL_H_
/******** Header file includes***************************/
#include "ipd_platform.h"
#include "ipd_base.h"
#include "error_codes.h"

/******** Macros ****************************************/
/******** Constants *************************************/
//Error Codes
#define IC_ERR_SERIAL_OPEN  							-1
#define IC_ERR_SERIAL_SET_ATTRIB  						-2
#define IC_ERR_SERIAL_PORT_SETTING  					-3
#define IC_ERR_SERIAL_BREAK  							-4
#define IC_ERR_SERIAL_INVALID_WRITE_COUNT 				-5
#define IC_ERR_SERIAL_INVALID_CHSIZE 					-6
#define IC_ERR_SERIAL_INVALID_PARITY_VAL 				-7 
#define IC_ERR_SERIAL_INVALID_STOPBIT 					-8
#define IC_ERR_SERIAL_INVALID_READ_COUNT  				-9
#define IC_ERR_SERIAL_READ 								-10
#define IC_ERR_SERIAL_WRITE 							-11
#define IC_ERR_DEVICE_FILE 								-12
#define IC_ERR_SERIAL_CLOSE								-13
#define IC_ERR_SET_RTS									-14
#define IC_ERR_GET_RTS									-15
#define IC_ERR_SET_DTR									-16	
#define IC_ERR_GET_DTR									-17
#define ICC_ERR_TIMEOUT                    				-18 
#define IC_ERR_GET_RTS_DTR								-19
#define IC_ERR_SET_RTS_DTR								-20
#define IC_ERR_SERIAL_TIMEOUT 								-131
/******** External Globals *******************************/

/******** Typedefs, Structs and Unions *******************/

// SERIAL PORT CONFIGURATION STRUCTURE 
typedef struct ConfigurePort 
{
	IC_UINT32 baudrate;          //Setting the Appropriate Baud Rate for the Serial port
	IC_UINT8 stopbit:2; 	 //1 ,1.5,2 stopbits enabled option
	IC_UINT8 parity :2;	 //No Parity,Even Parity &Odd parity Options enabled
	IC_UINT8 chsize:2;		 //5,6,7,8 data bits options enabled 	
	IC_UINT8 hw_flowcontrol:1;  //Enabling and Disabling Hardware Flow Control
	IC_UINT8 sw_flowcontrol:1;  //Enabling and Disabling Software Flow Control 
	IC_UINT8 vmin;		//Minimum number of characters to be read
	IC_UINT32 c_time;		//Time in milliseconds for the first character to be read
}ST_CONFIGURE_PORT;

// CONFIGURATION OF PARAMETERS FOR THE  PORT

typedef enum OpeningMode
{ 	
		E_SERIAL_BLOCKING,
		E_SERIAL_NONBLOCKING
}E_OPENING_MODE;

typedef enum Stopbitconfig 
{
	E_STOP_1,
	E_STOP_1_5,
	E_STOP_2,
	E_STOP_RES
}E_STOPBITCONFIG;		//enum for stop bit

typedef enum Parityconfig 
{	
	E_PARITY_NONE,
	E_PARITY_ODD,
	E_PARITY_EVEN
}E_PARITYCONFIG;		// parity bit

typedef enum Chsizeconfig 
{	
	E_CH_FIVE,
	E_CH_SIX,
	E_CH_SEVEN,
	E_CH_EIGHT
}E_CHSIZECONFIG; 			// data size

typedef enum Baudconfig 
{
	E_BD0,
	E_BD50,
	E_BD75,
	E_BD110,
	E_BD134,
	E_BD150,
	E_BD200,
	E_BD300,
	E_BD600,
	E_BD1200,
	E_BD1800,
	E_BD2400,
	E_BD4800,
	E_BD9600,
	E_BD19200,
	E_BD38400,
	E_BD57600 = 4097,
	E_BD115200
	
}E_BAUDCONFIG;

typedef enum Hwflowcontrolconfig 
{
	E_HW_FLOW_DISABLE,
	E_HW_FLOW_ENABLE
}E_HWFLOWCONTROLCONFIG ;		// hardware flow control 

typedef enum Swflowcontrolconfig 
{
	E_SW_FLOW_DISABLE,
	E_SW_FLOW_ENABLE
}E_SWFLOWCONTROLCONFIG ;		//software flow control

//Used by both RFID & FingerPrint
typedef  struct conn_serial  {
		IC_UINT8  aucPort[50];
		IC_UINT64  lSpeed;
	}ST_CONN_SERIAL;

typedef enum ConnType 
	{ 
		E_SERIAL,
		E_USB 
	}E_ConnType;

/******** Enums ******************************************/

/******** External Function Prototypes *******************/

IC_INT32 ipd_SerialOpen(IC_UINT8 *pc_port, IC_INT32 i_mode);					//API to open serial port descriptor
IC_INT32  ipd_SerialClose(IC_INT32 i_fd);						//API to close serial port descriptor
IC_INT32 ipd_SerialConfigure(IC_INT32 i_fd, ST_CONFIGURE_PORT *pst_portsettings);	//API to configure serial port	
IC_INT32 ipd_SerialRead( IC_INT32 fd, IC_UINT8 *pucBuffer, IC_INT32 iCount, IC_UINT32 * piNRead );
IC_INT32 ipd_SerialWrite( IC_INT32 fd,IC_UINT8 *pucBuffer,IC_INT32 iCount ,IC_UINT32 * piNWrite );
IC_INT32 ipd_SerialBreak( IC_INT32 fd, IC_UINT32 i_duration );
IC_INT32 ipd_SerialReadCt( IC_INT32 fd, IC_UINT8 *pucBuffer, IC_INT32 iCount, IC_UINT32 * piNRead );

IC_INT32 ipd_SetRts(IC_INT32 i_fd,IC_UINT8 CtlBit);
IC_INT32 ipd_SetDtr(IC_INT32 i_fd,IC_UINT8 CtlBit);
IC_INT32 ipd_SetRtsDtr(IC_INT32 fd, IC_UINT8 CtlBitRts, IC_UINT8 CtlBitDtr);
#endif //_SERIAL_H_
/* End of File */

