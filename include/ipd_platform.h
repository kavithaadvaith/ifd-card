/********************************************************
 *		     Integra Micro Software Services (P) Ltd
 *			#4, 12th KM, Bellary Road, 
 *			Jakkur, Bangalore 560064
 *		
 *		© Copyright 2006, IMSSPL, Bangalore, Karnataka
 *			All Rights Reserved
 *
 * @file	<ipd_platform.h>
 * @ingroup Basic Datatypes
 *
 * @author	kiran            (kiranp@integramicro.com)
 * @date		1-12-2006
 * @brief   Basic Datatypes.

 ********************************************************
 * Revision History
 *------------------
 * Revision  1-Dec-2006 <Time> <modified by>
 * Brief Revision Description
 *
 *******************************************************/

/*******File Guard *******/
#ifndef  PLATFORM_H
#define  PLATFORM_H

/******** Header file includes*******************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <malloc.h>
#include <math.h>
#include <time.h>
#include <fcntl.h>
#include <stdarg.h>
//#include <uuid/uuid.h>
#include <limits.h>
#ifndef _OS_WINDOWS_
#include <unistd.h>
#include <semaphore.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/sem.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <signal.h>
#include <termios.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <sys/mount.h>
#else
#include <WINSOCK.H>
#endif


/******** Macros ****************************************/

/******** Constants **************************************/

/******** External Globals ********************************/

/******** Typedefs, Structs and Unions **********************/

/******** Enums ****************************************/

/******** External Function Prototypes **********************/

#endif //PLATFORM_H
/* End of File */
