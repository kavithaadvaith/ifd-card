//#include "gl11.h"
#include "Txn_card_code.h"


IC_DEV_HANDLE lg_devHandle;
IC_UINT32 CardGlobalErrCode;
IC_UINT32 cardidentity[10]={0,};
IC_UINT32 cardphyidentity[10]={0,};
IC_UINT32 cardindex = 0;
IC_UINT8  ucLogNumTgt;
IC_UINT8 aucUIDBuffer[MAX_CARDS][MAX_UID_SIZE];
IC_DEV_HANDLE g_devHandle;


static ST_DEV_HANDLE CtSerHandle;
static ST_DEV_HANDLE *CtSerialHandle = (ST_DEV_HANDLE *) &CtSerHandle;



static void Delay(IC_INT64 l_nsec)
{
	time_t t1,t2;

	t1 = time(NULL);
	while(1)
	{
		t2 = time(NULL);
		if ((t2 - t1) >= l_nsec)
			break;
	}
}

int main()
{

	int retval = -1;
	int ret = -1;

	sleep(2);
	
	printf("\nInsert card\n");

	sleep(5);
	ret = InitializeCard();

	if(ret != IC_OK)
	{
		ContactSerialDeInit(lg_devHandle);
		printf("\n InitializeCard failed\n");
	}

	ret = CheckIfCardPresent();

	if(ret != IC_OK)
	{
		printf("\n CheckIfCardPresent failed\n");
	}

	
}

int InitializeCard()
{

	int ret = -1;


	/*************************** RFID interface initialization ***************************/

	E_DEVTYPE devtype;

	devtype = E_GNOME ;
	//		devtype = E_IMTS_V511 ;
	//		devtype = E_IMTS ;

	ret = ContactInit( lg_devHandle, devtype );

	if(ret != 0)
	{
		ContactSerialDeInit(lg_devHandle);

		ret = ContactInit( lg_devHandle, devtype );

		if(ret != 0)
		{
			printf("\nContactInit failed - Initialization failed\n");
			return -1;
		}
	}

	printf("\nCard reading successful.\n");

	return 0;
}

IC_RETCODE ContactInit ( IC_VOID *pst_dev_handle, E_DEVTYPE devtype )
{
	//devtype == E_GNOME;
	if (IC_OK != ContactSerialInit_imts_v511( pst_dev_handle, devtype ))
	{
		printf("Error in Initializing the Contact Serial Device \n");
		return ICC_ERR_CONTACT_SERIAL_INIT;
	}

	return IC_OK;
}

IC_RETCODE ContactSerialInit_imts_v511 ( IC_VOID *pst_dev_handle, E_DEVTYPE devtype)
{
	IC_INT32 SerialConfigureStatus, ReturnStatus;
	ST_CONFIGURE_PORT st_serial_settings;
	IC_UINT8 s[2][15] ;
	IC_INT32 i = 0;
	if(devtype == E_IMTS_V511)
	{
		strcpy(s[0], "/dev/ttymxc4");
		strcpy(s[1], "/dev/ttyUSB0");

	}
	else if(devtype == E_GNOME)
	{
		strcpy(s[0], "/dev/ttymxc3");
		strcpy(s[1], "/dev/ttyUSB0");
	}
	else
	{
		printf("\nInvalid device type\n");
		return IC_ERR;
	}

	SerialConfigureStatus =0;
	for( i = 0; i<2; i++ )
	{
		CtSerialHandle->fd = ipd_SerialOpen( s[i], E_SERIAL_BLOCKING);
			
		
		if( CtSerialHandle->fd == -1 )
		{
			printf("4.Contct Serial Open Failed\n");
			CardGlobalErrCode = 4;
			continue;
		}
		break;
	}
	if( i == 2 )
		return ICC_ERR_RFID_INIT;


	st_serial_settings . stopbit = E_STOP_1;
	st_serial_settings . parity = E_PARITY_NONE;
	st_serial_settings . chsize = E_CH_EIGHT;
	st_serial_settings . hw_flowcontrol = E_HW_FLOW_DISABLE;
	st_serial_settings . sw_flowcontrol = E_SW_FLOW_DISABLE;
	st_serial_settings . vmin = 0;
	st_serial_settings . c_time = 500;  //C_time =0 for visiontek

	if(devtype == E_IMTS_V511)
		st_serial_settings . baudrate = E_BD38400 ;
	else
		st_serial_settings . baudrate = E_BD115200 ;



	ReturnStatus = ipd_SerialConfigure(CtSerialHandle->fd, &st_serial_settings);

	if( ReturnStatus !=IC_OK )
	{
		CardGlobalErrCode =5;
		printf("\nContact serial configuration failed\n");
		return ICC_ERR_SERIAL_CONFIGURE;
	}

	if(SetBaudrate_gl14() <0){
		fprintf(stdout,"Please Check Baudrate\n");
		if (IC_OK != ContactSerialDeInit( pst_dev_handle ))
		{
			printf("Error in DeInitializing the Contact Serial Device \n");
			return ICC_ERR_CONTACT_SERIAL_INIT;
		}
	}

	// Powering the Contact Serial Device 
	return IC_OK;

}

int SetBaudrate_gl14(void)
{
	char Resp[25]="", Cmd[6] ={0x60,0x00,0x01,0x6A,0x01,0x0A};
	int i=0, ssk=0;

	fprintf(stdout,"BaudRate_Read Returns Values(5): ");
	for(i=0; i<5; i++)
	{
		if(write( CtSerialHandle->fd , Cmd, 6) !=6)
			perror("Write");
		sleep(1);
		ssk = read(CtSerialHandle->fd , Resp, sizeof(Resp));
		fprintf(stdout,"%d \n",ssk);
		//fprintf(stdout,"Resp =%s \n",Resp);
		if(!strcmp(Resp, "Baudrate Set"))
		{
			fprintf(stdout,"\nBaudrate Set Success:===>%d",i);
			break;
		}
		else if(!strcmp(Resp, Cmd))
		{
			fprintf(stdout,"\nThis Baudrate is Already Set:===>%d",i);
			break;
		}
	}
	if(IC_OK!=ContactCardPowerOff(CtSerialHandle))
	{
		fprintf(stdout,"\nThis ContactCardPowerOff failed:===>%d",i);
	}
	printf(" \n");
	if(i >4 || ssk<0)
	{
		fprintf(stdout,"Baudrate Setting Time Out/Set Fail\n");
		return -1;
	}

	return IC_OK;

}

IC_RETCODE  ContactCardPowerOff ( IC_VOID *pst_dev_handle )
{
	IC_UINT32 ReturnStatus;
	IC_UINT8 InitBuffer[] = { 0x60, 0x00, 0x00, 0x4D, 0x2D};
	IC_UINT32 Length = 5;
	IC_UINT8 ReadBuffer[50]= {0,};
	IC_UINT32 rxlength=0, CmdStatus = 0; //, RxChkSum =0 , CalChkSum = 0   ;

	/* Card Power Up  Command */
#if 1    
	printf("\nCtReader Scosta Power Off Command\n");
#endif
	ReturnStatus = CardTransmit(CtSerialHandle->fd, InitBuffer, Length, ReadBuffer, &rxlength);

	printf("\nCtReader Scosta Power Off Command Resp \n");

	if (ReturnStatus != IC_OK)
	{

		CmdStatus = ReadBuffer[0];
		/*Tracking the Error Reponse */
		switch(CmdStatus)
		{
			case 0x60 :
				printf("\nCommand Success \n");
				break;

			case 0xE0 :
				if(ReadBuffer[4] == 0xc0)
				{
					printf("\nCard Absent \n");
					return ICC_ERR_CARD_ABSENT;
				}
				else
				{
					printf("\nCard Power Off failed\n");
					CardGlobalErrCode = 598;
					return ICC_ERR_CARD_POWERUP;
				}
			default :
				printf("\nInvalid Response \n");
				CardGlobalErrCode = 599;
				return ICC_ERR_CARD_POWERUP;
		}
	}

	return IC_OK;
}


IC_INT32 ipd_SerialOpen(IC_UINT8 *pc_port, IC_INT32 i_mode)
{
	IC_INT32 fd;

	IC_UINT64       i_ndelay = 0; //Blocking Mode -KP
	IC_INT32        i_oflag = O_RDWR | O_NOCTTY | O_NDELAY;

	if (i_mode == E_SERIAL_BLOCKING)
	{
		i_oflag = O_RDWR | O_NOCTTY;
		i_ndelay = 0;
	}

	if((fd = open((char *)pc_port, i_oflag)) == -1)
	{
		perror("\nSerial open ");
		return IC_ERR_SERIAL_OPEN;
	}

	if((fcntl(fd, F_SETFL, i_ndelay) == -1 ))
	{
		perror("\nfcntl ");
		return IC_ERR_SERIAL_OPEN;
	}
	printf("\nfd is : %d\n", fd);     
	return fd;      
}


IC_INT32 ipd_SerialConfigure( IC_INT32 fd, ST_CONFIGURE_PORT *pst_portsettings )
{

	struct termios st_config;

	if( tcgetattr(fd, &st_config ) == -1)  // Get the Current attributes for the port
	{
		fprintf(stderr,"Port Settings error\n");
		return IC_ERR_SERIAL_PORT_SETTING;
	}

	st_config.c_cflag |= (CLOCAL | CREAD); //Becoming the owner of the port and enabling the reciever
	st_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//choosing raw input and disabling cannonical input
	st_config.c_oflag &= ~OPOST;//Choosing the raw output
	st_config.c_cflag &= ~CBAUD; //Bit mask for baud rate
	st_config.c_cflag |= pst_portsettings->baudrate; // Setting up the Required Baudrate
	st_config.c_iflag &= ~(ICRNL | INLCR); //Fixed Bug

	st_config.c_cflag &= ~CSIZE;//Setting the character size by masking 
	switch( pst_portsettings -> chsize )  //Setting up the character size (5,6,7 and 8 data bits)
	{
		case E_CH_FIVE:
			st_config.c_cflag |= CS5;               // Data bits :5
			break;

		case E_CH_SIX:
			st_config.c_cflag |= CS6;               //Data bits  :6
			break;

		case E_CH_SEVEN :
			st_config.c_cflag |= CS7;               //Data bits : 7
			break;

		case E_CH_EIGHT :
			st_config.c_cflag |= CS8;               //Data bits :8
			break;

		default :
			perror("Invalid CHAR sizeerror in chsize");
			return IC_ERR_SERIAL_INVALID_CHSIZE;
	}

	switch ( pst_portsettings -> parity )   //Setting up the parity(NO PARITY,ODD,EVEN,)
	{
		case E_PARITY_NONE :
			st_config.c_cflag &= ~PARENB;
			break;

		case E_PARITY_ODD :
			st_config.c_cflag |= PARENB;
			st_config.c_cflag |= PARODD;
			st_config.c_iflag |= (INPCK | ISTRIP);
			break;

		case E_PARITY_EVEN :
			st_config.c_cflag |= PARENB;
			st_config.c_cflag &= ~ PARODD;
			st_config.c_iflag |= (INPCK | ISTRIP);
			break;
		default :
			perror(" Invalid Parity Error \n");
			return IC_ERR_SERIAL_INVALID_PARITY_VAL;

	}

	switch( pst_portsettings -> stopbit )   //Setting up the stop bit(1,1.5,2)
	{
		case E_STOP_1:
			st_config.c_cflag &= ~CSTOPB; //One Stop Bit
			break;

		case E_STOP_1_5:
			break;

		case E_STOP_2:
			st_config.c_cflag |= CSTOPB; //2 Stop Bits
			break;

		default :
			perror("\nInvalid  in the stopbit \n");
			return IC_ERR_SERIAL_INVALID_STOPBIT;
	}

	if (pst_portsettings -> hw_flowcontrol)   //Enabling and Disabling the hardware flow control
		st_config.c_cflag |=  CRTSCTS; // Enable
	else
		st_config.c_cflag &= ~CRTSCTS; // Disable

	if (pst_portsettings -> sw_flowcontrol) //Enabling and disabling the software flow control
		st_config.c_iflag |= (IXON|IXOFF|IXANY); //Enable
	else
		st_config.c_iflag &= ~(IXON|IXOFF|IXANY); //Disable

	st_config.c_cc[VMIN] = pst_portsettings->vmin; //Minimum character to be read
	st_config.c_cc[VTIME] = (pst_portsettings->c_time / 100); // Max time allowed to read the first character
	// Convert millisec to VTIME unit.
	//Setting the new settings 
	if(tcsetattr(fd,TCSANOW,&st_config) == -1)
	{
		fprintf(stderr,"error setting the new attributes\n");
		return IC_ERR_SERIAL_SET_ATTRIB;
	}

	return IC_OK;
}

IC_INT32 ipd_SerialWrite( IC_INT32 fd,IC_UINT8 *pucBuffer,IC_INT32 iCount ,IC_UINT32 * piNWrite )
{
	IC_INT32 iTemp = 0;
	IC_INT32 iLocalWrite = 0;
	IC_INT32 iWriteCount = iCount;
	IC_UINT8 *pucWriteBuffer = pucBuffer;

	*piNWrite = 0;

	if(iCount < 0)
		return IC_ERR_SERIAL_INVALID_WRITE_COUNT;

	if((tcflush(fd,TCIOFLUSH) == -1 ))
	{
		perror("tcflush : \n");
		return -1 ;
	}

	while (iTemp != iWriteCount )
	{
		iLocalWrite = write( fd, pucWriteBuffer, iCount );
		if(iLocalWrite == -1)
		{
			perror("Serial Write:");
			*piNWrite = iTemp;
			return IC_ERR_SERIAL_WRITE;
		}

		iTemp += iLocalWrite;
		iCount -= iLocalWrite;
		pucWriteBuffer += iLocalWrite;
	}
	*piNWrite = iTemp;

	return IC_OK;
}



IC_RETCODE ContactSerialDeInit ( IC_VOID *pst_dev_handle)
{

	close(CtSerialHandle->fd);

	printf("\ngl11_ifd_control poweroff  success\n ");

	return IC_OK;

}


int CheckIfCardPresent()
{
	int ret = -1;

	/***************************** Card Present Check ******************************/

	ST_USR_CARD_DET st_usrdet;

	st_usrdet.ucBCOverride = '0';
	
	ret = IsSameCardPresent(lg_devHandle, &st_usrdet);

	if(ret == ERR_GAT_CARD_LIST_FAILED)
	{
		printf("\nCard is not present in the slot\n");
		return IC_ERR;
	}
	else
	{
		printf("\nCard is present in the slot\n");
		return IC_OK;
	}
}

IC_RETCODE IsSameCardPresent(IC_DEV_HANDLE lg_devHandle, ST_USR_CARD_DET st_usrdet)
{

	int i, j, card_count, uid_len, ret;
	IC_UINT8  uid_buff[UID_BUFLEN] = {0,};
	IC_UINT32 uid_buflen = 0;

	IC_DEV_HANDLE localg_devHandle;
	ST_CARD st_cardspresent[MAX_CARDSDETECT];
	ST_CARD st_carduid;

	/* getting card list */
	if(IC_OK != ContactGetCardList(lg_devHandle, uid_buff, &uid_buflen))
	{
		printf("\nContactGetCardList failed\n");
		return ERR_GAT_CARD_LIST_FAILED;
	}

	for (i = 0, card_count = 0; i < uid_buflen ; i=i+1+12, card_count++)
	{
		uid_len = uid_buff[i];
		st_cardspresent[card_count].ucUidLen = uid_len;
		for(j=0; j < uid_len ; j++)
		{
			st_cardspresent[card_count].aucUidBuf[j]= uid_buff[(i+1)+j];
		}
	}

	st_carduid = st_cardspresent[0];

	if(memcmp(st_carduid.aucUidBuf, st_usrdet.stAccGenDet.stCardUID.aucUidBuf, st_carduid.ucUidLen) != 0)
	{
	//	printf("\ndifferent card\n");
		return IC_ERR;
	}

	return IC_OK;

}

IC_RETCODE  ContactGetCardList ( IC_VOID *pst_dev_handle, IC_UINT8 *bufferuids, IC_UINT32 *pi_length )
{
	IC_UINT32 ReturnStatus = IC_OK;
	IC_INT8 log_msg[250 + 1] = {0,};

	select_card_vt(pst_dev_handle, 0x01);

	ReturnStatus = ContactSerialGetCardList(pst_dev_handle, bufferuids, pi_length);

	if (ReturnStatus != IC_OK)
	{
		printf("\nGetting card list failed (Error value : %d)\n", ReturnStatus);
		return ReturnStatus;
	}

	return IC_OK;

}

IC_RETCODE select_card_vt(IC_VOID *pst_dev_handle, IC_UINT32 CardNum)
{
	IC_UINT32 ReturnStatus = IC_OK;
	IC_UINT8  SelCard[] = {0x60,0x00,0x01,0x6A ,0x02,0x09};
	IC_UINT8  ReadBuffer[50] = {0, };
	IC_UINT32 rxlength = 0;

#ifdef GL_14
	if(CardNum==0x02)
		CardNum = 0x03;
#endif
	SelCard[4] = CardNum;   //0x01 - Inrenal SAM 0x02 - External IFD
	SelCard[5] = calculate_crc(SelCard, 5);

	printf("\nCtReader Switching Command\n");

	ReturnStatus = CardTransmit(CtSerialHandle->fd, SelCard, 6, ReadBuffer, &rxlength);
	if (ReturnStatus != IC_OK)
	{
		printf("\nRead Command Failed \n");
		return ReturnStatus;
	}

	usleep(200000);

	return IC_OK;
}

IC_RETCODE  ContactSerialGetCardList ( IC_VOID *pst_dev_handle, IC_UINT8 *bufferuids, IC_UINT32 *pi_length )
{

	IC_UINT8 ReadBuffer[50] = {0,};
	IC_UINT32 ReturnStatus;
	IC_UINT8 WriteBuffer[] = SCOSTA_GET_CHIP_SERIAL_CT;
	IC_UINT32 rxlength = 0 ;
	IC_UINT8 ByteToRead=0;

	cardindex = 0;

	IC_UINT32 i;
	// Power ON Command

	ReturnStatus = IsContactCardActive(CtSerialHandle);

	if (ReturnStatus != IC_OK)
	{
		if (ReturnStatus == ICC_ERR_CARD_DEAVTIVE)
		{
			ReturnStatus = ContactCardPowerUp(CtSerialHandle);
			if (ReturnStatus != IC_OK)
			{
				printf("\nError in Card Powering UP \n");
				return ReturnStatus;
			}
		}
		else
		{
			printf("\nFailed in card active check\n");
			return ReturnStatus;
		}
	}

	/*Delay for stabilising the Contact Circuit  */
	usleep(1000);
	// Get Card List Command
	printf("\nCtReader SCOSTA_GET_CHIP_SERIAL Command\n");
	ReturnStatus = CardTransmit(CtSerialHandle->fd, WriteBuffer, SCOSTA_GET_CHIP_SERIAL_LEN_CT, ReadBuffer, &rxlength);
	if (ReturnStatus != IC_OK)
	{
		printf("\nGet Card List Command Failed \n");
		return ReturnStatus;
	}

	/*Validating the Response */
	//      if((ReadBuffer[16] != 0x90) || (ReadBuffer[17] != 0x00))
	if((ReadBuffer[ReadBuffer[2] + 2] != 0x90) )
	{
		if((ReadBuffer[ReadBuffer[2] + 2] != 0x61) )
		{
			if((ReadBuffer[ReadBuffer[2] + 2] != 0x6C) )
			{

				printf("Command Failure  %s \n", __FUNCTION__);
				CardGlobalErrCode = 26;
				return ICC_ERR_CMD_CHIP_SERIAL_NUM;
			}
		}
	}

	if ( (ReadBuffer[ReadBuffer[2] + 2] == 0x6C ) )
	{
		ByteToRead = ReadBuffer[ReadBuffer[2] + 3];

		ReturnStatus =  GetCardResponse( CtSerialHandle, ReadBuffer, ByteToRead );

		if(ReturnStatus != IC_OK )
		{
			printf("\nError in Getting Response %s \n", __FUNCTION__);
			CardGlobalErrCode =27;
			return ReturnStatus;
		}

		*bufferuids = ReadBuffer[0];

		memcpy(bufferuids + 1, ReadBuffer + 1, ByteToRead);
		memcpy(aucUIDBuffer[cardindex], bufferuids, bufferuids[0] + 1);

		*pi_length += UID_LENGTH + 1;/*Extra Byte for the Length */

	}

	else if ( (ReadBuffer[ReadBuffer[2] + 2] == 0x90 ) )
	{
		*bufferuids = ReadBuffer[2] - 2 ;

		memcpy(bufferuids + 1, ReadBuffer + 4, ReadBuffer[2] - 2);
		memcpy(aucUIDBuffer[cardindex], bufferuids, bufferuids[0] + 1);

		//*pi_length = SCOSTA_CHIP_SERIAL_LEN_CT + 1;    //Extra Byte for the Length 
		*pi_length += UID_LENGTH + 1;/*Extra Byte for the Length */
	}

	printf("\nlength %d\n", *pi_length);
	printf("\nUID For Customer \n");
	for(i=4;i<4 + ReadBuffer[2] - 2;i++)
		printf("%2.2x ", ReadBuffer[i]);
	printf("\n");

	cardidentity[cardindex] = SCOSTA;
	cardphyidentity[cardindex++] = CONTACT_CARD;
	ucLogNumTgt = cardindex;

	return IC_OK;
}


IC_RETCODE  IsContactCardActive( IC_VOID *pst_dev_handle )
{

	IC_UINT32 ReturnStatus;
	//IC_UINT8 InitBuffer[] = {0x60,0x00,0x00,0xA6,0xC6 };
	IC_UINT8 InitBuffer[] = {0x60,0x00,0x07,0x00,0x00,0xa4,0x00,0x00,0x02,0x3f,0x00,0xfe};
	IC_UINT32 Length = 12; //0x05;
	IC_UINT8 ReadBuffer[50]= {0,};
	IC_UINT32 rcount=0, CmdStatus = 0 ;

	/* Card Power Up  Command */
	printf("\nCtReader IsContactCardActive/Present Command\n");
	ReturnStatus = CardTransmit(CtSerialHandle->fd, InitBuffer, Length, ReadBuffer, &rcount);

	CmdStatus = ReadBuffer[0];
	/*Tracking the Error Reponse */
	switch(CmdStatus)
	{
		case 0x60 :                     printf("\nCommand Success \n");
						break;

		case 0xE0 :                     if(ReadBuffer[4] == 0xC0)
						{
							printf("\nCard Absent \n");
							return ICC_ERR_CARD_ABSENT;
						}
						else if ((ReadBuffer[4] == 0x40) || (ReadBuffer[4] == 0xE7) || (ReadBuffer[4] == 0xE5) || (ReadBuffer[4] == 0xE6)|| (ReadBuffer[4] == 0x22) )
						{
							printf("\nCard inactive\n");
							return ICC_ERR_CARD_DEAVTIVE;
						}
						else
						{
							printf("\nCard transmit error\n");
							return IC_ERR;
						}

		default :               printf("\nInvalid Response \n");
					printf("\nInvalid response from card\n");
					return ICC_ERR;
	}

	if (ReturnStatus != IC_OK)
	{
		printf("\nCard transmit failed\n");
		printf("\nFailed in IsContactCardPresent\n");
		return ReturnStatus;
	}

	return IC_OK;
}

IC_RETCODE  ContactCardPowerUp ( IC_VOID *pst_dev_handle )
{
	IC_UINT32 ReturnStatus;
	IC_UINT8 InitBuffer[] = SCOSTA_POWERUP_3V;
	IC_UINT32 Length = SCOSTA_CARD_POWERUP_3V_CMD_LEN;
	//      IC_UINT8 InitBuffer[] = SCOSTA_POWERUP_IS0;
	//      IC_UINT32 Length = SCOSTA_CARD_POWERUP_CMD_LEN;

	IC_UINT8 ReadBuffer[50]= {0,};
	IC_UINT32 rxlength=0, CmdStatus = 0   ;


	/* Card Power Up  Command */
	printf("\nCtReader Scosta Power Up Command\n");
	ReturnStatus = CardTransmit(CtSerialHandle->fd, InitBuffer, Length, ReadBuffer, &rxlength);
	if (ReturnStatus != IC_OK)
	{

		CmdStatus = ReadBuffer[0];
		/*Tracking the Error Reponse */
		switch(CmdStatus)
		{
			case 0x60 :
				printf("Command Success \n");
				break;

			case 0xE0 :
				if(ReadBuffer[4] == 0xc0)
				{
					printf("\nCard Absent \n");
					return ICC_ERR_CARD_ABSENT;
				}
				else
				{
					printf("\nCard Power Up failed\n");
					CardGlobalErrCode = 15;
					return ICC_ERR_CARD_POWERUP;
				}
			default :
				printf("\nInvalid Response \n");
				CardGlobalErrCode = 16;
				return ICC_ERR_CARD_POWERUP;
		}
	}

	return IC_OK;
}


IC_RETCODE CardTransmit( IC_UINT32 fd, IC_UINT8 *bSendBuffer, IC_UINT32 txlength,IC_UINT8 *bRecvBuffer, IC_UINT32 *rxlength )
{

	IC_UINT32  WriteStatus, ReadStatus;
	IC_UINT32 wcount;
	IC_UINT8  CalChkSum=0, RxChkSum = 0;
	//IC_UINT32 HeaderCnt = 0;
	IC_UINT32  rcount=0 ;
	IC_UINT32 iRetry = 3;
	IC_UINT32  retryFlag = 0;
	IC_UINT32 ReadLen = 0, CurrentState = 0;
	IC_UINT8 *pucReadBuffer=bRecvBuffer;

	IC_UINT32 i;

	ST_RFID_DEV_HANDLE CtSerialHandle ;
	CtSerialHandle.fd = fd;


	for(i=0;i<txlength;i++)
		printf("%2.2x ", bSendBuffer[i]);
	printf("\n");

	WriteStatus = ipd_SerialWrite( fd, bSendBuffer, txlength, &wcount );
	if (WriteStatus != IC_OK)
	{
#ifdef LED
		// Switching Off the LED for the Contact Serial Reader //
		ipd_LedCtRdr_Off( &CtSerialHandle  );
#endif

		printf("\nWrite to Device Failed \n");
		CardGlobalErrCode = 18;
		printf("\nSerial write failed\n");
		return WriteStatus;
	}
#if 1
	CurrentState = STX_FRAME;
	ReadLen = 1;
	*rxlength = 0;

	while (iRetry)
	{
		if (CurrentState == LENGTH_FRAME)
			ReadLen = 2;
		else if (CurrentState == DATA_FRAME )
			ReadLen = bRecvBuffer[2] + 2;
		else if (CurrentState == 0x00)
			break;

		while(ReadLen)
		{
			ReadStatus = ipd_SerialReadCt( fd, pucReadBuffer, ReadLen, &rcount );
			if (ReadStatus != IC_OK)
			{
				printf("\nRead from Device Failed \n");
				printf("\nRetry count :  %d \n", iRetry);
				if (--iRetry)
				{
					Delay(1);
					retryFlag = 1;
					break;
				}
				else
				{
					printf("\nErr in Serial read\n");
					CardGlobalErrCode = 19;
					return ReadStatus;
				}
			}
			else
			{
				if (CurrentState == STX_FRAME)
				{
					if ( pucReadBuffer[0] != 0x60 && pucReadBuffer[0] != 0xE0 )
						continue;
				}

				pucReadBuffer += rcount;
				*rxlength += rcount;
				ReadLen -= rcount;
				iRetry = 3;
				retryFlag = 0;
			}
		}

		if (retryFlag == 1)
			continue;

		if (CurrentState == STX_FRAME)
			CurrentState = LENGTH_FRAME;
		else if (CurrentState == LENGTH_FRAME)
			CurrentState = DATA_FRAME;
		else if (CurrentState == DATA_FRAME)
			CurrentState = 0;
	}
	pucReadBuffer -= (*rxlength);
#endif

	printf("\nResponse <---  in CardTransmit: \n");
	for(i=0;i< *rxlength;i++)
		printf("%2.2x ",bRecvBuffer[i] );
	printf("\n");


	switch(bRecvBuffer[0])
	{
		case 0x60 :
			printf("\nCommand Success \n");
			break;

		case 0xE0 :
			printf("\nError in CardTransmit Command \n");
			CardGlobalErrCode = 22;
			return ICC_ERR;
		default :
			printf("\nInvalid Response \n");
			CardGlobalErrCode =23;
			return ICC_ERR;
	}
	if ((bRecvBuffer[0] != 0x60) || (bRecvBuffer[1] != 0x00))
	{
		printf("\nError in CardTransmit Command \n");
		CardGlobalErrCode = 24;
		return ICC_ERR_CNT_CMD;
	}

	RxChkSum = bRecvBuffer[bRecvBuffer[2] + 4];
	CalChkSum = calculate_crc (bRecvBuffer, bRecvBuffer[2] + 4);

	/*Verifying the Checksum for the Recieved Response */
	if ( RxChkSum != CalChkSum )
	{
		printf("\nChecksum Verification Failed \n");
		CardGlobalErrCode = 25;
		return ICC_ERR_RESP_CHECKSUM;
	}
	/* Updating Recieved Length for the Transmitted Command */
	//*rxlength = HeaderCnt + DataCnt;      

	return IC_OK;
}

IC_INT32 ipd_SerialReadCt( IC_INT32 fd, IC_UINT8 *pucBuffer, IC_INT32 iCount, IC_UINT32 * piNRead )
{
	IC_INT32 iTemp = 0;
	IC_INT32 iLocalRead = 0;
	IC_INT32 iReadCount=iCount;
	IC_INT32 retval;
	IC_UINT8 *pucReadBuffer=pucBuffer;

	struct timeval timeout;

	fd_set set;
	FD_ZERO(&set);
	FD_SET(fd, &set);


	*piNRead = 0;
	timeout.tv_sec=0;
	timeout.tv_usec=300000;

	if(iCount < 0)
		return IC_ERR_SERIAL_INVALID_READ_COUNT;

	if(iCount == 0)
		return IC_OK;

	while(iTemp != iReadCount)
	{

		retval=select(fd + 1, &set, NULL, NULL, &timeout);
		if(retval==-1)
		{
			perror("\n Error in Select : ");
			return(retval);
		}
		else /*if(retval == 1)*/
		{
			iLocalRead = read( fd, pucReadBuffer, iCount );
			if(iLocalRead == -1)
			{
				perror ("ReadSerial: ");
				*piNRead = iTemp;
				return IC_ERR_SERIAL_READ;
			}
			iTemp += iLocalRead;
			iCount -= iLocalRead;
			pucReadBuffer += iLocalRead;
		}
	}

	pucReadBuffer[iTemp]=0;
	pucReadBuffer-=iTemp;
	*piNRead = iTemp;

	return IC_OK;
}


IC_RETCODE GetCardResponse( IC_VOID *pst_dev_handle, IC_UINT8 *ResponseBuffer, IC_UINT8 ByteToRead )
{
	printf("\nInside GetCardResponse\n");

	IC_UINT32  ReturnStatus ;
	IC_UINT8 GetDataCard[] = GETCARD_RESPONSE ;
	IC_UINT8 ReadBuffer[50] = {0,};


	GetDataCard[4] = ByteToRead;

	ReturnStatus = CtSendData( CtSerialHandle, GetDataCard, ReadBuffer, GETCARD_RESPONSE_LEN);

	if(ReturnStatus != IC_OK)
	{
		printf("\nErr in Serial Transmit %s \n", __FUNCTION__);
		return ReturnStatus ;
	}

	memcpy(ResponseBuffer, ReadBuffer + 4, ByteToRead);


	return IC_OK;
}

IC_INT32 CtSendData( IC_VOID *pst_dev_handle, IC_UINT8 *Buffer, IC_UINT8 *ReadBuff, IC_UINT32 bytes )
{

	IC_UINT8 ReadBuffer[100]={0,};
	IC_UINT32 ReturnStatus,rxlength;
	IC_UINT32 count=0;
	IC_UINT8 ReadBlockCmdTxp[50] = {0,};


	ReadBlockCmdTxp[0] = SCOSTA_FRAME_START;
	ReadBlockCmdTxp[1] = 0x00;
	ReadBlockCmdTxp[2] = bytes;
	ReadBlockCmdTxp[3] = SCOSTA_APDU_COMMAND;

	memcpy(ReadBlockCmdTxp + 4, Buffer, bytes);

	ReadBlockCmdTxp[bytes + 4] =calculate_crc(ReadBlockCmdTxp, 4 + bytes);


	ReturnStatus = CardTransmit(CtSerialHandle->fd, ReadBlockCmdTxp, 5 + bytes, ReadBuffer, &rxlength);
	if (ReturnStatus != IC_OK)
	{
		printf("\nRead Command Failed \n");
		return ReturnStatus;
	}

	if((ReadBuffer[ReadBuffer[2] + 2] != 0x90) || ReadBuffer[ReadBuffer[2] + 3] != 0x00)
	{
		if((ReadBuffer[ReadBuffer[2] + 2] != 0x61) )
		{
			getErrStr(ReadBuffer[ReadBuffer[2] + 2], ReadBuffer[ReadBuffer[2] + 3] );
			printf("\nAPDU Error \n");
			return ICC_ERR_SCOSTA_CMD;
		}
	}


	/* Incrementing the Buffer Position Depending on the count and Updating the Number of Blocks Left */

	count = ReadBuffer[2] - 2;
	memcpy( ReadBuff, ReadBuffer, ReadBuffer[2] + 4 );


	return IC_OK;
}

IC_UINT8  getErrStr(IC_UINT8 SW1, IC_UINT8 SW2 )
{
	printf("**********ERROR CODES FROM SCOSTA STANDARD*********************************\n");
	switch(SW1)
	{
		case 0x62 :

			switch(SW2)
			{
				case 0x00:      printf("41.........No Information Given .......\n");
						CardGlobalErrCode = 41;
						return IC_OK;
				case 0x81:      printf("42.......Part of Returned data may be corrupted .......\n");
						CardGlobalErrCode = 42;
						return IC_OK;
				case 0x82:      printf("43.......End of file or record reached before reading Ne Bytes.... \n");
						CardGlobalErrCode = 43;
						return IC_OK;
				case 0x83:      printf("44.......Selected File Deactivated ......\n");
						CardGlobalErrCode = 44;
						return IC_OK;
				case 0x84:      printf("45.........File Control Information Not Formatted ...... \n");
						CardGlobalErrCode =45;
						return IC_OK;
				case 0x85:      printf("46.......Selected File in Termination State..... \n");
						CardGlobalErrCode = 46;
						return IC_OK;
				default:                printf("48.......Invalid Error Value ......\n");
							CardGlobalErrCode = 48;
							return IC_OK;
			}
		case 0x63 :
			switch(SW2)
			{
				case 0x00:      printf("49.......No Information Given .......\n");
						CardGlobalErrCode = 49;
						return IC_OK;
				case 0x81:      printf("50.......File filled up by the last Write .......\n");
						CardGlobalErrCode = 50;
						return IC_OK;

				default:                if(SW2 /0x10 == 0xc)
							{
								printf("51.......Wrong Key  Authentication.......\n");
								CardGlobalErrCode = 51;
							}
							else
							{
								printf("54.......Invalid Error Value .......\n");
								CardGlobalErrCode = 54;
							}
							return IC_OK;
			}
		case 0x64 :
			switch(SW2)
			{
				case 0x00:      printf("52.......Execution Error .......\n");
						CardGlobalErrCode = 52;
						return IC_OK;
				case 0x01:      printf("53.......Immediate Respnse required by the card .......\n");
						CardGlobalErrCode = 53;
						return IC_OK;
				default:                printf("516.......Invalid Error Value .......\n");
							CardGlobalErrCode = 516;
							return IC_OK;
			}
		case 0x65 :
			switch(SW2)
			{
				case 0x00:      printf("55.......No Information Given .......\n");
						CardGlobalErrCode = 55;
						return IC_OK;
				case 0x81:      printf("56.......Memory Failure .......\n");
						CardGlobalErrCode = 56;
						return IC_OK;
				default:                printf("57.......Invalid Error Value .......\n");
							CardGlobalErrCode = 57;
							return IC_OK;
			}
		case 0x68 :
			switch(SW2)
			{
				case 0x00:      printf("58.......No Information Given .......\n");
						CardGlobalErrCode = 58;
						return IC_OK;
				case 0x81:      printf("59.......Logical Channel not Supported ....... \n");
						CardGlobalErrCode = 59;
						return IC_OK;
				case 0x82:      printf("60.......Secure Messaging not Supported  .......\n");
						CardGlobalErrCode = 60;
						return IC_OK;
				case 0x83:      printf("61.......Last Command of the Chain Expected  .......\n");
						CardGlobalErrCode = 61;
						return IC_OK;
				case 0x84:      printf("62.......Command Chaining not Supported ....... \n");
						CardGlobalErrCode = 62;
						return IC_OK;
				default:                printf("63.......Invalid Error Value .......\n");
							CardGlobalErrCode = 63;
							return IC_OK;
			}
		case 0x69 :
			switch(SW2)
			{
				case 0x00:      printf("64.......No Information Given .......\n");
						CardGlobalErrCode = 64;
						return IC_OK;
				case 0x81:      printf("65.......Command incompatible with File Structure ....... \n");
						CardGlobalErrCode = 65;
						return IC_OK;
				case 0x82:      printf("66.......Secure Status not satisfied .......  \n");
						CardGlobalErrCode = 66;
						return IC_OK;
				case 0x83:      printf("67.......Authentication Method Blocked  ....... \n");
						CardGlobalErrCode = 67;
						return IC_OK;
				case 0x84:      printf("68.......Reference data not usable .......  \n");
						CardGlobalErrCode = 68;
						return IC_OK;
				case 0x85:      printf("69.......Conditions of use not satisfied  .......\n");
						CardGlobalErrCode = 69;
						return IC_OK;
				case 0x86 :     printf("70.......Command Not allowed (NO CURRENT EF) .......  \n");
						CardGlobalErrCode = 70;
						return IC_OK;
				case 0x87:      printf("71.......Expected Secure Messaging Data Objects Missing .......  \n");
						CardGlobalErrCode = 71;
						return IC_OK;
				case 0x88:      printf("72.Incorrect Secure Messaging Data Objects Missing \n");
						CardGlobalErrCode = 72;
						return IC_OK;
				default:                printf("73.......Invalid Error Value .......\n");
							CardGlobalErrCode = 73;
							return IC_OK;
			}

		case 0x6A :
			switch(SW2)
			{
				case 0x00:      printf("74.......No Information Given .......\n");
						CardGlobalErrCode = 74;
						return IC_OK;
				case 0x80:      printf("75.......Incorrect Parameters in the Command Data Field ....... \n");
						CardGlobalErrCode = 75;
						return IC_OK;
				case 0x81:      printf("76.......Function not Supported  .......\n");
						CardGlobalErrCode = 76;
						return IC_OK;
				case 0x82:      printf("77.......File or Application not Found .......  \n");
						CardGlobalErrCode = 77;
						return IC_OK;
				case 0x83:      printf("78.......Record not Found  .......  \n");
						CardGlobalErrCode = 78;
						return IC_OK;
				case 0x84:      printf("79.......Not Enough  Memory Space in the File .......  \n");
						CardGlobalErrCode = 79;
						return IC_OK;
				case 0x85:      printf("80.......Nc inconsistent with TLV Structure   ....... \n");
						CardGlobalErrCode = 80;
						return IC_OK;
				case 0x86:      printf("81.......Incorrect Parameters with Parametes P1-P2  ....... \n");
						CardGlobalErrCode = 81;
						return IC_OK;
				case 0x87:      printf("82.......Nc inconsistent with Parameters  P1-P2 .......\n");
						CardGlobalErrCode = 82;
						return IC_OK;
				case 0x88:      printf("83.......Reference dat or reference data not Found ..........\n");
						CardGlobalErrCode = 83;
						printf(".......(Exact meaning depending on the command .......\n");
						return IC_OK;
				case 0x89:      printf("85.......File Already Exists .......\n");
						CardGlobalErrCode = 85;
						return IC_OK;
				default:                printf("87.......Invalid Error Value .......\n");
							CardGlobalErrCode = 87;
							return IC_OK;
			}
			printf("********************************************************\n");
	}

	return IC_OK;
}

IC_UINT8 calculate_crc(IC_UINT8 *buf, IC_UINT32 len)
{
	IC_UINT8 store='\0';
	IC_UINT32 i;
	for(i = 0; i < len; i++ )
		store = (store)^(buf[i]);
	return store;
}      
