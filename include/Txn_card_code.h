#ifndef TXN_CARD_CODE_H
#define TXN_CARD_CODE_H

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "ipd_Serial.h"
//#include "gl11.h"


#define  IC_RETCODE int
typedef char              IC_INT8 ;
typedef int               IC_INT32 ;

typedef unsigned char     IC_UINT8 ;
typedef void              IC_VOID ;
typedef unsigned short    IC_UINT16 ;

typedef unsigned int      IC_UINT32 ;
typedef unsigned long     IC_UINT64 ;
typedef IC_VOID * IC_DEV_HANDLE;


#define IC_OK  0
#define IC_ERR -1

#define IC_ERR_SERIAL_OPEN                                                      -1
#define ICC_ERR_CARD_ABSENT 142
#define ICC_ERR_CARD_POWERUP 143 

#define ICC_ERR_CONTACT_SERIAL_INIT 147
#define ICC_ERR_RFID_INIT               8
#define IC_ERR_SERIAL_INVALID_CHSIZE                                    -6
#define ERR_GAT_CARD_LIST_FAILED        -551
#define IC_ERR_SERIAL_INVALID_WRITE_COUNT                               -5
#define IC_ERR_SERIAL_READ                                                              -10
#define ICC_ERR_CARD_DEAVTIVE 171
#define ICC_ERR_CMD_CHIP_SERIAL_NUM 131
#define IC_ERR_SERIAL_WRITE                                                     -11
#define SCOSTA_GET_CHIP_SERIAL_CT {0x60,0x00,0x05,0x00,0x00,0xCA,0x02,0x46,0x10,0xFB}
#define SCOSTA_GET_CHIP_SERIAL_LEN_CT 10
#define SCOSTA_POWERUP_3V  {0x60,0x00,0x01,0x6D,0x01,0x0D}
#define SCOSTA_CARD_POWERUP_3V_CMD_LEN 6
#define SCOSTA_FRAME_START 0x60
#define SCOSTA_APDU_COMMAND 0x00
#define UID_LEN        32
#define  UID_BUFLEN       256
#define ICC_ERR_SCOSTA_CMD 145
#define GETCARD_RESPONSE  { 0x00, 0xC0, 0x00, 0x00, 0x00}
#define GETCARD_RESPONSE_LEN 5
#define IC_ERR_SERIAL_INVALID_READ_COUNT  				-9
#define ICC_ERR_RESP_CHECKSUM 144 
#define ICC_ERR_CNT_CMD 146
#define UID_LENGTH      12      
#define   LENGTH_FRAME  0x02
#define   DATA_FRAME    0x03
#define   STX_FRAME             0x01
#define ICC_ERR 159
#define IC_ERR_SERIAL_INVALID_STOPBIT 					-8
#define IC_ERR_SERIAL_INVALID_READ_COUNT  				-9
#define IC_ERR_SERIAL_PORT_SETTING  					-3
#define ICC_ERR_SERIAL_CONFIGURE	9
#define  MAX_CARDSDETECT  8
#define IC_ERR_SERIAL_SET_ATTRIB  						-2
#define IC_ERR_SERIAL_PORT_SETTING  					-3
#define IC_ERR_SERIAL_INVALID_PARITY_VAL 				-7


#define ACC_LEN                                     17
#define MAX_AC_ON_CARD 4
#define EXPDATE_LEN    8
#define TERMINAL_ID_LEN        13
#define MAX_SCHEME_ON_ACCOUNT 4
#define CARDNO_LEN                              17
#define NAME_LEN       20
#define REQID_LEN      17
#define BANK_ACC_LEN                            20
#define MAX_CARDS 5
#define MAX_UID_SIZE 20


typedef struct dev_handle
{
	IC_INT32 fd; // File Descriptor - USB to Serial Device
	IC_INT8 *fb; // Frame Buffer for Display
	IC_UINT32 CardType;
	//Common Resources to be added here..
}ST_DEV_HANDLE;


typedef enum devtype
{
	E_IMTS,
	E_IMTS_V511,
	E_GNOME
}E_DEVTYPE;





typedef struct st_card
{
	IC_UINT8 ucRfidStandard;
	IC_UINT8 ucUidLen;
	IC_UINT8 aucUidBuf[UID_LEN];

}ST_CARD;

typedef struct rfid_dev_handle
{
	int fd;
	IC_INT8 *fb; // Frame Buffer for Display
	unsigned int CardType;
}ST_RFID_DEV_HANDLE;

enum cards{MIFARE_CARD=1, SMARTMX_1, SMARTMX_2, SMARTMX_3, SCOSTA, BC_SAM, BC_INTSAM, DESFIRE};

enum
{
	CONTACT_CARD = 1,
	CONTACTLESS_CARD,
	VIRTUAL_CARD
};

/************************************************************************************/
typedef enum eReqTyp
{
	E_DBN,
	E_DEN,
	E_SBL,
	E_OFFLINE,  //added by rajendra 10.07.07
	E_ONLINE,  //added by rajendra 10.07.07
	E_ONL_WD, /* Online Transaction Type is withdraw */
	E_ONL_DEP, /* Online Transaction Type is deposit */
	E_OFLSBL,
	E_DISBURSE_START,
	E_DISBURSE_CLOSE,
	E_PAYMENT_START,
	E_PAYMENT_CLOSE,
	E_BC_ONL_WD,
	E_BC_ONL_DEP

}E_REQTYP;

typedef struct group_Cust_info
{
	IC_UINT8 ucAccType;
	IC_INT8 acCardName[21];
	IC_INT8 aucCardNum[30];
	IC_UINT32 uiFpFileId;
	//IC_INT8 acAccNo[31];

}ST_GROUP_CUST_INFO;



typedef enum eReqStat
{
	E_INIT,
	E_PEND,
	E_EOR,
	E_PUSH
}E_REQSTAT;



typedef enum eCardType
{   
	E_MERCH,   
	E_USR,     
	E_ADMIN,
	E_BRANCH_MERCH    
}E_CARDTYPE;

typedef enum eCardStat /* Added by Manish */
{
	E_CARD_ACTIVE = 1,
	E_CARD_BLOCKED,
	E_CARD_FP_ABSENT,
	E_CARD_FP_PRESENT,
	E_CARD_ACTIVE_FAIL
}E_CARDSTAT;


typedef struct SchemeList
{
	IC_UINT8  ucSchemeID; // 1st Nibble: Acc Sno, 2nd Nibble: Acc Type
	IC_UINT16 uhSchemeIndex;
}ST_SCHEME_INFO;


/*********************************ACC Info***************************/
typedef struct ACCInfo /* Added by KiranP 27-02-2007 */
{
	IC_UINT8 ucACCType; // 1st Nibble: Acc Sno, 2nd Nibble: Acc Type
	IC_UINT16 uhACCIndex;
	IC_INT8 aucACCNO[ACC_LEN]; 
}ST_ACCINFO;

/*********************************CardBasic Det***************************/
typedef struct CardInfo /* Added by KiranP 27-02-2007 */
{
	IC_UINT8   ucVersion;
	E_CARDTYPE eCardType;
	IC_UINT8   ucAccCount;
	E_CARDSTAT eCardStatus;
	ST_ACCINFO astAccInfo[MAX_AC_ON_CARD]; 
	IC_INT32   OSType;
}ST_CARDINFO;

typedef enum AccType
{
	//E_CPDP = 0x0001,
	//E_CGCC = 0x0002
	E_CARD_DEPT = 0x0001,
	E_CARD_LOAN = 0x0002,
	E_GOV
} E_ACCTYPE;

typedef enum eACCStat /* Modified by Kiran P 27-1-07, changed eCardStat to eACCStat */
{
	E_ACTIVE = 1,
	E_CREDIT_FREEZE,  //added on 01-08-07
	E_DEBIT_FREEZE,   //added on 01-08-07
	E_CREDIT_FREEZE_TAG,     // added on 01-08-07
	E_DEBIT_FREEZE_TAG,      // added on 01-08-07
	E_CR_DB_FREEZE,          // added on 01-08-07
	E_CR_DB_FREEZE_TAG,      // added on 01-08-07
	E_BLOCKED,
	E_CLOSED
}E_ACCSTAT;


/*********************************AccBasic Det***************************/
typedef struct AccBasicDet /* Modified by KiranP 27-02-2007, changed CardBasicDet to AccBasicDet */
{   
	IC_UINT8   ucVersion;     
	E_ACCTYPE  eACCType;                                //ACCType  : E_DEPT or E_LOAN
	IC_UINT8   ucAccSubType;       // FIXED_LOAN,RUNNING_LOAN,SB_DEPT   
	E_ACCSTAT  eAccStatus;   //ACCSTATUS: Active or Blocked or Credit freezed or Debit Freeze 
	IC_UINT8   aucExpiryDate[EXPDATE_LEN];
	IC_INT8   aucTerminalID[TERMINAL_ID_LEN+7];
	IC_UINT8   ucSchemeCount;
	ST_SCHEME_INFO  astSchemeInfo[MAX_SCHEME_ON_ACCOUNT];
}ST_ACC_BASICDET;    

typedef struct AccGenDet /* Modified by KiranP 27-02-2007, changed CardGenDet to AccGenDet */
{
	IC_INT8   aucAcNo[ACC_LEN+1];
	IC_INT8   aucCardNo[CARDNO_LEN+1];
	IC_UINT8   aucAcHolderName[NAME_LEN +1];
	E_REQSTAT  eReqStatus;
	E_REQTYP   eReqType;
	IC_INT8   aucReqID[REQID_LEN];
	ST_CARD    stCardUID;  //Place Holder for Application 
	IC_UINT8   aucBankAcNo[BANK_ACC_LEN + 1]; //Place Holder for Application
	//IC_UINT8   aucBankAcNo[BANK_ACC_LEN + 3]; //Place Holder for Application " L" or " D"
	//IC_UINT8   aucDirPath[PATH_LENGTH + 1]; //Place Holder for Application 
	//IC_UINT8   aucGPRSStr[MAX_GPRS_LEN + 1]; //Place Holder for Application 
	IC_UINT16  ucValMin;//Only in Case of Recu Loan Account the fields gets filled in  ; Minimum Value of the Installment
	IC_UINT32  ucValMax;// Maximum value of the installment
	IC_UINT16  ucValMultiple; //Multiple Value that to be allowed for the installment
	IC_UINT8 ucIMax;//Maximum Installments permitted for the Rec Account 

}ST_ACC_GENDET;

/***************************USER_BALANCE DET******************************/
typedef struct UsrBalDet
{
	//IC_INT32 lBalance;
	//IC_UINT8 ucPaisa;
	//ST_BALANCE stBalance;
	IC_INT32 iLowerBalAllowd;   /* +ve, -ve or 0 */
	IC_UINT32 iDepLimPerDay;    /* +ve */
	IC_UINT32 iDepInDay;        /* +ve */
	IC_UINT32 iWithDrawLimPerDay;/* +ve */
	IC_UINT32 iWithDrawInDay;    /* +ve */
	IC_UINT32 iDepLimPerTrans;   /* +ve */
	IC_UINT32 iWdrLimPerTrans;   /* +ve */
}ST_USRBAL_DET;

typedef struct UserTransStatus
{
	IC_UINT8 ucTotalTrans;          //#LT Lim   //Added to Unsigned char Manish 
	IC_UINT8 ucCountTrans;          //#LT Cnt //Added to Unsigned char Manish 
	IC_UINT8 ucLastTransIndex;      //#LT Indx //Added to Unsigned char Manish 
	IC_UINT8 ucTransCountPerDay;    //#TxCnt //Added to Unsigned char Manish 
	IC_UINT8 ucTransLimPerDay;      //#Tx lim
	IC_UINT16 uhLangOpt;      
}ST_USR_TRANS_STAT;

typedef struct Additional_group_info 
{
	IC_UINT8 ucGroupVersion;
	IC_UINT8 ucNoOfPeople;
	ST_GROUP_CUST_INFO astGroupCustInfo[6];
}ST_ADDITIONAL_GROUP_INFO;


typedef enum eTransType
{
#if 0
	E_WITHDRAW,
	E_DEPOSIT, 
	E_SYNCBAL,   /* Added by manish 7-1-07 */
	E_ONLINE_WD, /* Only merchant card will have this. User will have E_WITHDRAW */
	E_ONLINE_DEP, /* Only merchant card will have this. User will have E_DEPOSIT */
	E_OFF_LINE_OVERRIDE_WITHDRAW,
	E_OFF_LINE_OVERRIDE_DEPOSIT,
	E_OFF_LINE_DISBURSEMENT,
	E_OFF_LINE_DISBURS_OVERRIDE,                    // Print Code - 0104 
	E_OFF_LINE_DISBURS_CRDNT_READ,                  // Print Code - 0102
	E_OFF_LINE_DISBURS_CARD_NOT_ISSUED,              // Print Code - 0103
	E_FTR_WITHDRAW,
	E_FTR_DEPOSIT,
	E_OFF_LINE_PAYMENT,                             //Print Code - 0105
	E_OFF_LINE_PAYMENT_OVERRIDE,                    // Print Code - 0106 
	E_OFF_LINE_PAYMENT_CRDNT_READ,                  // Print Code - 0107
	E_OFF_LINE_PAYMENT_CARD_NOT_ISSUED,              // Print Code - 0108
	E_ONL_DISBURSEMENT,

	E_IMPS_AEPS_ONUS_FTR = 20, /* Added by dileep for IMPS transactions */
	E_IMPS_IFIS_ONUS_FTR,
	E_IMPS_RUPAY_ONUS_FTR,
	E_IMPS_SCOT_ONUS_FTR,
	E_IMPS_BC_DEPOSIT,

	E_IFIS_ONUS_WITHDRAW = 32,
	E_IFIS_ONUS_DEPOSIT,
	E_IFIS_ONUS_FTR,
	E_IFIS_OFFUS_WITHDRAW,
	E_IFIS_OFFUS_DEPOSIT,
	E_IFIS_OFFUS_FTR,
	E_IFIS_ONUS_BAL_ENQ,
	E_IFIS_OFFUS_BAL_ENQ,
	E_IFIS_SC_WITHDRAW , //For SmartCard Online
	E_IFIS_SC_DEPOSIT,
	E_TP_DEPOSIT, //For non FI third party deposit
	E_SP_DEPOSIT, //For FI BC settlement
	E_SP_WITHDRAW, //For FI BC settlement
	E_SCP_WITHDRAW, //For Smart card pin based txn
	E_SCP_DEPOSIT, //For Smart card pin based txn

	E_ONUS_WITHDRAW = 48,
	E_ONUS_DEPOSIT,
	E_ONUS_FTR,
	E_OFFUS_WITHDRAW,
	E_OFFUS_DEPOSIT,
	E_OFFUS_FTR,
	E_ONUS_BAL_ENQ,
	E_OFFUS_BAL_ENQ,

#endif
	E_SHG_AEPS_WITHDRAW, //For shg Double auth -ganesha
}E_TRANSTYPE;



typedef struct UserCardDet
{
	ST_CARDINFO stCardInfo;
	ST_ACCINFO stAccInfo;
	ST_ACC_BASICDET stAccBasicDet;
	ST_ACC_GENDET stAccGenDet;
	ST_USRBAL_DET stUsrBalDet;
	ST_USR_TRANS_STAT stUsrTransStat;
	ST_ADDITIONAL_GROUP_INFO st_grp_cust_info;
	ST_CARD st_carduid;

	//ST_FP_DET stFPDet;
	IC_UINT8 ucBCOverride;
//	E_TRANSTYPE eDisTxnType;
	//ST_FINGERPRINT_DATA st_cust_fp; //Added by Dileep for SCOT.
	//ST_SELECTED_FP stSelectedFP;
	//ST_CUST_CARD_FP stCustFPInCard[11];
	//ST_TXN_PRINT_DET st_PrintDet;

}ST_USR_CARD_DET;

/*############################### FUNCTION DECLARATIONS ##########################################*/


int InitializeCard();
IC_RETCODE ContactInit ( IC_VOID *pst_dev_handle, E_DEVTYPE devtype );
IC_RETCODE ContactSerialInit_imts_v511 ( IC_VOID *pst_dev_handle, E_DEVTYPE devtype);
int SetBaudrate_gl14(void);
IC_RETCODE  ContactCardPowerOff ( IC_VOID *pst_dev_handle );
IC_INT32 ipd_SerialOpen(IC_UINT8 *pc_port, IC_INT32 i_mode);
IC_INT32 ipd_SerialConfigure( IC_INT32 fd, ST_CONFIGURE_PORT *pst_portsettings );
IC_INT32 ipd_SerialWrite( IC_INT32 fd,IC_UINT8 *pucBuffer,IC_INT32 iCount ,IC_UINT32 * piNWrite );
IC_RETCODE ContactSerialDeInit ( IC_VOID *pst_dev_handle);

IC_RETCODE GetCardResponse(IC_VOID *pst_dev_handle, IC_UINT8 *ResponseBuffer, IC_UINT8 ByteToRead);
IC_UINT8 calculate_crc(IC_UINT8 *buf, IC_UINT32 len);
IC_UINT8  getErrStr(IC_UINT8 SW1, IC_UINT8 SW2 );
/*
int CheckIfCardPresent();
IC_RETCODE IsSameCardPresent(IC_DEV_HANDLE lg_devHandle, ST_USR_CARD_DET st_usrdet);
IC_RETCODE  ContactGetCardList ( IC_VOID *pst_dev_handle, IC_UINT8 *bufferuids, IC_UINT32 *pi_length );
IC_RETCODE select_card_vt(IC_VOID *pst_dev_handle, IC_UINT32 CardNum);
IC_RETCODE  ContactSerialGetCardList ( IC_VOID *pst_dev_handle, IC_UINT8 *bufferuids, IC_UINT32 *pi_length );
IC_RETCODE  IsContactCardActive( IC_VOID *pst_dev_handle );
IC_RETCODE  ContactCardPowerUp ( IC_VOID *pst_dev_handle );
IC_RETCODE CardTransmit( IC_UINT32 fd, IC_UINT8 *bSendBuffer, IC_UINT32 txlength,IC_UINT8 *bRecvBuffer, IC_UINT32 *rxlength );
IC_INT32 ipd_SerialReadCt( IC_INT32 fd, IC_UINT8 *pucBuffer, IC_INT32 iCount, IC_UINT32 * piNRead );
IC_INT32 CtSendData( IC_VOID *pst_dev_handle, IC_UINT8 *Buffer, IC_UINT8 *ReadBuff, IC_UINT32 bytes );


*/


#endif
