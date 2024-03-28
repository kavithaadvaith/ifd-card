#ifndef RFID_ERR_H
#define RFID_ERR_H
/********************ERROR CODES **********************/
#define ICC_ERR_DEVICE_FILE		6 
#define ICC_ERR_NO_TRANSPONDER          7
#define ICC_ERR_RFID_INIT      		8
#define ICC_ERR_SERIAL_CONFIGURE	9
#define ICC_ERR_WRITE_SERIAL_NUM     10
#define ICC_ERR_READ_SERIAL_NUM      11
#define ICC_ERR_WRITE_SERIAL_STANDARD 12
#define ICC_ERR_READ_SERIAL_STANDARD  13
#define ICC_ERR_WRITE_SEARCH_TXP  14
#define ICC_ERR_READ_SEARCH_TXP   15
#define ICC_ERR_WRITE_SELECT_TXP 16 
#define ICC_ERR_READ_SELECT_TXP  17
#define ICC_ERR_WRITE_TXPINFO    18
#define ICC_ERR_READ_TXPINFO     19
#define ICC_ERR_WRITE_FSINFO     20
#define ICC_ERR_READ_FSINFO      21
#define ICC_ERR_WRITE_SELECT_SEGMENT 22 
#define ICC_ERR_READ_SELECT_SEGMENT  23
#define ICC_ERR_CMDREAD_WRITE    24
#define ICC_ERR_CMD_READ         25
#define ICC_ERR_CMDWRITE_READ    26
#define ICC_ERR_CMD_WRITE    27
#define ICC_ERR_INVALID_CRC_READER_INFO 29
#define ICC_ERR_INVALID_CRC_STANDARD  30
#define ICC_ERR_INVALID_CRC_SEARCH_TXP 31
#define ICC_ERR_INVALID_CRC_SEL_TXP  32
#define ICC_ERR_INVALID_CRC_TXP_INFO 33
#define ICC_ERR_INVALID_CRC_FS_TXP  34 
#define ICC_ERR_INVALID_CRC_SELECT_SEG_TXP 35
#define ICC_ERR_INVALID_CRC_SEG_INFO_TXP 36
#define ICC_ERR_INVALID_CRC_SETAPPL 37
#define ICC_ERR_INVALID_CRC_READ 38
#define ICC_ERR_INVALID_CRC_WRITE 39
#define ICC_ERR_LOGIN_COMMAND_WRITE 40
#define ICC_ERR_CRC_READ_BLOCK 41
#define ICC_ERR_LOGIN_TXP 42
#define ICC_ERR_READ_BLOCK 43
#define ICC_ERR_CARD_LIST 44
#define ICC_ERR_CRC_CARD_LIST 45
#define ICC_ERR_WRITE_CARD_LIST_COMMAND 46
#define ICC_ERR_INSUFFICIENT_SPACE 47
#define ICC_ERR_INVALID_BLOCKNUMBER 48
#define ICC_ERR_WRITE_VALUE 49
#define ICC_ERR_READ_COMMAND_WRITE 50
#define ICC_ERR_COMMAND_READ_VALUE 51
#define ICC_ERR_READ_VALUE 52
#define ICC_ERR_COMMAND_WRITE_VALUE 53
#define ICC_ERR_COMMAND_WRITE_VALUE_INCREMENT 54
#define ICC_ERR_READ_VALUE_INCREMENT 55
#define ICC_ERR_COMMAND_WRITE_DECREMENT 56
#define ICC_ERR_READ_VALUE_DECREMENT 57
#define ICC_ERR_COMMAND_READ_BLOCK 58
#define ICC_ERR_COMMAND_WRITE_BLOCK 59
#define ICC_ERR_INVALID_CRC_READ_BLOCK 60
#define ICC_ERR_INVALID_CRC_WRITE_BLOCK 61
#define ICC_ERR_INVALID_CRC_READ_VALUE 62
#define ICC_ERR_INVALID_CRC_WRITE_VALUE 63
#define ICC_ERR_INVALID_CRC_INCREMENT_VALUE 64
#define ICC_ERR_INVALID_CRC_DECREMENT_VALUE 65
#define ICC_ERR_NO_MEMORY_SPACE 66
#define ICC_ERR_INVALID_WRITE_ALIGNMENT 67
#define ICC_ERR_WRITE_BLOCK 68
#define ICC_ERR_SERIAL_CLOSE 69
#define ICC_ERR_INVALID_TID 70
#define ICC_ERR_INVALID_MERACC_NUM 71
#define ICC_ERR_INVALID_USERACC_NUM 72
#define INVALID_DECIMAL_VALUE 73
#define ICC_ERR_INVALID_USER_CARD 74
#define ICC_ERR_INVALID_MER_CARD 75
#define ICC_ERR_INVALID_TLC_ID 76
#define ICC_ERR_INVALID_TELNO 77
#define ICC_ERR_MEMORY_SPACE 78
#define ICC_ERR_INVALID_ACCOUNT 79
#define ICC_ERR_INVALID_BALANCE 80
#define ICC_ERR_INVALID_IMAGE 81
#define ICC_ERR_INVALID_COUNT 82
#define ICC_ERR_INVALID_TERMINAL_NUM  83
#define ICC_ERR_BINARY_CMD_WRITE 84
#define ICC_ERR_INVALID_MERID_NUM 85
#define ICC_ERR_ACC_COUNT 86
#define ICC_ERR_ACK 87
#define ICC_ERR_CARD_INFO 88
#define ICC_ERR_LEN 88
#define ICC_ERR_CMD_AUTH_TXP 89
#define ICC_ERR_INVALID_CRC_AUTH_TXP 90 
#define ICC_ERR_CMD_READ_TXP 91
#define ICC_ERR_CMD_WRITE_TXP 92
#define ICC_ERR_UID_SELECT 93
/************************************************NFC ERRORS SET ********************************/

#define ICC_ERR_NFC_SWITCH_OFF_CMD 94 
#define ICC_ERR_ACK_SWITCH_OFF  95
#define ICC_ERR_LEN_SWITCH_OFF 96
#define ICC_ERR_INVALID_CRC_SWITCH_OFF  97 
#define ICC_ERR_NFC_SWITCH_ON_CMD 98
#define ICC_ERR_ACK_SWITCH_ON 99
#define ICC_ERR_LEN_SWITCH_ON 100
#define ICC_ERR_INVALID_CRC_SWITCH_ON 101
#define ICC_ERR_NFC_SWITCH_OFF 102
#define ICC_ERR_NFC_SWITCH_ON 103
#define ICC_ERR_CARD_SELECT 104
#define ICC_ERR_INVALID_CRC_GETCARDLIST  105
#define ICC_ERR_SEL_APPLET 106
#define ICC_ERR_READ_SMX 107
#define ICC_ERR_READ_MIFARE 108
#define ICC_ERR_CARD_INVALID_CHOICE 109
/*****************************************************************************/
#define ICC_ERR_WRITE_CARD_TYPE_COMMAND 110
#define ICC_ERR_CARD_TYPE 111
#define ICC_ERR_WRITE_SMX 112
#define ICC_ERR_WRITE_MIFARE 113
#define ICC_ERR_CMD_APPLET 114
#define ICC_ERR_CARD_NOT_PRESENT 115
#define ICC_ERR_SELECT_DEVICE 116
#define ICC_ERR_NFC_CONFIG 117
#define ICC_ERR_LEN_CONFIG 118
#define ICC_ERR_INVALID_CRC_CONFIG 119
#define ICC_ERR_CRYPTO 120
#define ICC_ERR_ENCRYPTION 121
#define ICC_ERR_DECRYPTION 122
#define ICC_ERR_INVALID_SECTOR 123
#define ICC_ERR_MEMORY 124
//#define IC_ERR_NULL_PARAMS 125
#define ICC_ERR_WRITE_EEPROM 126
#define ICC_ERR_READ_EEPROM 127

#define ICC_ERR_CARD_ABSENT 142
#define ICC_ERR_CARD_POWERUP 143 
#define ICC_ERR_RESP_CHECKSUM 144 
#define ICC_ERR_SCOSTA_CMD 145
#define ICC_ERR_CNT_CMD 146
#define ICC_ERR_CONTACT_SERIAL_INIT 147
#define ICC_ERR_NOCARDFOUND 148
#define ICC_ERR_CHKSUM 149


#define ICC_ERR_CMD_PN65_FAIL 130
#define ICC_ERR_CMD_CHIP_SERIAL_NUM 131
#define ICC_ERR_INVALID_PARAMETER 123
#define ICC_ERR 159
#define ICC_ERR_NFC_BAUD 155
#define ICC_ERR_CMD_SELECT_MASTER_FILE 125 
#define ICC_ERR_CMD_CARD_FAILURE 127
#define ICC_ERR_CMD_PN65 128
#define ICC_ERR_INVALID_HANDLE 122
#define ICC_ERR_LEN_CHECK_SUM 124
#define ICC_ERR_DATA_CHECK_SUM 126
#define ICC_ERR_PIN_FAILURE 149
#define ICC_ERR_SIMUL_DESFIRE 157
#define ICC_ERR_CALCHECKSUM 158 
#define ICC_ERR_AUTH 160
#define ICC_ERR_MUTUAL_AUTH 148
#define ICC_ERR_FP 140

#define ICC_ERR_FILE_OPEN 162
#define ICC_ERR_FILE_READ 163
#define ICC_ERR_FILE_SIZE 164
#define IC_MIFARE_UNSECURED 165

#define CARD_EMPTY		166
#define CARD_NOT_EMPTY 	167
#define ICC_ERR_FILE_NOT_FOUND 168
#define PARTIAL_PERSO 169
#define KEY_MISMATCH  170
#define ICC_ERR_CARD_DEAVTIVE 171
#define IC_ERR_ZERO_ACC_ADDED 172
#define ICC_COND_NOT_SATISFIED 173

#define ICC_ERR_BC_CARD_LIST	174
#define ICC_ERR_BC_EXTRA_CARD	175
#define ICC_ERR_BC_NO_CARD		176
#define ICC_ERR_BC_SELECT_FAIL	177
#define ICC_ERR_BC_CARD_TYPE	178
#define ICC_ERR_BC_CNF			179
#define ICC_ERR_BC_POWER_UP		180
#define ICC_ERR_BC_SEL_APP		181
#define ICC_ERR_BC_INVALID		182

#define ICC_ERR_BBC_CARD_LIST	183
#define ICC_ERR_BBC_EXTRA_CARD	184
#define ICC_ERR_BBC_NO_CARD		185
#define ICC_ERR_BBC_SELECT_FAIL	186
#define ICC_ERR_BBC_CARD_TYPE	187
#define ICC_ERR_BBC_CNF			188
#define ICC_ERR_BBC_POWER_UP		189
#define ICC_ERR_BBC_SEL_APP		190
#define ICC_ERR_BBC_INVALID		191

#define ICC_ERR_READ_DESFIRE 153
#define ICC_ERR_WRITE_DESFIRE 154

#define ICC_ERR_PROPER_CARD		201
#define ICC_ERR_IMPROPER_DATA 202

#if 1 //added Hari for E_CAMPUS
#define ICC_ERR_CREATE_FILE_SYTEM 205
#define ICC_ERR_WRITE_DUMMY_TXION 206
#define ICC_ERR_LOADING_DATA_CARD 207
#define ICC_ERR_SECURING_CARD 208
#define ICC_ERR_AUTH_CARD 209
#define ICC_ERR_ACTIVATE_FS 210
#endif
/*SHG group Card*/
#define ICC_ERR_VERSION_MISMATCH 211
#define ICC_ERR_INVALID_NO_OF_PEOPLE_COUNT 212
#define ICC_ERR_WRONG_FPFILE_ID 213

/**/
#ifdef _OS_WINDOWS_
#define __FUNCTION__ "function"
#else
#define __FUNCTION
#endif
#endif
