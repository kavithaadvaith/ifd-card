#ifndef ERROR_CODES_H
#define ERROR_CODES_H

#define IC_OK 0

#define IC_ERR_NETWORK_DOWN		100
#define IC_ERR_NETWORK_UNREACHABLE	101
#define IC_ERR_NETWORK_DROPPED		102


#define ERR_IMPROPER_KEY_LEN		 300
#define ERR_KEY_UPDATE_FAIL  		 301 
#define ERR_KEY_EX_SERVER   		 302

#define IC_ERR_TXNID			400
#define IC_ERR_CENTRL_SRV		401
#define IC_ERR_RESP			402
#define IC_ERR_STAN_RRN			403
#define IC_ERR_PINCODE_LEN		404
#define IC_ERR_INV_PINCODE		405
#define IC_ERR_INV_AUTH			406

#define IC_ERR_BIN			407
#define IC_ERR_PROTOBUFF		408
#define IC_ERR_ENCRYPT			409
#define IC_ERR_AUA_KEY			410
#define IC_ERR_AUA_CONFIG_FILE		411
#define IC_ERR_PARSE_CUST_NAME          415
#define IC_ERR_RM_LTS_FILE		416

#define IC_ERR_NORESP			417
#define IC_ERR_PARSE			418
#define IC_ERR_LTS_KEYEXP		419

/*SHG ERROR CODES*/

#define PARSE_ACC_INFO_FAIL     -865
#define ACC_INFO_FARME_FAIL     -866
#define FIRST_AUTH_FARME_FAIL   -867
#define FIRST_AUTH_PARSE_FAIL   -868
#define SECOND_AUTH_FARME_FAIL  -869
#define SECOND_AUTH_PARSE_FAIL  -870

#endif

