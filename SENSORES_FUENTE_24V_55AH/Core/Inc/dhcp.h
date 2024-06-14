/*
 * dhcp.h
 *
 *  Created on: June 14, 2024
 *      Author: A.R.T
 */

#ifndef __DHCP_H__
#define __DHCP_H__

#include "wizchip_conf.h"
#include "socket.h"
#include "w5500.h"
#include <string.h>

#define DHCP_RET_NONE        0
#define DHCP_RET_TIMEOUT     1
#define DHCP_RET_ERR         2
#define DHCP_RET_UPDATE      3
#define DHCP_RET_CONFLICT    4
#define DHCP_RET_RELEASE     5

#define DHCP_MSG_LEN         236

#define DHCP_MSG_OFFER       2
#define DHCP_MSG_REQUEST     3
#define DHCP_MSG_DECLINE     4
#define DHCP_MSG_ACK         5
#define DHCP_MSG_NAK         6
#define DHCP_MSG_RELEASE     7
#define DHCP_MSG_INFORM      8

#define DHCP_FLAG_BROADCAST  0x8000

#define DHCP_MAGIC_COOKIE    0x63825363

#define DHCP_DISCOVER        1
#define DHCP_OFFER           2
#define DHCP_REQUEST         3
#define DHCP_DECLINE         4
#define DHCP_ACK             5
#define DHCP_NAK             6
#define DHCP_RELEASE         7
#define DHCP_INFORM          8

#define DHCP_HTYPE_ETH       1

#define DHCP_HLEN_ETH        6

#define DHCP_SPORT           67
#define DHCP_CPORT           68

#define DHCP_SNAME_LEN       64
#define DHCP_FILE_LEN        128
#define DHCP_OPT_LEN         312

#define DHCP_FLAG_UNICAST    0x0000
#define DHCP_FLAG_BROADCAST  0x8000

#define DHCP_OPTION_PAD      0
#define DHCP_OPTION_SUBNET   1
#define DHCP_OPTION_ROUTER   3
#define DHCP_OPTION_DNS      6
#define DHCP_OPTION_HOSTNAME 12
#define DHCP_OPTION_REQ_IP   50
#define DHCP_OPTION_LEASE    51
#define DHCP_OPTION_MSG_TYPE 53
#define DHCP_OPTION_SERVER_ID 54
#define DHCP_OPTION_PARAM_REQ 55
#define DHCP_OPTION_END      255

typedef struct dhcp_msg {
    uint8_t op;
    uint8_t htype;
    uint8_t hlen;
    uint8_t hops;
    uint32_t xid;
    uint16_t secs;
    uint16_t flags;
    uint8_t ciaddr[4];
    uint8_t yiaddr[4];
    uint8_t siaddr[4];
    uint8_t giaddr[4];
    uint8_t chaddr[16];
    uint8_t sname[DHCP_SNAME_LEN];
    uint8_t file[DHCP_FILE_LEN];
    uint32_t cookie;
    uint8_t options[DHCP_OPT_LEN];
} dhcp_msg_t;

void DHCP_init(uint8_t s, uint8_t *buf);
int8_t DHCP_run(void);
void DHCP_stop(void);
void DHCP_reset(void);
void getIPfromDHCP(uint8_t *ip);
void getGWfromDHCP(uint8_t *gw);
void getSNfromDHCP(uint8_t *sn);

#endif /* __DHCP_H__ */
