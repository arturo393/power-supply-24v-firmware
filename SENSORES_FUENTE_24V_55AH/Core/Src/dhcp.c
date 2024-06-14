/*
 * dhcp.c
 *
 *  Created on: June 14, 2024
 *      Author: A.R.T
 */

#include "dhcp.h"

static uint8_t DHCP_SOCKET;
static uint8_t *DHCP_BUFFER;
static uint8_t DHCP_state;
static uint32_t DHCP_XID;
static uint8_t DHCP_SERVER_IP[4];
static uint8_t DHCP_LEASED_IP[4];
static uint8_t DHCP_SUBNET_MASK[4];
static uint8_t DHCP_GATEWAY[4];
static uint8_t DHCP_DNS[4];

void DHCP_init(uint8_t s, uint8_t *buf) {
    DHCP_SOCKET = s;
    DHCP_BUFFER = buf;
    DHCP_XID = 0x12345678;
    DHCP_state = DHCP_DISCOVER;
    memset(DHCP_SERVER_IP, 0, 4);
    memset(DHCP_LEASED_IP, 0, 4);
    memset(DHCP_SUBNET_MASK, 0, 4);
    memset(DHCP_GATEWAY, 0, 4);
    memset(DHCP_DNS, 0, 4);
}

static void send_DHCP_DISCOVER(void) {
    dhcp_msg_t *dhcp_msg = (dhcp_msg_t *)DHCP_BUFFER;
    memset(dhcp_msg, 0, sizeof(dhcp_msg_t));
    dhcp_msg->op = 1; // BOOTREQUEST
    dhcp_msg->htype = DHCP_HTYPE_ETH;
    dhcp_msg->hlen = DHCP_HLEN_ETH;
    dhcp_msg->hops = 0;
    dhcp_msg->xid = htonl(DHCP_XID);
    dhcp_msg->flags = htons(DHCP_FLAG_BROADCAST);
    memcpy(dhcp_msg->chaddr, net_info.mac, 6);
    dhcp_msg->cookie = htonl(DHCP_MAGIC_COOKIE);
    uint8_t *opt_ptr = dhcp_msg->options;
    *opt_ptr++ = DHCP_OPTION_MSG_TYPE;
    *opt_ptr++ = 1;
    *opt_ptr++ = DHCP_DISCOVER;
    *opt_ptr++ = DHCP_OPTION_END;
    sendto(DHCP_SOCKET, (uint8_t *)dhcp_msg, sizeof(dhcp_msg_t), DHCP_SERVER_IP, DHCP_SPORT);
}

static void send_DHCP_REQUEST(void) {
    dhcp_msg_t *dhcp_msg = (dhcp_msg_t *)DHCP_BUFFER;
    memset(dhcp_msg, 0, sizeof(dhcp_msg_t));
    dhcp_msg->op = 1; // BOOTREQUEST
    dhcp_msg->htype = DHCP_HTYPE_ETH;
    dhcp_msg->hlen = DHCP_HLEN_ETH;
    dhcp_msg->hops = 0;
    dhcp_msg->xid = htonl(DHCP_XID);
    dhcp_msg->flags = htons(DHCP_FLAG_BROADCAST);
    memcpy(dhcp_msg->chaddr, net_info.mac, 6);
    dhcp_msg->cookie = htonl(DHCP_MAGIC_COOKIE);
    uint8_t *opt_ptr = dhcp_msg->options;
    *opt_ptr++ = DHCP_OPTION_MSG_TYPE;
    *opt_ptr++ = 1;
    *opt_ptr++ = DHCP_REQUEST;
    *opt_ptr++ = DHCP_OPTION_REQ_IP;
    *opt_ptr++ = 4;
    memcpy(opt_ptr, DHCP_LEASED_IP, 4);
    opt_ptr += 4;
    *opt_ptr++ = DHCP_OPTION_SERVER_ID;
    *opt_ptr++ = 4;
    memcpy(opt_ptr, DHCP_SERVER_IP, 4);
    opt_ptr += 4;
    *opt_ptr++ = DHCP_OPTION_END;
    sendto(DHCP_SOCKET, (uint8_t *)dhcp_msg, sizeof(dhcp_msg_t), DHCP_SERVER_IP, DHCP_SPORT);
}

int8_t DHCP_run(void) {
    uint8_t msg_type = 0;
    dhcp_msg_t *dhcp_msg = (dhcp_msg_t *)DHCP_BUFFER;
    if (recvfrom(DHCP_SOCKET, DHCP_BUFFER, sizeof(dhcp_msg_t), DHCP_SERVER_IP, DHCP_SPORT) > 0) {
        uint8_t *opt_ptr = dhcp_msg->options;
        while (*opt_ptr != DHCP_OPTION_END) {
            switch (*opt_ptr) {
                case DHCP_OPTION_MSG_TYPE:
                    msg_type = *(opt_ptr + 2);
                    break;
                case DHCP_OPTION_SERVER_ID:
                    memcpy(DHCP_SERVER_IP, opt_ptr + 2, 4);
                    break;
                case DHCP_OPTION_SUBNET:
                    memcpy(DHCP_SUBNET_MASK, opt_ptr + 2, 4);
                    break;
                case DHCP_OPTION_ROUTER:
                    memcpy(DHCP_GATEWAY, opt_ptr + 2, 4);
                    break;
                case DHCP_OPTION_DNS:
                    memcpy(DHCP_DNS, opt_ptr + 2, 4);
                    break;
                case DHCP_OPTION_REQ_IP:
                    memcpy(DHCP_LEASED_IP, opt_ptr + 2, 4);
                    break;
            }
            opt_ptr += opt_ptr[1] + 2;
        }
        if (msg_type == DHCP_OFFER) {
            DHCP_state = DHCP_REQUEST;
            send_DHCP_REQUEST();
            return DHCP_RET_UPDATE;
        } else if (msg_type == DHCP_ACK) {
            return DHCP_RET_UPDATE;
        } else if (msg_type == DHCP_NAK) {
            DHCP_state = DHCP_DISCOVER;
            return DHCP_RET_ERR;
        }
    } else {
        if (DHCP_state == DHCP_DISCOVER) {
            send_DHCP_DISCOVER();
        } else if (DHCP_state == DHCP_REQUEST) {
            send_DHCP_REQUEST();
        }
    }
    return DHCP_RET_NONE;
}

void DHCP_stop(void) {
    close(DHCP_SOCKET);
}

void getIPfromDHCP(uint8_t *ip) {
    memcpy(ip, DHCP_LEASED_IP, 4);
}

void getGWfromDHCP(uint8_t *gw) {
    memcpy(gw, DHCP_GATEWAY, 4);
}

void getSNfromDHCP(uint8_t *sn) {
    memcpy(sn, DHCP_SUBNET_MASK, 4);
}
