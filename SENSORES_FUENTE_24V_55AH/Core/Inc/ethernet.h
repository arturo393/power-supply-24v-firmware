/*
 * ethernet.h
 *
 *  Created on: June 14, 2024
 *      Author: A.R.T
 */

#ifndef INC_ETHERNET_H_
#define INC_ETHERNET_H_
#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "stm32g0xx_hal_spi.h"

extern SPI_HandleTypeDef hspi1;
extern uint8_t *p;


//To handle the interrupt registers//
#define Sn_IR_SEND_OK  (0xff & (1<<4))
#define Sn_IR_TIME_OUT (0xff & (1<<3))
#define Sn_RECEIVE     (0xff & (1<<2))
#define Sn_DISCONNECT  (0xff & (1<<1))
#define Sn_CONNECT     (0xff & (1<<0))
#define Sn_IR_MASK     (0xff & ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4)))

#define S_N_TX_OFFSET 0x01
#define S_N_RX_OFFSET 0x02
//common block registers//
#define COMMON_REG_OFFSET 0X00
#define PHYCFGR_RST_OFFSET 0x2E
//Selection of the socket registers//
#define S_CR_OFFSET 0x01
#define S_IR_OFFSET 0x02
#define S_TX_RD_OFFSET 0x22
#define S_TX_WR_OFFSET 0x24
#define S_RX_RS_OFFSET 0x26
#define S_RX_RD_OFFSET 0x28
#define S_RX_WR0_OFFSET 0x2A
#define S_RX_WR1_OFFSET 0x2B
#define S_SR_OFFSET 0x03
#define S_PORT_OFFSET 0x04
#define S_DHAR_OFFSET 0x06
#define S_DIPR_OFFSET 0x0C
#define S_DPORT_OFFSET 0x10
#define S_MSS_OFFSET 0x12
#define S_TOS_OFFSET 0x15
#define S_TTL_OFFSET 0x16
#define S_RXBUF_SIZE_OFFSET 0x1E
#define S_TXBUF_SIZE_OFFSET 0x1F
#define S_TX_FS_OFFSET 0x20
#define S_IMR_OFFSET 0x2C
#define S_FRAG_OFFSET 0x2D
#define S_KPALVTR_OFFSET 0x2F

// PARA SELECCIÓN DE BLOQUES DE REGISTROS DE SOCKET//
#define socket_0_register 0x01
#define socket_1_register 0x05
#define socket_2_register 0x09
#define socket_3_register 0x0D
#define socket_4_register 0x11
#define socket_5_register 0x15
#define socket_6_register 0x19
#define socket_7_register 0x1D


//PARA SELECCIÓN DE BUFFER DE TX DE SOCKET//
#define socket_0_tx_buffer 0x02
#define socket_1_tx_buffer 0x06
#define socket_2_tx_buffer 0x0A
#define socket_3_tx_buffer 0x0E
#define socket_4_tx_buffer 0x12
#define socket_5_tx_buffer 0x16
#define socket_6_tx_buffer 0x1A
#define socket_7_tx_buffer 0x1E

//PARA SELECCIÓN DE BUFFER DE RX DE SOCKET//
#define socket_0_rx_buffer 0x03
#define socket_1_rx_buffer 0x07
#define socket_2_rx_buffer 0x0B
#define socket_3_rx_buffer 0x0F
#define socket_4_rx_buffer 0x13
#define socket_5_rx_buffer 0x17
#define socket_6_rx_buffer 0x1B
#define socket_7_rx_buffer 0x1F

//PARA ENVÍO EN SOCKET RESPECTIVO//
#define S_CR_OPEN 0x01
#define S_CR_LISTEN 0x02
#define S_CR_CONNECT 0x04
#define S_CR_DISCONECT 0x08
#define S_CR_CLOSE 0x10
#define S_CR_SEND 0x20
#define S_CR_SEND_MAC 0x21
#define S_CR_SEND_KEEP 0x22
#define S_CR_RECV 0x40


typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *nssPort;
    uint16_t nssPin;
    GPIO_TypeDef *nrstPort;
    uint16_t nrstPin;
} W5500_HW_t;

//W5500 SPI
void init_w5500_hw(W5500_HW_t *w5500_hw, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *nssPort, uint16_t nssPin, GPIO_TypeDef *nrstPort,
		uint16_t nrstPin);

extern W5500_HW_t w5500_hw;

// For SPI:
void transmitir_spi(uint8_t* p, uint8_t len);
void transmitir_recibir_spi(uint8_t* p_t, uint8_t len_t, uint8_t* p_r, uint16_t len_r);
void eth_read_reg(uint8_t BSB_SELECT,uint16_t addr, uint8_t *buffer_r, uint16_t buffer_r_len);

// Functions to adjust the chip W5500 registers  :
void common_register_block(uint8_t* buff, uint16_t address, uint8_t* data,uint8_t len);
void socket_write_register(uint8_t* buff, uint16_t address,uint8_t bsb, uint8_t* data,uint16_t len);
void eth_write_reg(uint8_t bsb, uint16_t address, uint8_t *data, uint16_t len);
void common_reg_config(uint8_t buffer[243], uint8_t mode, uint8_t gar[],uint8_t sub_r[], uint8_t shar[], uint8_t sipr[]);
void socket_reg_config(uint8_t buffer[243], uint8_t S_MR, uint8_t S_PORT[2],
		uint8_t S_DHAR[6], uint8_t S_DPORT[2], uint8_t S_MMS[2], uint8_t S_TTL,
		uint8_t S_RXBUF_SIZE, uint8_t S_TXBUF_SIZE, uint8_t S_CR_open,
		uint8_t S_CR_listen);

// For Transmission
void eth_transmit(uint8_t socket_n_register, uint8_t *data_transmitir, uint16_t data_len);
void socket_cmd_cfg(uint8_t sn_reg, uint8_t cmd);
uint8_t read_socket_n_rx_buffer(uint8_t sn_reg, uint8_t *data_reception);
uint8_t update_socket_n_rx_buffer_read_addr(uint8_t sn_reg);
uint16_t read_socket_n_rx_buffer_len(uint8_t sn_reg);
uint16_t read_socket_n_rx_buffer_read_addr(uint8_t sn_reg);
void update_socket_n_rx_buffer_addr(uint8_t sn_reg, uint16_t offset_address);
uint8_t read_socket_n_rx_buffer(uint8_t sn_reg, uint8_t *data_reception);

// Modbus TCP/IP
void send_modbus_tcp_ip(uint8_t socket, uint16_t transaction_id, uint8_t unit_id,
  uint16_t start_address, uint32_t curr_bat, uint32_t curr_up, uint32_t curr_down,
  uint32_t volt_bat, uint8_t ac_active);

#endif /* INC_ETHERNET_H_ */
