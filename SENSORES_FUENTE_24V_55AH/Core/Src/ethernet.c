/*
 * ethernet.c
 *
 *  Created on: Jan 8, 2024
 *      Author: A.R.T.
 */
#include "ethernet.h"


// This function transmits data via SPI. It's used to modify, access, or write to a specific register.
void transmitir_spi(uint8_t *p, uint8_t len) {
	HAL_StatusTypeDef res = HAL_ERROR;
	HAL_GPIO_WritePin( w5500_hw.nssPort, w5500_hw.nssPin, GPIO_PIN_RESET); // pull the pin low
	res = HAL_SPI_Transmit(w5500_hw.spi, p, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(w5500_hw.nssPort, w5500_hw.nssPin, GPIO_PIN_SET); // pull the pin high
	if (res != HAL_OK)
		Error_Handler();
	HAL_Delay(10);
}

// This function transmits a command and receives data from the w5500 chip via SPI.
void transmitir_recibir_spi(uint8_t *p_t, uint8_t len_t, uint8_t *p_r,
		uint16_t len_r) {
	HAL_StatusTypeDef res;
	HAL_GPIO_WritePin( w5500_hw.nssPort, w5500_hw.nssPin, GPIO_PIN_RESET); // pull the pin low
	res = HAL_SPI_Transmit(w5500_hw.spi, p_t, len_t, HAL_MAX_DELAY);
	res = HAL_SPI_Receive(w5500_hw.spi, p_r, len_r, HAL_MAX_DELAY);
	HAL_GPIO_WritePin( w5500_hw.nssPort, w5500_hw.nssPin, GPIO_PIN_SET); // pull the pin high

	if (res != HAL_OK)
		Error_Handler();
	HAL_Delay(10);
}

// Configures the common registers responsible for IP and MAC addresses.
void common_register_block(uint8_t *buff, uint16_t address, uint8_t *data,
		uint8_t len) {
	uint8_t bsb = 0x00;
	uint8_t rwb = 0x01 << 2; // write
	uint8_t om = 00; // VDM
	uint8_t c_phase = bsb | rwb | om;
	uint16_t os_address = address;
	os_address = os_address << 8;
	p = buff;
	memcpy(p, &os_address, 2);
	p += 2;
	memcpy(p, &c_phase, 1);
	p += 1;
	for (int i = 0; i < len; i++) {
		memcpy(p, &data[i], 1);
		p += 1;
	}
	transmitir_spi(buff, (3 + len));
	//free(buff);
}

// Writes to the socket registers to enable channel communication. BSB[4:0] selects the socket to use.
void eth_write_reg(uint8_t bsb, uint16_t address, uint8_t *data, uint16_t len) {

	uint8_t *buff;
	buff = malloc(sizeof(uint8_t) * len + 3);
	if (buff == NULL)
		Error_Handler();

	uint16_t os_address;
	uint8_t rwb;
	uint8_t om;
	uint8_t c_phase;
	uint8_t t = 3 + len;
	bsb = bsb << 3;
	rwb = 0x01 << 2; // write
	om = 00; // VDM
	c_phase = bsb | rwb | om;
	os_address = (address << 8) + ((address >> 8) & 0x00FF);
	p = buff;
	memcpy(p, &os_address, 2);
	p += 2;
	memcpy(p, &c_phase, 1);
	p += 1;
	for (int i = 0; i < len; i++) {
		memcpy(p, &data[i], 1);
		p += 1;
	}
	transmitir_spi(buff, t);
	free(buff);
}

// Writes to the socket registers to enable channel communication. BSB[4:0] selects the socket to use.
void socket_write_register(uint8_t *buff, uint16_t address, uint8_t bsb,uint8_t *data, uint16_t len) {
	uint16_t os_address;
	uint8_t rwb;
	uint8_t om;
	uint8_t c_phase;
	uint8_t t = 3 + len;
	bsb = bsb << 3;
	rwb = 0x01 << 2; // write
	om = 00; // VDM
	c_phase = bsb | rwb | om;
	//os_address_1 = address << 8;
	os_address = (address << 8) + ((address >> 8) & 0x00FF);
	p = buff;
	memcpy(p, &os_address, 2);
	p += 2;
	memcpy(p, &c_phase, 1);
	p += 1;
	for (int i = 0; i < len; i++) {
		memcpy(p, &data[i], 1);
		p += 1;
	}
	transmitir_spi(buff, t);
}


// Reads from the specified register via SPI.
void eth_read_reg(uint8_t BSB_SELECT, uint16_t offset, uint8_t *buffer_r,
		uint16_t buffer_r_len) {
	uint16_t offset_address=0;
	uint8_t buffer_t[3];
	offset_address = offset << 8;
	uint8_t BSB = BSB_SELECT << 3; // block select bit: 0x01 SOCKET REGISTER, 0x02 SOCKET TX BUFFER, 0x03 SOCKET RX BUFFER
	uint8_t RWB = 0x00 << 2; // read
	uint8_t OM = 00; // VDM
	uint8_t control_phase = BSB | RWB | OM;
	buffer_t[0] = (offset >> 8) & 0xFF;
	buffer_t[1] = (offset & 0xFF);
	buffer_t[2] = control_phase;
	transmitir_recibir_spi(buffer_t, 3, buffer_r, buffer_r_len);
}


// Configures the common registers.
void common_reg_config(uint8_t buffer[243], uint8_t mode, uint8_t gar[],
		uint8_t sub_r[], uint8_t shar[], uint8_t sipr[]) {
	//---------------------- configuration common register
	common_register_block(buffer, 0x00, (uint8_t*) &mode, sizeof(mode));
	common_register_block(buffer, 0x01, gar, sizeof(gar));
	common_register_block(buffer, 0x05, sub_r, sizeof(sub_r));
	common_register_block(buffer, 0x09, shar, sizeof(shar));
	common_register_block(buffer, 0x0F, sipr, sizeof(sipr));
}

// Configures the socket registers.
void socket_reg_config(uint8_t buffer[243], uint8_t S_MR, uint8_t S_PORT[2],
		uint8_t S_DHAR[6], uint8_t S_DPORT[2], uint8_t S_MMS[2], uint8_t S_TTL,
		uint8_t S_RXBUF_SIZE, uint8_t S_TXBUF_SIZE, uint8_t S_CR_open,
		uint8_t S_CR_listen) {
	//---------------------- configuration socket register
	socket_write_register(buffer, 0x00, 0x01, (uint8_t*) &S_MR, sizeof(S_MR));
	socket_write_register(buffer, 0x04, 0x01, (uint8_t*) S_PORT,
			sizeof(S_PORT));
	socket_write_register(buffer, 0x06, 0x01, (uint8_t*) &S_DHAR,
			sizeof(S_DHAR));
	socket_write_register(buffer, 0x10, 0x01, (uint8_t*) S_DPORT,
			sizeof(S_DPORT));
	socket_write_register(buffer, 0x12, 0x01, (uint8_t*) &S_MMS, sizeof(S_MMS));
	socket_write_register(buffer, 0x16, 0x01, (uint8_t*) &S_TTL, sizeof(S_TTL));
	socket_write_register(buffer, 0x1E, 0x01, (uint8_t*) &S_RXBUF_SIZE,
			sizeof(S_RXBUF_SIZE));
	socket_write_register(buffer, 0x1F, 0x01, (uint8_t*) &S_TXBUF_SIZE,
			sizeof(S_TXBUF_SIZE));
	socket_write_register(buffer, 0x01, 0x01, (uint8_t*) &S_CR_open,
			sizeof(S_CR_open));
	socket_write_register(buffer, 0x01, 0x01, (uint8_t*) &S_CR_listen,
			sizeof(S_CR_listen));
}


// Handles the transmission process through the Ethernet interface.
void eth_transmit(uint8_t sn_reg, uint8_t *data, uint16_t data_len) {
	/*
	 * 1. Read the starting address for saving the transmitting data.
	 2. Save the transmitting data from the starting address of Socket n TX
	 buffer.
	 3. After saving the transmitting data, update Sn_TX_WR to the
	 increased value as many as transmitting data size. If the increment
	 value exceeds the maximum value 0xFFFF(greater than 0x10000 and the
	 carry bit occurs), then the carry bit is ignored and will automatically
	 update with the lower 16bits value.
	 4. Transmit the saved data in Socket n TX Buffer by using SEND/SEND
	 command
	 */

	uint8_t s_TX_RD[2];
	uint16_t ptr = 0;

	eth_read_reg(sn_reg, S_TX_RD_OFFSET, s_TX_RD, sizeof(s_TX_RD));

	ptr = (s_TX_RD[0] << 8) + s_TX_RD[1];
	eth_write_reg(sn_reg + S_N_TX_OFFSET, ptr, data, data_len);

	ptr = (ptr + data_len);
	s_TX_RD[0] = (ptr >> 8) & 0x00FF;
	s_TX_RD[1] = (ptr) & 0x00FF;
	eth_write_reg(sn_reg, S_TX_WR_OFFSET, s_TX_RD, sizeof(s_TX_RD));

	uint8_t cmd[1];
	cmd[0] = S_CR_SEND;
	eth_write_reg(sn_reg, S_CR_OFFSET, cmd, sizeof(cmd));

}

// Configures the command register of socket
void socket_cmd_cfg(uint8_t sn_reg, uint8_t cmd) {
	// SOCK_ESTABLISHED
	eth_write_reg(sn_reg, S_CR_OFFSET, (uint8_t*) &cmd, sizeof(cmd));

}

// Gives the length of data buffer reception
uint16_t read_socket_n_rx_buffer_len(uint8_t sn_reg) {
	uint8_t s_RX_RS[2];
	eth_read_reg(sn_reg, S_RX_RS_OFFSET, s_RX_RS, sizeof(s_RX_RS));
	return ((s_RX_RS[1]) & 0xFFFF) | ((s_RX_RS[0] << 8) & 0xFFFF);
}

// To handle the start and end of the receive buffer
uint16_t read_socket_n_rx_buffer_read_addr(uint8_t sn_reg) {
	uint8_t s_RX_RD[2];
	eth_read_reg(sn_reg, S_RX_RD_OFFSET, s_RX_RD, sizeof(s_RX_RD));
	return ((s_RX_RD[1]) & 0xFFFF) | ((s_RX_RD[0] << 8) & 0xFFFF);
}

// Update the address of buffer
void update_socket_n_rx_buffer_addr(uint8_t sn_reg, uint16_t offset_address) {
	uint8_t s_RX_RD[2];
	s_RX_RD[0] = (offset_address >> 8) & 0x00FF;
	s_RX_RD[1] = (offset_address) & 0x00FF;
	eth_write_reg(sn_reg, S_RX_RD_OFFSET, &(s_RX_RD[0]), 1);
	eth_write_reg(sn_reg, S_RX_RD_OFFSET+1, &(s_RX_RD[1]), 1);
}

// Extract the data of buffer reception
uint8_t read_socket_n_rx_buffer(uint8_t sn_reg, uint8_t *data_rcv) {
	uint16_t len_rx;
	uint16_t s_RX_RD_addr;
	uint16_t s_RX_RD_addr_updated;
	len_rx = read_socket_n_rx_buffer_len(sn_reg);

	if (len_rx <= 0)
		return (0);
	s_RX_RD_addr = read_socket_n_rx_buffer_read_addr(sn_reg);
	eth_read_reg(sn_reg + S_N_RX_OFFSET, s_RX_RD_addr, data_rcv, len_rx);

	s_RX_RD_addr_updated = s_RX_RD_addr + len_rx;
	update_socket_n_rx_buffer_addr(sn_reg, s_RX_RD_addr_updated);

	return len_rx;
}


// initialization chip W5500
void init_w5500_hw(W5500_HW_t *w5500_hw, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *nssPort, uint16_t nssPin, GPIO_TypeDef *nrstPort,
		uint16_t nrstPin) {
	// Ensure that the pointers are not NULL
	assert_param(w5500_hw != NULL);
	assert_param(hspi != NULL);
	assert_param(nssPort != NULL);
	assert_param(nrstPort != NULL);


	w5500_hw->nssPin = nssPin;
	w5500_hw->nssPort = nssPort;
	w5500_hw->nrstPin = nrstPin;
	w5500_hw->nrstPort = nrstPort;
	w5500_hw->spi = hspi;
	HAL_GPIO_WritePin(nssPort, nssPin, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(nssPort, nssPin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(nrstPort, nrstPin, GPIO_PIN_SET);


}


// Send packet with specific TCP/IP protocol
void send_modbus_tcp_ip(uint8_t socket, uint16_t transaction_id, uint8_t unit_id,
                        uint16_t start_address, uint32_t curr_bat, uint32_t curr_up, uint32_t curr_down,
                        uint32_t volt_bat, uint8_t ac_active) {

    uint8_t tx_buffer[29]; // 7 bytes MBAP + 22 bytes PDU

    // Built MBAP Header
    tx_buffer[0] = (transaction_id >> 8) & 0xFF; // Transaction ID high byte
    tx_buffer[1] = transaction_id & 0xFF;        // Transaction ID low byte
    tx_buffer[2] = 0x00;                         // Protocol ID high byte
    tx_buffer[3] = 0x00;                         // Protocol ID low byte
    tx_buffer[4] = 0x00;                         // Length high byte
    tx_buffer[5] = 0x15;                         // Length low byte (PDU length + Unit ID)
    tx_buffer[6] = unit_id;                      // Unit ID

    // Built PDU
    tx_buffer[7] = 0x10;                         // Function Code (Write Multiple Registers)
    tx_buffer[8] = (start_address >> 8) & 0xFF;  // Start Address high byte
    tx_buffer[9] = start_address & 0xFF;         // Start Address low byte
    tx_buffer[10] = 0x00;                        // Quantity of Registers high byte
    tx_buffer[11] = 0x06;                        // Quantity of Registers low byte
    tx_buffer[12] = 0x0C;                        // Byte Count (6 registers * 2 bytes)

    // convert integer values ​​to floats
    float curr_bat_f = (float)curr_bat * 3.3 / 4096;
    float curr_up_f = (float)curr_up * 3.3 / 4096;
    float curr_down_f = (float)curr_down * 3.3 / 4096;
    float volt_bat_f = (float)volt_bat * 3.3 / 4096;

    // Convert the values from float to bytes
    uint8_t *curr_bat_bytes = (uint8_t*)&curr_bat_f;
    uint8_t *curr_up_bytes = (uint8_t*)&curr_up_f;
    uint8_t *curr_down_bytes = (uint8_t*)&curr_down_f;
    uint8_t *volt_bat_bytes = (uint8_t*)&volt_bat_f;

    // Add the values to PDU
    tx_buffer[13] = curr_bat_bytes[1];
    tx_buffer[14] = curr_bat_bytes[0];
    tx_buffer[15] = curr_bat_bytes[3];
    tx_buffer[16] = curr_bat_bytes[2];
    tx_buffer[17] = curr_up_bytes[1];
    tx_buffer[18] = curr_up_bytes[0];
    tx_buffer[19] = curr_up_bytes[3];
    tx_buffer[20] = curr_up_bytes[2];
    tx_buffer[21] = curr_down_bytes[1];
    tx_buffer[22] = curr_down_bytes[0];
    tx_buffer[23] = curr_down_bytes[3];
    tx_buffer[24] = curr_down_bytes[2];
    tx_buffer[25] = volt_bat_bytes[1];
    tx_buffer[26] = volt_bat_bytes[0];
    tx_buffer[27] = volt_bat_bytes[3];
    tx_buffer[28] = volt_bat_bytes[2];

    // Transmission of packet
    eth_transmit(socket, tx_buffer, sizeof(tx_buffer));
}


















