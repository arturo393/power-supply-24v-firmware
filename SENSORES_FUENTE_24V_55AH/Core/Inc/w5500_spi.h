/*
 * w5500_spi.h
 *
 *  Created on: Dec 4, 2023
 *      Author: claudio
 */
#include "stm32g0xx_hal.h"
#include "wizchip_conf.h"
#include "stdio.h"
#include "w5500.h"

#ifndef INC_W5500_SPI_H_
#define INC_W5500_SPI_H_



#endif /* INC_W5500_SPI_H_ */

uint8_t SPIReadWrite(uint8_t);
void wizchip_select(void);
void wizchip_deselect(void);
uint8_t wizchip_read();
void wizchip_write(uint8_t);
void wizchip_readburst(uint8_t*, uint16_t);
void wizchip_writeburst(uint8_t*, uint16_t);
void W5500IOInit();
void W5500Init();
