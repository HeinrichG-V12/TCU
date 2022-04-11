/*
 * crc.h
 *
 *  Created on: Apr 10, 2022
 *      Author: Heinrich
 */

#ifndef INC_CRC_H_
#define INC_CRC_H_

#include "stdint.h"

typedef unsigned char crc_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

crc_t calc_crc(const crc_t message[], int nBytes);
uint32_t crc32(const void *buf, uint32_t size);
uint32_t crc32inc(const void *buf, uint32_t crc, uint32_t size);

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* INC_CRC_H_ */
