/* sd_black_box.h
 * Header for SD black box logging/storage
 */
#ifndef SD_BLACK_BOX_H
#define SD_BLACK_BOX_H

#include "fatfs.h"
#include <stdarg.h> //for va_list var arg functions
#include "main.h"

uint8_t sd_card_write(uint32_t *buffer, uint32_t length);
void sd_card_init(const char *filename);

//SD CARD TEST
void myprintf(UART_HandleTypeDef *huart, const char *fmt, ...);
void test_sd_card_write(UART_HandleTypeDef *huart);

#endif /* SD_BLACK_BOX_H */
