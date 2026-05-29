/* sd_black_box.h
 * Header for SD black box logging/storage
 */
#ifndef SD_BLACK_BOX_H
#define SD_BLACK_BOX_H

#include "fatfs.h"
#include <stdarg.h> //for va_list var arg functions
#include "main.h"

void myprintf(UART_HandleTypeDef *huart, const char *fmt, ...);
void test_sd_card_write(UART_HandleTypeDef *huart);
uint8_t save_data_to_SD_SPI();

#endif /* SD_BLACK_BOX_H */
