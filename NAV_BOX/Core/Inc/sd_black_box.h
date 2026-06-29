/* sd_black_box.h
 * Header for SD black box logging/storage
 */
#ifndef SD_BLACK_BOX_H
#define SD_BLACK_BOX_H

#include "fatfs.h"
#include <stdarg.h> //for va_list var arg functions
#include "main.h"

// FIX: buffer is uint8_t* (byte pointer) not uint32_t* — matches how char
// buffers are passed from sd_store() and how f_write() treats the data.
uint8_t sd_card_write(const uint8_t *buffer, uint32_t length);
void sd_card_init(const char *filename);

//SD CARD TEST
void myprintf(UART_HandleTypeDef *huart, const char *fmt, ...);
void test_sd_card_write(UART_HandleTypeDef *huart);

#endif /* SD_BLACK_BOX_H */
