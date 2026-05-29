#include "sd_black_box.h"

//-------------------------------SD_CARD_SPI---------------------------------------------
static FIL gps_csv_file;
static FATFS fs_sd;
static uint8_t sd_initialized = 0;
static uint32_t write_count = 0;
uint8_t sd_status = 0;
//-------------------------------SD_CARD_SPI---------------------------------------------

void myprintf(UART_HandleTypeDef *huart, const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(huart, (uint8_t*)buffer, len, -1);
}

void test_sd_card_write(UART_HandleTypeDef *huart){
  myprintf(huart, "\r\n~ SD card demo by kiwih ~\r\n\r\n");

  HAL_Delay(1000); //a short delay is important to let the SD card settle

  //some variables for FatFs
  FATFS FatFs; 	//Fatfs handle
  FIL fil; 		//File handle
  FRESULT fres; //Result after operations

  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
	myprintf(huart, "f_mount error (%i)\r\n", fres);
	while(1);
  }

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
	myprintf(huart, "f_getfree error (%i)\r\n", fres);
	while(1);
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  myprintf(huart, "SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

  //Now let's try and write a file "write.txt"
  fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(fres == FR_OK) {
	myprintf(huart, "I was able to open 'write.txt' for writing\r\n");
  } else {
	myprintf(huart, "f_open error (%i)\r\n", fres);
  }

  //Copy in a string
  BYTE readBuf[30];

  strncpy((char*)readBuf, "a new file is made!", 19);
  UINT bytesWrote;
  fres = f_write(&fil, readBuf, 19, &bytesWrote);
  if(fres == FR_OK) {
	myprintf(huart, "Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
  } else {
	myprintf(huart, "f_write error (%i)\r\n", fres);
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

  //We're done, so de-mount the drive
  f_mount(NULL, "", 0);
}

/* OPTIMIZED IMPLEMENTATION - PERSISTENT FILE HANDLE, NON-BLOCKING */
uint8_t save_data_to_SD_SPI(){
    return 1;
}