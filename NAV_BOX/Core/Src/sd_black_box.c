#include "sd_black_box.h"

#include <stdio.h>
#include <string.h>

//-------------------------------SD_CARD_SPI---------------------------------------------
static FIL log_file;      // FatFs log_file object for the active CSV/log log_file on the SD card
static DIR log_dir;       // FatFs directory object for the SD card root directory
static FATFS fs_sd;       // FatFs volume object representing the mounted SD card filesystem
static uint8_t sd_initialized = 0;
static uint8_t log_file_open = 0;
static uint32_t write_count = 0;

static FRESULT debug;
#define SD_SYNC_INTERVAL 16   // call f_sync every N successful writes

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
  FATFS FatFs;  //Fatfs handle
  FIL fil;      //log_file handle
  FRESULT fres; //Result after operations

  //Open the log_file system
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

  //Now let's try and write a log_file "write.txt"
  fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(fres == FR_OK) {
    myprintf(huart, "I was able to open 'write.txt' for writing\r\n");
  } else {
    myprintf(huart, "f_open error (%i)\r\n", fres);
  }

  //Copy in a string
  BYTE readBuf[30];

  strncpy((char*)readBuf, "a new log_file is made!", 19);
  UINT bytesWrote;
  fres = f_write(&fil, readBuf, 19, &bytesWrote);
  if(fres == FR_OK) {
    myprintf(huart, "Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
  } else {
    myprintf(huart, "f_write error (%i)\r\n", fres);
  }

  //Be a tidy kiwi - don't forget to close your log_file!
  f_close(&fil);

  // FIX 1: Do NOT unmount here. sd_card_init() will remount using fs_sd.
  // Unmounting here with a local FATFS object that is about to go out of scope
  // corrupts FatFs's internal state for the subsequent sd_card_init() mount.
  // f_mount(NULL, "", 0);  <-- removed
}

uint8_t sd_card_init(const char *filename) {
  /*
    Start Log log_file
    filenmame format: NAME_000
  */
  //check if filename is given
  if (filename == NULL) {
    filename = "default_log.csv";
  }

  //maunt SD cardfilesystem
  if (f_mount(&fs_sd, "", 1) != FR_OK) {
    sd_initialized = 0;
    log_file_open = 0;
    return 2;
  }

  if (log_file_open) {
    f_close(&log_file);
    log_file_open = 0;
  }

  //f_unlink(filename);

  if (f_open(&log_file, filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
    sd_initialized = 0;
    return 3;
  }

  log_file_open = 1;
  write_count = 0;

  sd_initialized = 1;
  return 0;
}

/* OPTIMIZED IMPLEMENTATION - PERSISTENT log_file HANDLE, NON-BLOCKING */
uint8_t sd_card_write(const uint8_t *buffer, uint32_t length) {
  if (sd_initialized == 0 || log_file_open == 0) {
    return 1;
  }
  if(buffer == NULL){
    return 2;
  }
  if(length == 0){
    return 3;
  }

  FRESULT fres;
  UINT bytes_written = 0;

  fres = f_write(&log_file, buffer, length, &bytes_written);
  if (fres != FR_OK) {
    return 4;
  }
  if( bytes_written != length){
    return 5;
  }

  write_count++;

  // FIX 4: Periodically sync to flush the FatFs sector cache to the physical
  // card. Without this, f_write only fills RAM buffers and nothing is ever
  // committed to the SD card unless the log_file is cleanly closed first.
  if ((write_count % SD_SYNC_INTERVAL) == 0) {
    f_sync(&log_file);
  }

  return 0;
}
