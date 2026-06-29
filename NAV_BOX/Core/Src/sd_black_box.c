#include "sd_black_box.h"

#include <stdio.h>
#include <string.h>

//-------------------------------SD_CARD_SPI---------------------------------------------
static FIL csv_file;      // FatFs file object for the active CSV/log file on the SD card
static FATFS fs_sd;       // FatFs volume object representing the mounted SD card filesystem
static uint8_t sd_initialized = 0;
static uint32_t write_count = 0;

#define SD_SYNC_INTERVAL 16   // call f_sync every N successful writes

static uint8_t file_exists(const char *path) {
  return (f_stat(path, NULL) == FR_OK) ? 1U : 0U;
}

static uint8_t build_indexed_filename(const char *filename, uint32_t index, char *out, size_t out_size) {
  const char *slash = strrchr(filename, '/');
  const char *backslash = strrchr(filename, '\\');
  const char *path_sep = slash;

  if (backslash != NULL && (path_sep == NULL || backslash > path_sep)) {
    path_sep = backslash;
  }

  const char *name_start = (path_sep != NULL) ? path_sep + 1 : filename;
  const char *extension = strrchr(name_start, '.');

  if (extension != NULL) {
    size_t prefix_len = (size_t)(extension - filename);
    int written = snprintf(out, out_size, "%.*s_%04lu%s",
                           (int)prefix_len, filename,
                           (unsigned long)index,
                           extension);
    return (written > 0 && (size_t)written < out_size) ? 1U : 0U;
  }

  int written = snprintf(out, out_size, "%s_%04lu", filename, (unsigned long)index);
  return (written > 0 && (size_t)written < out_size) ? 1U : 0U;
}
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
  FIL fil;      //File handle
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

  // FIX 1: Do NOT unmount here. sd_card_init() will remount using fs_sd.
  // Unmounting here with a local FATFS object that is about to go out of scope
  // corrupts FatFs's internal state for the subsequent sd_card_init() mount.
  // f_mount(NULL, "", 0);  <-- removed
}

void sd_card_init(const char *filename) {
  /*
    Start Log file
  */
  char log_filename[64];

  if (filename == NULL) {
    sd_initialized = 0;
    return;
  }

  // FIX 2: Mount using the persistent static fs_sd object.
  // The test function no longer unmounts, so this remount cleanly
  // replaces the test's local FATFS handle with our long-lived one.
  if (f_mount(&fs_sd, "", 1) != FR_OK) {
    sd_initialized = 0;
    return;
  }

  if (!file_exists(filename)) {
    int written = snprintf(log_filename, sizeof(log_filename), "%s", filename);
    if (written <= 0 || (size_t)written >= sizeof(log_filename)) {
      sd_initialized = 0;
      return;
    }
  } else {
    uint32_t index = 1U;

    do {
      if (!build_indexed_filename(filename, index, log_filename, sizeof(log_filename))) {
        sd_initialized = 0;
        return;
      }
      index++;
    } while (file_exists(log_filename));
  }

  // FIX 3: FA_WRITE must be combined with FA_CREATE_NEW so the file is
  // opened for writing, not just created. Without FA_WRITE, f_write() fails.
  if (f_open(&csv_file, log_filename, FA_WRITE | FA_CREATE_NEW) != FR_OK) {
    sd_initialized = 0;
    return;
  }

  // Write CSV header
  const char *header = "type,ax,ay,az,pitch,roll,yaw,temp,pressure,lat,lon,alt,fix,numSV,gps_tow,timestamp\r\n";
  UINT bytes_written;
  f_write(&csv_file, header, strlen(header), &bytes_written);
  f_sync(&csv_file); // flush header immediately so it survives a reset

  sd_initialized = 1;
}

/* OPTIMIZED IMPLEMENTATION - PERSISTENT FILE HANDLE, NON-BLOCKING */
uint8_t sd_card_write(const uint8_t *buffer, uint32_t length) {
  if (!sd_initialized || buffer == NULL || length == 0) {
    return 0;
  }

  FRESULT fres;
  UINT bytes_written = 0;

  fres = f_write(&csv_file, buffer, length, &bytes_written);
  if (fres != FR_OK || bytes_written != length) {
    return 0;
  }

  write_count++;

  // FIX 4: Periodically sync to flush the FatFs sector cache to the physical
  // card. Without this, f_write only fills RAM buffers and nothing is ever
  // committed to the SD card unless the file is cleanly closed first.
  if ((write_count % SD_SYNC_INTERVAL) == 0) {
    f_sync(&csv_file);
  }

  return 1;
}
