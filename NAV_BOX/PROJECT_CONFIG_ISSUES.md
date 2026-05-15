# NAV_BOX Project Configuration Issues Report

## CRITICAL ISSUES

### 1. **Missing FATFS Instance Declaration in `save_data_to_SD_SPI()`**
- **File**: `Core/Src/main.c`
- **Problem**: The function uses a global `fs_sd` variable but it's not initialized in the variable declarations
- **Impact**: SD card initialization fails when `save_data_to_SD_SPI()` is called
- **Location**: Line 299-303 (save_data_to_SD_SPI function) uses `fs_sd` but it's declared in test_sd_card_write
- **Fix**: Ensure `static FATFS fs_sd;` is properly declared globally and NOT redeclared locally

---

### 2. **Uninitialized NSLP Data Structures**
- **File**: `Core/Inc/nslp_packets.h` 
- **Problem**: `BNO055_nslp_data_a` and `BNO055_nslp_data_g` are declared but NOT initialized
```c
struct BNO055_payload_a BNO055_nslp_data_a;  // ❌ Uninitialized
struct BNO055_payload_g BNO055_nslp_data_g;  // ❌ Uninitialized
```
- **Impact**: Random/garbage values transmitted via NSLP on first call
- **Fix**: Initialize with zero values:
```c
struct BNO055_payload_a BNO055_nslp_data_a = {0};
struct BNO055_payload_g BNO055_nslp_data_g = {0};
```

---

### 3. **SPI Speed Configuration Issue**
- **File**: `Core/Src/main.c` (line 883) and `FATFS/Target/user_diskio_spi.c` (lines 29-30)
- **Problem**: SPI prescaler is set to `SPI_BAUDRATEPRESCALER_4` (~4.5 Mbps) in initialization, but SD card initialization needs slower speeds first
```c
#define FCLK_SLOW() { MODIFY_REG(..., SPI_BAUDRATEPRESCALER_128); }  /* ~280 KBits/s */
#define FCLK_FAST() { MODIFY_REG(..., SPI_BAUDRATEPRESCALER_8); }    /* ~4.5 MBits/s */
```
- **Impact**: SD card may not initialize properly because it starts too fast
- **Fix**: The code SHOULD be working since `USER_SPI_initialize()` calls `FCLK_SLOW()` first, but verify this is being called

---

### 4. **No Delay Between SD Mount and File Operations**
- **File**: `FATFS/App/fatfs.c` and `Core/Src/main.c`
- **Problem**: FATFS initialization happens immediately without SD card stabilization time
- **Current Code**:
```c
fres = f_mount(&fs_sd, "", 0);  // Mounts immediately
if (fres != FR_OK) return 1;
fres = f_open(&gps_csv_file, ...);  // Opens file immediately
```
- **Impact**: Mount can fail on first attempt or file operations can timeout
- **Fix**: Already added `HAL_Delay(100)` in the corrected test function - apply same to `save_data_to_SD_SPI()`

---

### 5. **USART2 Baud Rate Mismatch**
- **File**: `Core/Src/main.c` (line 948)
- **Problem**: USART2 configured to 9600 baud, but NSLP transmission expects potentially different rate
- **Impact**: NSLP packet transmission may have timing issues
- **Fix**: Verify NSLP expected baud rate matches 9600, or recalculate `nslp_min_delay()` with actual baud

---

## MAJOR ISSUES

### 6. **Main Loop Logic Flow Error**
- **File**: `Core/Src/main.c` (lines 751-763)
- **Problem**: ALL sensor functions are commented out, only `test_sd_card_write()` runs in an infinite loop
```c
while (1) {
    // parse_gps_data();      // ❌ Commented
    // parse_bno055_data();   // ❌ Commented
    // poll_n_parse_bmp180_data(); // ❌ Commented
    // process_sensor_fusion();    // ❌ Commented
    // sd_status = save_data_to_SD_SPI(); // ❌ Commented
    // nslp_transmit_packets(); // ❌ Commented
    
    sd_status = test_sd_card_write();  // ✓ Only this runs
}
```
- **Impact**: No actual sensor data collection/processing; SD test called repeatedly
- **Fix**: Uncomment sensor functions and move test outside main loop

---

### 7. **Data Type Mismatch in NSLP Packets**
- **File**: `Core/Inc/nslp_packets.h` (lines 30-50)
- **Problem**: BNO055 payload uses `double` for floats, but BMP180 uses `float`
```c
struct BNO055_payload_a {
    double ax;   // 8 bytes
    double ay;   // 8 bytes
    double az;   // 8 bytes
    uint32_t time;
};

struct BMP_payload {
    float temp;   // 4 bytes  ❌ Inconsistent
    uint32_t press;
    uint32_t time;
};
```
- **Impact**: Packet size inconsistency; receiving side expects different data layout
- **Severity**: Medium (will work but receiver needs to know about this)
- **Fix**: Use consistent float type (preferably `float` for 4-byte alignment):
```c
struct BNO055_payload_a {
    float ax;
    float ay;
    float az;
    uint32_t time;
};
```

---

### 8. **Missing Synchronization in Multi-Peripheral System**
- **File**: `Core/Src/main.c` 
- **Problem**: No synchronization mechanism between:
  - GPS DMA completion (USART1)
  - BNO055 DMA completion (I2C1)
  - BMP180 polling (I2C1)
  - NSLP transmission (USART2)
- **Impact**: Race conditions possible; data might be corrupted mid-transmission
- **Fix**: Add proper mutex/semaphore if using RTOS, or atomic flags if not

---

### 9. **GPS Fix Validation Logic Error**
- **File**: `Core/Src/main.c` (line 244)
- **Problem**: Logic checks if fixType == 0 (invalid) then returns, but function returns `int` not explicitly assigned
```c
uint8_t parsecheck = UBX_ParseNavSolFrame(...);
if(GPS_nslp_data.fixType == 0){
    return 0;  // ❌ Returns 0, but no error value set
}
```
- **Impact**: Function doesn't indicate whether parsing succeeded
- **Fix**: Return proper status codes

---

### 10. **CSV Header Written Every File Open**
- **File**: `Core/Src/main.c` (lines 334-337 in save_data_to_SD_SPI)
- **Problem**: Checks `f_size()` immediately after opening, but if file exists with data, size > 0 check is correct. However, no newline handling if appending to existing file
- **Impact**: CSV format might be corrupted if appending to existing file
- **Fix**: Better header management

---

## MEDIUM ISSUES

### 11. **Hardcoded SPI Handle Reference**
- **File**: `FATFS/Target/user_diskio_spi.c` (line 25-26)
- **Problem**: Macro `SD_SPI_HANDLE` is used but defined in `main.h`
- **Potential Issue**: If SPI handle name changes, code breaks silently
- **Fix**: Add compile-time verification or document dependency

---

### 12. **No Error Recovery in GPS Parser**
- **File**: `Core/Src/main.c` (parse_gps_data function)
- **Problem**: If `UBX_ParseNavSolFrame()` fails, no error handling or retry logic
- **Impact**: Corrupted GPS data could be used for fusion
- **Fix**: Check parsecheck return value and only use data if valid

---

### 13. **Missing Return Value in parse_gps_data()**
- **File**: `Core/Src/main.c` (line 244)
- **Problem**: Function has no return type (void), but line 244 has `return 0;`
- **Compiler Issue**: Will generate warning/error
- **Fix**: Remove return statement or change function to return uint8_t

---

### 14. **BMP180 Timing Not Synchronized**
- **File**: `Core/Src/main.c` (lines 223-234)
- **Problem**: BMP180 polls on fixed interval but doesn't coordinate with other sensor timings
- **Impact**: Different sensor data collected at different times; fusion less accurate
- **Fix**: Synchronize all sensor polling intervals or use event-driven architecture

---

## CONFIGURATION ISSUES

### 15. **FATFS Code Page Configuration**
- **File**: `FATFS/Target/ffconf.h` (line 81)
- **Setting**: `#define _CODE_PAGE 850` (OEM - Multilingual Latin I)
- **Note**: Acceptable but ensure this matches your file system expectations

---

### 16. **Missing FATFS Time Support**
- **File**: `FATFS/App/fatfs.c` (line 48)
- **Problem**: `get_fattime()` returns 0 - no RTC configured
```c
DWORD get_fattime(void) {
    return 0;  // ❌ Files will have timestamp 1980-01-01 00:00:00
}
```
- **Impact**: CSV files won't have meaningful timestamps
- **Fix**: Implement RTC or use system ticks:
```c
DWORD get_fattime(void) {
    uint32_t ticks = HAL_GetTick();
    uint32_t days = ticks / 86400000;
    // Convert to FAT timestamp
    // ...
}
```

---

### 17. **String Function Configuration**
- **File**: `FATFS/Target/ffconf.h` (line 60)
- **Setting**: `#define _USE_STRFUNC 2` (Enable with LF-CRLF conversion)
- **Note**: Good for text files (CSV), but verify receiver expects CRLF

---

## SUMMARY TABLE

| Priority | Issue | Location | Status |
|----------|-------|----------|--------|
| 🔴 CRITICAL | Uninitialized NSLP structures | nslp_packets.h | Unfixed |
| 🔴 CRITICAL | No SD delay after mount | main.c:299+ | Partially Fixed |
| 🔴 CRITICAL | Main loop test-only | main.c:751 | Unfixed |
| 🟠 MAJOR | Data type inconsistency | nslp_packets.h | Unfixed |
| 🟠 MAJOR | Missing GPS validation check | main.c:244 | Unfixed |
| 🟠 MAJOR | Return statement in void function | main.c:244 | Unfixed |
| 🟡 MEDIUM | No RTC for FAT timestamps | fatfs.c:48 | Unfixed |
| 🟡 MEDIUM | BMP180 timing sync | main.c:223+ | Unfixed |

---

## QUICK FIXES PRIORITY

1. **Uncomment main loop functions** (Line 751-763)
2. **Initialize NSLP data structures** (nslp_packets.h lines 44, 55)
3. **Fix return statement** (main.c:244)
4. **Standardize float types** (nslp_packets.h)
5. **Remove test function from main loop** (main.c:751)
6. **Add RTC time support** (fatfs.c:48)

