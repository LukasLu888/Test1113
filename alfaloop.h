/*
* Copyright (C) 2016 AlfaLoop Technology Co., Ltd.
*
* Licensed under the Apache License, Version 2.0 (the "License";
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* 	  www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#ifndef __ALFALOOP_H_
#define __ALFALOOP_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
#define API_VERSION		0.9.5
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
// Standard C functions
void *memcpy(void *dest, const void *src, size_t n)
{
    char *dp = dest;
    const char *sp = src;
    while (n--)
        *dp++ = *sp++;
    return dest;
}
int memcmp(const void* s1, const void* s2,size_t n)
{
    const unsigned char *p1 = s1, *p2 = s2;
    while(n--)
        if( *p1 != *p2 )
            return *p1 - *p2;
        else
            p1++,p2++;
    return 0;
}
void *memchr(const void *s, int c, size_t n)
{
    unsigned char *p = (unsigned char*)s;
    while( n-- )
        if( *p != (unsigned char)c )
            p++;
        else
            return p;
    return 0;
}
void *memmove(void *dest, const void *src, size_t n)
{
    unsigned char tmp[n];
    memcpy(tmp,src,n);
    memcpy(dest,tmp,n);
    return dest;
}
void *memset(void *s, int c, size_t n)
{
    unsigned char* p=s;
    while(n--)
        *p++ = (unsigned char)c;
    return s;
}
int strcmp(const char* s1, const char* s2)
{
    while(*s1 && (*s1==*s2))
        s1++,s2++;
    return *(const unsigned char*)s1-*(const unsigned char*)s2;
}
size_t strlen(const char *s) {
    const char *p = s;
    while (*s) ++s;
    return s - p;
}
int strncmp(const char* s1, const char* s2, size_t n)
{
    while(n--)
        if(*s1++!=*s2++)
            return *(unsigned char*)(s1 - 1) - *(unsigned char*)(s2 - 1);
    return 0;
}
/*---------------------------------------------------------------------------*/
// Error Code
#define ENONE		 	0   /* No error occur */

// System
#define EINTERNAL    	1   /* Internal Error */
#define ENOMEM       	2   /* No Memory for operation */
#define ENULLP       	3   /* Null Pointer */
#define EINVAL       	4   /* Invalid argument */
#define ENOREADY	 	  5   /* Not ready */
#define EINVALSTATE  	6   /* Invalid state, operation disallowed in this state */
#define ENOFOUND     	7   /* Not found */
#define ENOSUPPORT   	8   /* Not supported */
#define ETIME        	9   /* Timer expired (timeout) */
#define EPERM        	10  /* Operation not permitted */
#define EIO      	  	11  /* I/O error */
#define EFAULT       	12  /* Bad address */
#define EBUSY        	13  /* Device or resource busy */
#define EBADRQC      	14  /* Invalid request code */
#define EALREADY     	15  /* Operation already in progress */
#define EOVERFLOW    	16  /* Value too large for defined data type */
#define ELOAR			    17	/* Limit of available resource */
#define EDSNM			    18	/* Data size not match */


// File System
#define ENOENT       	30  /* No such file or directory */
#define EBADF        	31  /* Bad file number */
#define EROFS        	32  /* Read-only file system */
#define ENOTDIR      	33  /* Not a directory */
#define E2BIG        	34  /* Argument list too long */
#define ENOEXEC      	35  /* Exec format error */
#define EEXIST       	36  /* File exists */
#define EISDIR       	37  /* Is a directory */
#define EAPPEN       	38  /* Append faild */
#define EISCA         39  /* Insufficient storage capacity available.*/

// ELF Loader
#define EBEH       		40  /* Bad ELF header */
#define ENSYMT       	41  /* No symbol table */
#define ENSTRT       	42  /* No string table */
#define ENTXTSG       43  /* No text segment */
#define ESYMNF       	44  /* Symbol not found */
#define ESGNF       	45  /* Segment not found */
#define ENSTARTP      46  /* No starting point */

// Network
#define ENONET      	50  /* Machine is not on the network */
#define EADV        	51  /* Advertise error */
#define ECOMM       	52  /* Communication error on send */
#define EPROTO      	53  /* Protocol error */
#define EMULTIHOP   	54  /* Multihop attempted */
#define EBADMSG     	55  /* Not a data message */
#define EMSGSIZE    	56  /* Message too long */
#define ENOPROTOOPT 	57  /* Protocol not available */
#define EPROTONOSUPPORT 58  /* Protocol not supported */
#define ENETDOWN    	59  /* Network is down */
#define ENETUNREACH 	60  /* Network is unreachable */
#define ENETRESET   	61  /* Network dropped connection because of reset */
#define ECONNRESET  	62  /* Connection reset by peer */
#define ETIMEDOUT   	63  /* Connection timed out */
#define ECONNREFUSED  64  /* Connection refused */
#define EHOSTDOWN   	65  /* Host is down */
#define EHOSTUNREACH  66  /* No route to host */

/*---------------------------------------------------------------------------*/
// BleUuid
enum {
  BLE_UUID_TYPE_16 = 16,
  BLE_UUID_TYPE_128 = 128,
};

typedef struct {
  uint8_t type;
  uint16_t value;
} BleUuid16;

typedef struct {
  uint8_t type;
  uint8_t value[16];
} BleUuid128;

#define BLE_UUID16_INIT(uuid16)         \
    {                                   \
      .type = BLE_UUID_TYPE_16,        \
      .value = (uuid16),             \
    }

#define BLE_UUID128_INIT(uuid128...)    \
    {                                   \
      .type = BLE_UUID_TYPE_128,        \
      .value = { uuid128 },             \
    }

/*---------------------------------------------------------------------------*/
// BleDevice
#define BLE_DEVICE_ADDRESS_LEN           	6

typedef struct{
  uint8_t address[BLE_DEVICE_ADDRESS_LEN];
  uint8_t type;
} BleDevice;

/*---------------------------------------------------------------------------*/
// AdvDataBuilder
typedef struct {
  uint16_t  size;
  uint8_t   data[31];
} AdvData;

typedef struct {
  int (*addService16bitUUID)(AdvData *advdata, uint16_t uuid);
  int (*addServiceData)(AdvData *advdata, uint16_t uuid, uint8_t *data, uint8_t data_length);
  int (*addCompleteLocalName)(AdvData *advdata, uint8_t *name, uint8_t complete_name_length);
  int (*addShortLocalName)(AdvData *advdata, uint8_t *name, uint8_t short_name_length);
  int (*addManufacturerData)(AdvData *advdata, uint16_t manufacturerId, uint8_t *specificData, uint8_t length);
  int (*init)(AdvData *advdata);
  int (*initScanRsp)(AdvData *advdata);
} AdvDataBuilder;

AdvDataBuilder* AdvDataBuilderInit(void);
/*---------------------------------------------------------------------------*/
// BleAdapter
#define ADVERTISE_MAX_DATA_SIZE           31				// Maximum size of advertising data in octets.
#define BLE_GATT_MAX_CHARATERISTICS_NUM   5


#define BLE_GATT_SERVICE_UUID16                             0x1801
#define BLE_GATT_DESCRIPTOR_CFG_UUID16                      0x2902
#define BLE_GATT_MTU_SIZE_DEFAULT                           23
#define BLE_GATT_CHR_MAX_SIZE                              (BLE_GATT_MTU_SIZE_DEFAULT - 3)


#define BLE_GATT_SVC_TYPE_END                               0
#define BLE_GATT_SERVICE_TYPE_PRIMARY                       1
#define BLE_GATT_SERVICE_TYPE_SECONDARY                     2

#define BLE_GATT_CHR_PROPS_BROADCAST                        0x0001
#define BLE_GATT_CHR_PROPS_READ                             0x0002
#define BLE_GATT_CHR_PROPS_WRITE_NO_RSP                     0x0004
#define BLE_GATT_CHR_PROPS_WRITE                            0x0008
#define BLE_GATT_CHR_PROPS_NOTIFY                           0x0010
#define BLE_GATT_CHR_PROPS_INDICATE                         0x0020
#define BLE_GATT_CHR_PROPS_AUTH_SIGN_WRITE                  0x0040
#define BLE_GATT_CHR_PROPS_EXTENDED                         0x0080

// Characteristic permission
#define BLE_GATT_CHR_PERMISSION_READ                        0x0001
#define BLE_GATT_CHR_PERMISSION_WRITE                       0x0010

#define BLE_GATT_STATE_UNKNOWN                              0x0000
#define BLE_GATT_STATE_DISCONNECTED                         0x0001
#define BLE_GATT_STATE_CONNECTED                            0x0002

typedef int (* BleGattAccessCallback)(uint16_t conn_handle, uint16_t attr_handle,
                                      void *args);

typedef struct{
  BleUuid16                       *uuid;
  uint16_t                        *value_handle;
  uint16_t                        *cccd_handle;
  uint16_t                        props;
  uint16_t                        permission;
} BleGattCharacteristic;

typedef struct
{
  uint8_t                    type;
  BleUuid128                 *uuid;
  uint16_t                   *handle;
  uint8_t                    characteristic_count;
  BleGattCharacteristic      *characteristics;
} BleGattService;

typedef struct {
  int (*onCharacteristicWriteRequest)(uint16_t conn_handle, uint16_t handle, uint8_t *value, uint16_t length);
  int (*onCharacteristicReadRequest)(uint16_t conn_handle, uint16_t handle, uint8_t *value, uint16_t length);
  int (*onConnectionStateChanged)(uint16_t conn_handle, uint16_t state);
} BleGattServerCallback;

typedef enum {
  SCAN_POWER_SAVING = 0x00,
  SCAN_NORMAL,
  SCAN_FASTED
} ScanMode;

typedef struct{
  BleDevice    device;
  int8_t       rssi;                  			// Received Signal Strength Indication in dBm.
  uint8_t		   scan_response:1;
  uint8_t 	   type:2;
  uint8_t      len:5;
  uint8_t      data[ADVERTISE_MAX_DATA_SIZE];    // Advertising or scan response data.
} BleScanEvent;

typedef enum {
  ADV_INTERVAL_LEVEL_0 = 152,           // 152.5ms
  ADV_INTERVAL_LEVEL_1 = 318,           // 318.75ms
  ADV_INTERVAL_LEVEL_2 = 546,           // 546.25ms
  ADV_INTERVAL_LEVEL_3 = 852,           // 852.5ms
  ADV_INTERVAL_LEVEL_4 = 1022,          // 1022.5ms
  ADV_INTERVAL_LEVEL_5 = 2045,          // 2045.0ms
  ADV_INTERVAL_LEVEL_6 = 4082,          // 4082.5ms
  ADV_INTERVAL_LEVEL_7 = 5120,          // 5120.0ms
  ADV_INTERVAL_LEVEL_8 = 10040          // 10040.0ms
} AdvIntervalLevel;

typedef enum{
  ADV_EVT_SENDOUT = 0x00
} BleAdvEvent;

typedef void (* ScanEventHandler)(BleScanEvent *event);
typedef void (* AdvEventHandler)(BleAdvEvent const event);

typedef struct{
  int (*setAdvertisementData)(AdvData *advdata, AdvData *scanrsp_data);
  int (*startAdvertising)(AdvIntervalLevel level, AdvEventHandler handler, BleGattServerCallback *callback);
  int (*stopAdvertising)(void);
  int (*startScan)(ScanMode mode, ScanEventHandler handler);
  int (*stopScan)(void);
  int (*addService)(BleGattService *service);
  int (*notifyCharacteristic)(uint16_t conn_handle, uint16_t handle, uint8_t *value, uint16_t length);
} BleAdapter;

BleAdapter* BleAdapterInit(void);
/*---------------------------------------------------------------------------*/
// OSFile
#define OSFILE_DATA_MAX_LENGTH 			64
#define OSFILE_DATA_MAX_OPEN 			  4

typedef enum
{
  SeekSet = 0x00,
  SeekCur,
  SeekEnd
} SeekMode;

typedef struct {
  int (*open)(int16_t *fd, const uint32_t key, const char *mode);
  int (*write)(const int16_t fd, const void *data, const uint32_t size);
  int (*read)(const int16_t fd, void *data, const uint32_t size);
  int (*seek)(const int16_t fd, const uint32_t offset, const SeekMode mode);
  int (*close)(const int16_t fd);
  int (*exist)(const uint32_t key);
  int (*size)(const uint32_t key);
  int (*remove)(const uint32_t key);
  int (*removeAll)(void);
  int (*space)(uint32_t *total, uint32_t *used);
} FileIO;

FileIO *OSFileIOInit(void);
/*---------------------------------------------------------------------------*/
// OSSensor
typedef enum {
  TYPE_MOTION = 0x00,
  TYPE_ORIENTATION,
  TYPE_PRESSURE,
  TYPE_HEARTRATE,
  TYPE_TEMPERTURE,
  TYPE_HUMIDITY,
  TYPE_BATTERY
} SensorType;

typedef enum {
  RATE_SLOW = 0x00,   // 10 hz
  RATE_NORMAL,        // 20 hz
  RATE_FAST,          // 50 hz
  RATE_FASTED         // 100 hz
} SensorRate;

typedef enum {
  RANGE_LOW = 0x00,   // 2g
  RANGE_MIDDLE,       // 8g
  RANGE_HIGH          // 16g
} SensorRange;

typedef struct {
  SensorType type;
  uint8_t data_len;
  uint8_t data[24];
  uint32_t timestamp;
} SensorEvent;

typedef void (* SensorEventHandler)(SensorEvent *event);

typedef struct {
  SensorRate    rate;
  SensorRange   range;
} SensorParams;

typedef struct {
  bool  (*isSupported)(void);
  int   (*open)(SensorParams *params);
  int   (*close)(void);
  char* (*getVendor)(void);
} OSSensor;

int OSSensorEventAttach(SensorEventHandler handler);
int OSSensorEventDetach(void);
OSSensor* OSSensorInit(SensorType type);
/*---------------------------------------------------------------------------*/
// OSTimer
#define OSTIMER_MAX_NUMBER  3
typedef void (* OSTimerHandler)(void);

typedef struct{
  int (*set)(uint8_t id, uint32_t timeout_ms, OSTimerHandler handler);
  int (*restart)(uint8_t id);
  int (*reset)(uint8_t id);
  int (*stop)(uint8_t id);
} OSTimer;

OSTimer* OSTimerInit(uint8_t *timer_id);
/*---------------------------------------------------------------------------*/
// OSTick
void OSTickDelayMs(uint16_t millis);
uint32_t OSTickGetCount(void);
/*---------------------------------------------------------------------------*/
// OSLog
typedef struct{
  int (*airPrint)(const char *fmt, ...);
  int (*rtPrint)(const char *fmt, ...);
} OSLog;

// Singleton instance of OSLog
OSLog* OSLogInit(void);
/*---------------------------------------------------------------------------*/
// Hardware
enum {
	BAUDRATE_1200 = 1200,
  BAUDRATE_2400 = 2400,
  BAUDRATE_4800 = 4800,
  BAUDRATE_9600 = 9600,
	BAUDRATE_19200 = 19200,
  BAUDRATE_38400 = 28400,
  BAUDRATE_57600 = 57600,
  BAUDRATE_115200 = 115200,
};

#define HWID_MAX_LEN									 8

#define PIN_INPUT_NOPULL  	        0x00     // Pin pullup resistor disabled
#define PIN_INPUT_PULLUP		        0x01     // Pin pulldown resistor enabled
#define PIN_INPUT_PULLDOWN		      0x02     // Pin pullup resistor enabled
#define PIN_OUTPUT_LOW		          0x03
#define PIN_OUTPUT_HIGH	   	        0x04

#define PIN_EDGE_RISING  		 			  0x00
#define PIN_EDGE_FALLING	     		  0x01

typedef void (* UartRxHandler)(uint8_t data);
typedef void (* PinWatchtHandler)(uint32_t pin);

typedef struct{
	int (* pinSet)(uint32_t pin, uint8_t value);
	int (* pinRead)(uint32_t pin, uint8_t *value);
	int (* pinInfo)(uint32_t *pin, uint8_t *size);
	int (* pinWatchSet)(uint32_t pin_mask, uint32_t edge_mask, PinWatchtHandler handler);
	int (* pinWatchClose)(void);
  int (* uartInit)(uint32_t pin_tx, uint32_t pin_rx, uint32_t baudrate, UartRxHandler handler);
  int (* uartDisable)(void);
  int (* uartSend)(uint8_t *data, uint32_t length);
  int (* getHWId)(uint8_t *hwid, int max_len);
} Hardware;

// Creating New Hardware Instances
Hardware* OSHardwareInit(void);
/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif /* __ALFALOOP_H_ */
