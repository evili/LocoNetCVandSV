#ifndef   __LOCONET_CV_AND_SV_H__
#define   __LOCONET_CV_AND_SV_H__

#define ARDUINO_SERIAL_NUMBER_LEN 9u
#define MANUFACTURER_ID   13     // DIY Manufacturer Id
#define DEVELOPER_ID      42     // EdR (?)
#define PRODUCT_ID         1     // LocoServo
#define VERSION          001     // Version
#define DEFAULT_NODE_ID_L 0x00
#define DEFAULT_NODE_ID_H 0x01
// EEPROM Offset where begins our configuration.
#define EEPROM_SERVO_OFFSET  ((SV_ADDR_USER_BASE)-2)

#define NUM_SERVO_DRIVERS 1

//Adresses of Drivers
#define SERVO_DRIVER_0_ADDRESS 0x40


// #define SERVO_DRIVER_1_ADDRESS 0x41
// etc...
#define MAX_NUM_DRIVERS   12
#define SERVOS_PER_DRIVER 16

#define SERVO_MIN_POS  150u // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_MAX_POS  600u // this is the 'maximum' pulse length count (out of 4096)

typedef struct servo_driver_s {
  uint8_t address;
  uint16_t servos_bitmask;
} servo_driver_t;

typedef struct servo_s {
  uint16_t address;
  uint16_t open_pos;
  uint16_t closed_pos;
} servo_t;

typedef struct loco_servo_config_s {
  uint8_t config_crc;
  uint8_t num_drivers;
  servo_driver_t drivers[MAX_NUM_DRIVERS];
  servo_t servos[MAX_NUM_DRIVERS * SERVOS_PER_DRIVER];
} loco_servo_t;


typedef union config_eeprom_u {
  loco_servo_t loco_servo_cfg;
  uint8_t data[sizeof(loco_servo_t)];
} config_eeprom_t;

#endif // __LOCONET_CV_AND_SV_H__
