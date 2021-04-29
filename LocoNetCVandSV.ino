#include <avr/boot.h>
#include <util/crc16.h>
#include <assert.h>
#include <LocoNet.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

#include "LocoNetCVandSV.h"

#define LOCONET_TX_PIN 7
#define LNCV_COUNT 16

// PCA9685
#define ARTNR 9685 

lnMsg *LnPacket;

/*
   LNCV variables
*/
LocoNetCVClass lnCV;
boolean programmingMode;
uint16_t lncv[LNCV_COUNT];

void commitLNCVUpdate() {
  Serial.print("Module Address is now: ");
  Serial.print(0);
  Serial.print("\n");
}


/*
   LNSV variables
*/
LocoNetSystemVariableClass sv;
SV_STATUS   svStatus = SV_OK;
boolean     deferredProcessingNeeded = false;

/*
 * Other variables
 * 
 */
config_eeprom_t Config;

loco_servo_t *LocoServo;

LocoNetSystemVariableClass LocoNetSV;
SV_STATUS SvStatus;

uint8_t serial_number[ARDUINO_SERIAL_NUMBER_LEN];


/*
 * EEPROM init
 * 
 */
void init_EEPROM()
{
  serial_number[0] = boot_signature_byte_get(0x000E);
  serial_number[1] = boot_signature_byte_get(0x000F);
  serial_number[2] = boot_signature_byte_get(0x0010);
  serial_number[3] = boot_signature_byte_get(0x0011);
  serial_number[4] = boot_signature_byte_get(0x0012);
  serial_number[5] = boot_signature_byte_get(0x0013);
  serial_number[6] = boot_signature_byte_get(0x0015);
  serial_number[7] = boot_signature_byte_get(0x0016);
  serial_number[8] = boot_signature_byte_get(0x0017);

  uint16_t config_crc = 0xFFFF;
  config_crc = _crc16_update(config_crc, MANUFACTURER_ID);
  config_crc = _crc16_update(config_crc, DEVELOPER_ID);
  config_crc = _crc16_update(config_crc, PRODUCT_ID);
  config_crc = _crc16_update(config_crc, VERSION);

  for(int i=0; i<ARDUINO_SERIAL_NUMBER_LEN; i++)
    config_crc = _crc16_update(config_crc, serial_number[i]);

  uint8_t eeprom_crc = EEPROM.read(EEPROM_SERVO_OFFSET);
  if(eeprom_crc == config_crc)
    return; 

  // No previous config. So create default one
  EEPROM.write(SV_ADDR_NODE_ID_L-2, DEFAULT_NODE_ID_L);
  EEPROM.write(SV_ADDR_NODE_ID_H-2, DEFAULT_NODE_ID_H);

  // Build a serial number based on the 9 signature SN bytes
  uint16_t SN = 0xFFFF;
  uint16_t b;
  for(int i=0; i<ARDUINO_SERIAL_NUMBER_LEN; i++) {
    b = serial_number[i];
    SN = _crc16_update(config_crc, b);
  }

  EEPROM.write(SV_ADDR_SERIAL_NUMBER_L-2, (SN & 0x00FF)     );
  EEPROM.write(SV_ADDR_SERIAL_NUMBER_H-2, (SN & 0xFF00) >> 8);

  LocoServo->config_crc = config_crc;
  // Default to 1 servo Driver with 16 servos attached
  LocoServo->num_drivers = 1;
  LocoServo->drivers[0].address = SERVO_DRIVER_0_ADDRESS;
  LocoServo->drivers[0].servos_bitmask = 0xFFFF;
  for(int s = 0; s < SERVOS_PER_DRIVER; s++) {
    LocoServo->servos[s].address    = s+1;
    LocoServo->servos[s].open_pos   = SERVO_MIN_POS;
    LocoServo->servos[s].closed_pos = SERVO_MAX_POS; 
  }
  // Write to EEPROM
  for (int i = 0; i < sizeof(loco_servo_t); i++)
    EEPROM.write(EEPROM_SERVO_OFFSET+i, Config.data[i]);
}


void setup() {
  Serial.begin(115200);
  Serial.print("Starting LNCVandSV-test\n");
  Serial.print("EEPROM Size: ");
  Serial.println(E2END, HEX);
  LocoNet.init(LOCONET_TX_PIN);
  
  commitLNCVUpdate();
  programmingMode = false;
  
  sv.init(13, 4, 1, 1);
  sv.writeSVStorage(SV_ADDR_NODE_ID_H, 1 );
  sv.writeSVStorage(SV_ADDR_NODE_ID_L, 0);
  sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_H, 0x56);
  sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_L, 0x78);
}

void loop() {
  LnPacket = LocoNet.receive();

  if ( LnPacket )
  {
    uint8_t packetConsumed(LocoNet.processSwitchSensorMessage(LnPacket));
    if (packetConsumed == 0) {
      Serial.print("Loop ");
      Serial.print((int)LnPacket);
      packetConsumed = lnCV.processLNCVMessage(LnPacket);
      Serial.print("End Loop\n");
    }
    if (packetConsumed == 0) {
      svStatus = sv.processMessage(LnPacket);
      Serial.print("LNSV processMessage - Status: ");
      Serial.println(svStatus);
      deferredProcessingNeeded = (svStatus == SV_DEFERRED_PROCESSING_NEEDED);
    }

    if (deferredProcessingNeeded)
      deferredProcessingNeeded = (sv.doDeferredProcessing() != SV_OK);


  }
}

/*
   LNSV Callback(s)

*/
void  notifySVChanged(uint16_t Offset) {
  // Check if changed SV is "ours"?
  if (Offset < SV_ADDR_USER_BASE)
    return;
  // Offset is a SV "raw" Offset.
  // config_offset counts from Config struct start
  // eeprom_offset counts from EEPROM start
  uint16_t config_offset = Offset - SV_ADDR_USER_BASE;
  uint16_t eeprom_offset = Offset - EEPROM_SERVO_OFFSET;

  Config.data[config_offset] = EEPROM.read(eeprom_offset);
  // Check Num drivers is supported
  assert(LocoServo->num_drivers <= MAX_NUM_DRIVERS);

  uint16_t servo_offset = config_offset - (sizeof(servo_driver_t) +1);
  if (
    // Have we changed a servo?
    (servo_offset >= 0) &&
    // Have we changed a open/closed position?
    ((servo_offset % sizeof(servo_t)) != 0) ) {
    // Move servo to the closed (safe) position.
    servoMove(servo_offset / sizeof(servo_t), false);
  }
}

/* .
   LNCV Callbacks

*/
int8_t notifyLNCVread(uint16_t ArtNr, uint16_t lncvAddress, uint16_t,
                      uint16_t & lncvValue) {
  Serial.print("Enter notifyLNCVread(");
  Serial.print(ArtNr, HEX);
  Serial.print(", ");
  Serial.print(lncvAddress, HEX);
  Serial.print(", ");
  Serial.print(", ");
  Serial.print(lncvValue, HEX);
  Serial.print(")");
  // Step 1: Can this be addressed to me?
  // All ReadRequests contain the ARTNR. For starting programming, we do not accept the broadcast address.
  if (programmingMode) {
    if (ArtNr == ARTNR) {
      if (lncvAddress < LNCV_COUNT) {
        lncvValue = lncv[lncvAddress];
        Serial.print(" LNCV Value: ");
        Serial.print(lncvValue);
        Serial.print("\n");
        return LNCV_LACK_OK;
      } else {
        // Invalid LNCV address, request a NAXK
        return LNCV_LACK_ERROR_UNSUPPORTED;
      }
    } else {
      Serial.print("ArtNr invalid.\n");
      return -1;
    }
  } else {
    Serial.print("Ignoring Request.\n");
    return -1;
  }
}

int8_t notifyLNCVprogrammingStart(uint16_t & ArtNr, uint16_t & ModuleAddress) {
  // Enter programming mode. If we already are in programming mode,
  // we simply send a response and nothing else happens.
  Serial.print("notifyLNCVProgrammingStart ");
  if (ArtNr == ARTNR) {
    Serial.print("artnrOK ");
    if (ModuleAddress == lncv[0]) {
      Serial.print("moduleUNI ENTERING PROGRAMMING MODE\n");
      programmingMode = true;
      return LNCV_LACK_OK;
    } else if (ModuleAddress == 0xFFFF) {
      Serial.print("moduleBC ENTERING PROGRAMMING MODE\n");
      ModuleAddress = lncv[0];
      return LNCV_LACK_OK;
    }
  }
  Serial.print("Ignoring Request.\n");
  return -1;
}

/**
   Notifies the code on the reception of a write request
*/
int8_t notifyLNCVwrite(uint16_t ArtNr, uint16_t lncvAddress,
                       uint16_t lncvValue) {
  Serial.print("notifyLNCVwrite, ");
  //  dumpPacket(ub);
  if (!programmingMode) {
    Serial.print("not in Programming Mode.\n");
    return -1;
  }

  if (ArtNr == ARTNR) {
    Serial.print("Artnr OK, ");

    if (lncvAddress < LNCV_COUNT) {
      lncv[lncvAddress] = lncvValue;
      return LNCV_LACK_OK;
    }
    else {
      return LNCV_LACK_ERROR_UNSUPPORTED;
    }
  }
  else {
    Serial.print("Artnr Invalid.\n");
    return -1;
  }
}

/**
   Notifies the code on the reception of a request to end programming mode
*/
void notifyLNCVprogrammingStop(uint16_t ArtNr, uint16_t ModuleAddress) {
  Serial.print("notifyLNCVprogrammingStop ");
  if (programmingMode) {
    if (ArtNr == ARTNR && ModuleAddress == lncv[0]) {
      programmingMode = false;
      Serial.print("End Programing Mode.\n");
      commitLNCVUpdate();
    }
    else {
      if (ArtNr != ARTNR) {
        Serial.print("Wrong Artnr.\n");
        return;
      }
      if (ModuleAddress != lncv[0]) {
        Serial.print("Wrong Module Address.\n");
        return;
      }
    }
  }
  else {
    Serial.print("Ignoring Request.\n");
  }
}

/*
 * servo_move
 * 
 */
void servoMove(uint16_t servo, boolean open) {
  uint16_t driver = servo / SERVOS_PER_DRIVER;
  uint8_t servo_num = servo % SERVOS_PER_DRIVER;
  // drivers[driver];
}


/*
 *  __assert: handle diagnostic informations given by assertion and abort program execution.
 *  
 */
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
    // transmit diagnostic informations through serial link.
    Serial.println(__func);
    Serial.println(__file);
    Serial.println(__lineno, DEC);
    Serial.println(__sexp);
    Serial.flush();
    // abort program execution.
    abort();
}
