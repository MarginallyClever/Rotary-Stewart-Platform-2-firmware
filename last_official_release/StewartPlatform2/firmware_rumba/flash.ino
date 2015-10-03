//--------------------------------------------------------
// Evil Minion 5 Axis robot firmware
// dan@marginallyclever.com 
// 2015 September 3
// see http://evilminion.info/ for more information.
//--------------------------------------------------------
#include "configuration.h"
#include <EEPROM.h>




//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
void EEPROM_writeLong(int ee, long value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  EEPROM.write(ee++, *p++);
}


//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
long EEPROM_readLong(int ee) {
  long value = 0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  *p++ = EEPROM.read(ee++);
  return value;
}


//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
void EEPROM_writeFloat(int ee, float value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  EEPROM.write(ee++, *p++);
}


//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
float EEPROM_readFloat(int ee) {
  float value = 0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  *p++ = EEPROM.read(ee++);
  return value;
}


//------------------------------------------------------------------------------
void loadConfig() {
  char version_number=EEPROM.read(ADDR_VERSION);
  
  if(version_number!=EEPROM_VERSION) {
    // If not the current EEPROM_VERSION or the EEPROM_VERSION is sullied (i.e. unknown data)
    // Update the version number
    EEPROM.write(ADDR_VERSION,EEPROM_VERSION);

    // upgrade path from one version to the next, if needed.
    if(version_number==0) {
      // Update robot uuid
      robot_uid=0;
      saveUID();
    } else {
      // Code should not get here if it does we should display some meaningful error message
      Serial.println(F("An Error Occurred during LoadConfig"));
    }
  }

  robot_uid = EEPROM_readLong(ADDR_GUID);
  Serial.println(F("Calibration loaded."));
}


/**
 * Save the sensor adjustments (calibration offsets)
 */
void saveAdjustments() {
}


/**
 * Save the robot's unique ID
 */
void saveUID() {
  EEPROM_writeLong(ADDR_GUID,robot_uid);
}

