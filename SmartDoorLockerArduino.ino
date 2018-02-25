#include "RampIOTControl.h"
#include <Servo.h>
#include <SoftwareSerial.h>
SoftwareSerial cameraAddonSerial(12, 11);

/*This type is mandatory for smart door implementation*/
#define THING_TYPE "SMART_DOOR"
/*This id must be unique*/
#define THING_ID 2396641

/*Storage Slots*/
#define EEPROM_LOCK_MOV_SLOT 1
#define EEPROM_UNLOCK_MOV_SLOT 2
#define PIN_SLOT 3
/*End Storage Slots*/

/*Pins definition*/
#define SERVO_POS_PIN A0
#define DOOR_OPCL_PIN 3
#define CONF_LOCK_PIN 6
#define CONF_UNLOCK_PIN 7
#define SERVO_PIN 9
#define RESET_PIN 10
/*End Pins definition*/

#define LOCK_EVENT 1
#define UNLOCK_EVENT 2
#define DELTA_MOV 12
#define POSITION_THRESHOLD 20
#define OC_DELAY 1000
#define SERVO_MIN 36
#define SERVO_MAX 509
#define ANGLE_MIN 0
#define ANGLE_MAX 180
#define ERR_MARGIN 5

RampIOTControl rampiotControl;
Servo locker;
bool movingServo = false;
bool configured = false;
bool doorOpen = false;
bool doorLock = false;
uint8_t currentEvent = 0;
uint8_t lockLimit = 0;
uint8_t unlockLimit = 0;
unsigned long lastOCEventTime = 0;
char fireUserId[15];
char pin[6];

/*This method read the internal servo potentiometer and converts it on servo angle using arduino map function*/
uint16_t getServoPosition(){
  uint16_t servoF = analogRead(SERVO_POS_PIN); 
  servoF = servoF < SERVO_MIN ? SERVO_MIN : servoF > SERVO_MAX ? SERVO_MAX : servoF;  
  servoF = map(servoF, SERVO_MIN, SERVO_MAX, ANGLE_MIN, ANGLE_MAX);
  servoF = ANGLE_MAX - servoF;
  return servoF > ERR_MARGIN ? servoF-ERR_MARGIN : servoF;
}

void handleConfiguration(){  
  /*When user configure locker manually, then when press lock button it saves lock position on EEPROM*/
  if( !configured && digitalRead(CONF_LOCK_PIN) == LOW ){
    Storage storage;
    lockLimit = getServoPosition();
    char lockPositionStr[5];
    itoa(lockLimit, lockPositionStr, 10);    
    storage.saveData(EEPROM_LOCK_MOV_SLOT, lockPositionStr);    
    delay(500);
  }
  /*When user configure locker manually, then when press unlock button it saves unlock position on EEPROM*/
  else if( !configured && digitalRead(CONF_UNLOCK_PIN) == LOW ){
    Storage storage;
    unlockLimit = getServoPosition();
    char unlockPositionStr[5];
    itoa(unlockLimit, unlockPositionStr, 10);
    storage.saveData(EEPROM_UNLOCK_MOV_SLOT, unlockPositionStr);
    delay(500);
  }
}

/*Reading and notifying sensors status like door open/close sensor and servo lock/unlock position*/
void handleSensors(){
  uint8_t doorOC = digitalRead(DOOR_OPCL_PIN);
  if( (lastOCEventTime == 0 || millis()-lastOCEventTime > OC_DELAY) && 
      ((doorOpen && doorOC == LOW) || (!doorOpen && doorOC == HIGH)) ){
    doorOpen = !doorOpen;
    lastOCEventTime = millis();
    DynamicJsonBuffer jsonBuffer;
    JsonObject& status = jsonBuffer.createObject();
    status["state"] = doorOpen ? "opened" : doorLock ? "closed_locked": "closed_unlocked";
    status["event"] = doorOpen ? "open" : "close";
    rampiotControl.publishEvent(status, "MANUAL");
  }
  if( configured && (getServoPosition() == lockLimit || getServoPosition() == lockLimit-POSITION_THRESHOLD 
      || getServoPosition() == lockLimit+POSITION_THRESHOLD) && !doorLock ){
    doorLock = true;
    DynamicJsonBuffer jsonBuffer;
    JsonObject& status = jsonBuffer.createObject();
    status["state"] = "closed_locked";
    status["event"] = "lock";
    rampiotControl.publishEvent(status, fireUserId[0] != '\0' ? fireUserId : "MANUAL");
    fireUserId[0] = '\0';
  }
  else if(  configured && (getServoPosition() == unlockLimit || getServoPosition() == unlockLimit-POSITION_THRESHOLD 
      || getServoPosition() == unlockLimit+POSITION_THRESHOLD) && doorLock ){
    doorLock = false;
    DynamicJsonBuffer jsonBuffer;
    JsonObject& status = jsonBuffer.createObject();
    status["state"] = "closed_unlocked";
    status["event"] = "unlock";
    rampiotControl.publishEvent(status, fireUserId[0] != '\0' ? fireUserId : "MANUAL");
    fireUserId[0] = '\0';
  }
}

void sendDoorOpenStatus(){
  DynamicJsonBuffer jsonBuffer;
  JsonObject& status = jsonBuffer.createObject();
  status["state"] = "opened";
  status["event"] = "open";
  rampiotControl.publishEvent(status, "MANUAL");
}

void handleLockUnlockFireEvent(){
  switch(currentEvent){
     case LOCK_EVENT:
     /*If door is open then locker can't lock/unlock*/
     if( doorOpen ){
      currentEvent = 0;
      sendDoorOpenStatus();
      return;      
     }
     /*When servo lock movement is completed, then detach servo for allow manually lock/unlock*/
     if( getServoPosition() == lockLimit || 
        (unlockLimit < lockLimit && getServoPosition()+ERR_MARGIN >= lockLimit) || 
        (unlockLimit > lockLimit && getServoPosition()-ERR_MARGIN <= lockLimit) ){
       movingServo = false;
       locker.detach();
       currentEvent = 0;
     }
     else if( !movingServo ){
        movingServo = true;
        locker.attach(SERVO_PIN);
        locker.write(lockLimit);
     }
     break;
     case UNLOCK_EVENT:    
     /*If door is open then locker can't lock/unlock*/
     if( doorOpen ){
      currentEvent = 0;
      sendDoorOpenStatus();
      return;      
     }     
     /*When servo unlock movement is completed, then detach servo for allow manually lock/unlock*/
     if( getServoPosition() == unlockLimit || 
        (unlockLimit > lockLimit && getServoPosition()+ERR_MARGIN >= unlockLimit) || 
        (unlockLimit < lockLimit && getServoPosition()-ERR_MARGIN <= unlockLimit) ){
       movingServo = false;
       locker.detach();
       currentEvent = 0;
     }
     else if( !movingServo ){
        movingServo = true;
        locker.attach(SERVO_PIN);
        locker.write(unlockLimit);
     }
     break;
  }
}

/*Reset callback handler, after send reset signal to RampIOTShield, 
  then clear local storage for allow configure lock and unlock limits*/
void onReset(){
  Storage storage;
  storage.clearAll();
}

/*Message callback handler, when message arrives from MQTT server, 
  then this method will be called with message received*/
void onMessage(const char* topic, JsonObject& json, const char* fUserId){
  const char* event = json["event"];      
  if( fUserId ){
    strcpy(fireUserId, fUserId);  
  }  
  if( strcmp(event, "lock") == 0 ){    
    currentEvent = LOCK_EVENT;    
  }
  /*If user unlock using pin number*/
  else if( strcmp(event, "pin_unlock") == 0 ){
    const char* _pin = json["pin"];
    /*Only unlock if user pin is equals to device pin*/
    if( _pin && strcmp(pin, _pin) == 0 ){
        currentEvent = UNLOCK_EVENT; 
    }
  }
  else if( strcmp(event, "unlock") == 0 || strcmp(event, "face_unlock") == 0 ){
    currentEvent = UNLOCK_EVENT;
  }
}

/*
 * This callback method will be called when device login and every time that user update PIN number on App
*/
void onProperties(JsonObject& properties){
  const char* _pin = properties["pin"];
  if( _pin ){
    /*Updating device pin*/
    strcpy(pin, _pin);
  }  
}

void setup() {
  Storage storage;
  configured = !storage.isEmpty(EEPROM_LOCK_MOV_SLOT) && 
               !storage.isEmpty(EEPROM_UNLOCK_MOV_SLOT);  
  if( configured ){
    char lockMoveStr[5];
    char unlockMoveStr[5];
    storage.getData(EEPROM_LOCK_MOV_SLOT, lockMoveStr);
    storage.getData(EEPROM_UNLOCK_MOV_SLOT, unlockMoveStr);
    lockLimit = atoi(lockMoveStr);
    unlockLimit = atoi(unlockMoveStr);        
  }  
  pinMode(DOOR_OPCL_PIN, INPUT_PULLUP);
  pinMode(CONF_LOCK_PIN, INPUT_PULLUP);
  pinMode(CONF_UNLOCK_PIN, INPUT_PULLUP);
  /*Begin serial for camera addon*/
  cameraAddonSerial.begin(9600);
  /*Must be pull up because if it's DOWN then reset*/
  pinMode(RESET_PIN, INPUT_PULLUP);
  rampiotControl.begin(
    &Serial, //HardwareSerial
    RESET_PIN, //Reset PIN
    THING_ID, //Thing ID
    THING_TYPE //Thing type
  );
  rampiotControl.setMessageCallback(onMessage);
  rampiotControl.setResetCallback(onReset);
  rampiotControl.setPropertiesCallback(onProperties);
}

String camBuff = "";
void handleCamera(){
  while (cameraAddonSerial.available()) {
    char data = (char)cameraAddonSerial.read();
    if( data == '\n' ){
      char status[camBuff.length()+1];
      camBuff.toCharArray(status, camBuff.length()+1);
      rampiotControl.publishEvent(status, "CAMERA");
      camBuff = "";
    }else{
      camBuff.concat(data);   
    }
  }
}

void loop() {
  if( !configured ){
     handleConfiguration();
  }else{
    handleLockUnlockFireEvent(); 
    handleSensors();
    handleCamera();
  }  
  rampiotControl.handleThing();
}
