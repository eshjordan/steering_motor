#include <SPI.h>
#include <mcp2515.h>


MCP2515 mcp2515(10);

void initMotor(uint32_t, uint32_t, uint32_t);
void sendMessageCAN(uint32_t, uint8_t, uint64_t);

uint16_t sdoReceiveCobId = 0x64D;
static bool first = true;

/*Set these parameters*/
uint32_t velocityTarget;
uint32_t accelTarget;
uint32_t decelTarget;
int minEncoderIncrement;
int maxEncoderIncrement;
int convertedDeltaEncoderInc;


void setup() {
  // put your setup code here, to run once:
  while (!Serial) {}
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

}

void loop() {
  Serial.println("looping\n");
  delay(10);
}

uint32_t integerToLittleEnd(uint32_t val) {
  uint32_t temp = 0;
  uint32_t littleEnd = 0;
  
  for (int i = 0; i < 4; i++) {
    littleEnd = littleEnd << 8;
    littleEnd += (val && 0xFF);
    val = val >> 8;
  }

  return val;

}

void sendMessageCAN(uint32_t can_id, uint8_t can_dlc, uint64_t can_data) {
  uint8_t ar_can_data[8] = {0};

  for (int i = 7; i > -1; i--) {
    ar_can_data[i] = (uint8_t) (can_data && 0xFF);
    can_data = can_data >> 8;
  }

  struct can_frame canMsg;
  canMsg.can_id = can_id;
  canMsg.can_dlc = can_dlc;

  for (int i = 0; i < 8; i++) {
    canMsg.data[i] = ar_can_data[i];
  }

  mcp2515.sendMessage(&canMsg);
}

void initMotor(uint32_t prof_vel, uint32_t prof_accel, uint32_t prof_decel) {
  uint64_t can_data = 0;

  /*Clear the faults in the Control Word (6040h)*/ 
  can_data = 0x2B40600000000000;
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  can_data = 0x2B40600080000000;
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  can_data = 0x2B40600000000000;
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  /*Modes of Operation object (6060h)*/
  can_data = 0x2F60600001000000;
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  //Set Position Origin (2202h)
  can_data = 0x2302220000000000;
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  //Set Profile Velocity in PP Mode object (6081h)
  can_data = 0x2381600000000000 + integerToLittleEnd(velocityTarget);
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  //Set Profile Aceleration (6083h)
  can_data = 0x2383600000000000 + integerToLittleEnd(accelTarget);
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  //Set Profile Deceleration (6083h)
  can_data = 0x2384600000000000 + integerToLittleEnd(decelTarget);
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  //Set Negative software position limit (2205h)
  can_data = 0x2305220000000000 + integerToLittleEnd(minEncoderIncrement);
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  //Set Positive software position limit (2206h)
  can_data = 0x2306220000000000 + integerToLittleEnd(maxEncoderIncrement);
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

}

void updateSteeringAngle() {
  uint64_t can_data = 0;

  //PositionTargetState
  //Set target steering angle.
  can_data = 0x237A600000000000 + integerToLittleEnd(convertedDeltaEncoderInc);
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  //MotorMove1State
  if (first) {
    //Put the car in CiA402 'Ready to Switch on"
    can_data = 0x2B40600006000000;
    sendMessageCAN(sdoReceiveCobId, 8, can_data);
    first = false;
    delay(10);
  }
  
  //MotorMove2State
  //Configure PP to work with position relative targets
  //1/08/22 It might just be satisfactory to have this be only initialised.
  can_data = 0x2B4060006F000000;
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  //MotorMove3State (start motion)
  can_data = 0x2B4060007F000000;
  sendMessageCAN(sdoReceiveCobId, 8, can_data);
  delay(10);

  //Wait
  delay(10);
}



