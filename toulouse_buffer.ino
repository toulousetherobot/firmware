#include <FastCRC.h>
#include <FastCRC_cpu.h>
#include <FastCRC_tables.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

FastCRC16 CRC16;

// Comm constants
const byte start_byte = 0xAB;
const byte end_byte = 0xCD;
const byte home_byte = 0x12;
const byte stop_byte = 20; 

const byte ack = 41;
const byte nack = 40;

const uint16_t crc_cheat = 12345;

const int frameSize = 12;
int bytesToRead = 0;
String message = "";
int ackno = 0;


struct Frame {
  unsigned char SFD;
  unsigned char VERSION;
  unsigned char CODE;
  int16_t THETA1;
  int16_t THETA2;
  int16_t D3;
  uint16_t CRC;
  unsigned char EFD;
} __attribute__((packed));

const char VOID = 'x';

union FrameUnion {
  struct Frame frame;
  byte bytes[frameSize];
} __attribute__((packed)) frame;

byte frameBytes[frameSize];
int frameIndex = 0;

//Buffer params
const int bufferSize = 5;
int bufferReadIndex = 0;
int bufferWriteIndex = 0;
Frame frameBuffer[bufferSize];
boolean storedAck = false;

// Motor parameters
const int stepsPerRotation = 2746;
long actMoveTimeMillis = 1000;
long actWaitTimeMillis = 500;
long actLast = 0;

const float motor_move_speed = 1000.0;
const float motor_accel = 800.0;
const float motor_home_speed = 100.0;
const int act_min = 20;


// Homing parameters
const int m1_pos_dir = -1;
const int m2_pos_dir = 1;

const int m1_home_dir = -1;
const int m2_home_dir = 1;
const int m1_home_backoff_steps = 0;
const int m2_home_backoff_steps = 50;

const int m1_home_position = (int)((1.0/360) * stepsPerRotation);
const int m2_home_position = (int)((151.0/360) * stepsPerRotation);

// Pins

const int limit_5A_pin = 39;
const int limit_4A_pin = 35;
const int limit_3A_pin = 31;
const int limit_2B_pin = 29;
const int limit_2A_pin = 27;
const int limit_1B_pin = 25;
const int limit_1A_pin = 23;

const int m1_limit_pin = limit_1A_pin;
const int tapeswitch_pin = limit_1B_pin;

const int m1_step_pin = 9;
const int m1_dir_pin = 8;
const int m2_step_pin = 11;
const int m2_dir_pin = 10;
const int m3_step_pin = 13;
const int m3_dir_pin = 12;
const int m4_step_pin = 7;
const int m4_dir_pin = 6;
const int m5_step_pin = 5;
const int m5_dir_pin = 4;


const int act_dir_pin = 3;
const int act_en_pin = 2;

//const int estop_pin = 1;
const int estop_pin = limit_2A_pin;

const int act_in_pin = A0;
const int bumper_pin = A1;
const int resume_pin = limit_5A_pin;


// States
const int state_moving = 1;
const int state_idle = 2;
const int state_receiving = 3;

const int comm_idle = 1; // No bits to read
const int comm_receiving = 2; // Receiving message
const int comm_finished = 3; // Received EOF

const int state_not_homing = 0;
const int state_homing_actuator = 1;
const int state_homing_shoulder = 2;
const int state_backing_shoulder = 3;
const int state_homing_elbow = 4;
const int state_backing_elbow = 5;

const int tapeswitch_pressed = LOW;
const int tapeswitch_unpressed = HIGH;

const int microswitch_pressed = HIGH;
const int microswitch_unpressed = LOW;

const int estop_pressed = LOW;
const int estop_unpressed = HIGH;

// Variables
int act_target = 15;
int state = state_idle;
int comm_state = comm_idle;
int homing_state = state_not_homing;

boolean actuator_moving_up = false;
boolean receiving = false;
FrameUnion frameUnion;

// Motors
AccelStepper m1(AccelStepper::DRIVER, m1_step_pin, m1_dir_pin);
AccelStepper m2(AccelStepper::DRIVER, m2_step_pin, m2_dir_pin);
//AccelStepper m3(AccelStepper::DRIVER, m3_step_pin, m3_dir_pin);
//AccelStepper m4(AccelStepper::DRIVER, m4_step_pin, m4_dir_pin);
//AccelStepper m5(AccelStepper::DRIVER, m5_step_pin, m5_dir_pin);
//AccelStepper motors[2] = {m1, m2};
//int numMotors = 2;

// Debug variables
long hb = 0;
long interval = 20000;

const int headlessLength = 9;
struct Frame headlessFrame;
int16_t headlessTheta1[] =   {000, 100, 000, 000, -100, -100, -40, -100, 0};
int16_t headlessTheta2[] =   {000, 000, 100, 100, -100, -100, 0000, 0000, 0};
int16_t headlessActuator[] = {630, 630, 630, 020, 0020, 0630, 0630, 20,  20};
int headlessIndex = 0;

boolean headless = false;
boolean allowSerialInput = true;
void setup() {

  pinMode(m1_step_pin, OUTPUT);
  pinMode(m1_dir_pin, OUTPUT);
  pinMode(m2_step_pin, OUTPUT);
  pinMode(m2_dir_pin, OUTPUT);
  pinMode(m3_step_pin, OUTPUT);
  pinMode(m3_dir_pin, OUTPUT);
  pinMode(m4_step_pin, OUTPUT);
  pinMode(m4_dir_pin, OUTPUT);
  pinMode(m5_step_pin, OUTPUT);
  pinMode(m5_dir_pin, OUTPUT);
  pinMode(limit_1A_pin, INPUT);
  pinMode(act_en_pin, OUTPUT);
  pinMode(act_dir_pin, OUTPUT);
  pinMode(act_in_pin, INPUT);

  Serial.begin(9600);
 
  Serial3.begin(115200);
  /*
    Serial3.write(171);
    Serial3.write('A');
    Serial3.write(100);
    Serial3.write(1);
    Serial3.write(200);
    Serial3.write(2);
    Serial3.write(300);
    Serial3.write(3);
    Serial3.write(400);
    Serial3.write(4);
    Serial3.write(205);
  */

 
  m1.setSpeed(motor_move_speed);
  m1.setAcceleration(motor_accel);

  m1.setCurrentPosition(-stepsPerRotation/4);
  
  m2.setSpeed(motor_move_speed);
  m2.setAcceleration(motor_accel);

  state = state_moving;

  for (int i = 0; i < bufferSize; i++) {
    frameBuffer[i].VERSION = VOID;
  }
  
}


void loop() {
  if (motion() && frameBuffer[bufferReadIndex].VERSION != VOID) {
    executeFrame(frameBuffer[bufferReadIndex]);
    frameBuffer[bufferReadIndex].VERSION = VOID;
    bufferReadIndex = (bufferReadIndex + 1) % bufferSize;
    //Serial.print(F("Finished moving. Running frame ")); Serial.println(bufferReadIndex);

    if (headless) {
      headlessFrame.THETA1 = headlessTheta1[headlessIndex];
      headlessFrame.THETA2 = headlessTheta2[headlessIndex];
      headlessFrame.D3 = headlessActuator[headlessIndex];
      executeFrame(headlessFrame);
      state = state_moving;
      headlessIndex = (headlessIndex + 1) % headlessLength;
    }
  }
  
  comm_state = comm();

  if (comm_state == comm_finished) {
    struct Frame frame = frameUnion.frame;
    uint16_t crc = CRC16.ccitt(frameBytes, frameSize-3);
    
    //Serial.println(F("Finished receiving: \""));  Serial.print(message); Serial.println("\"");
    if (crc == frame.CRC || frame.CRC == crc_cheat) {
      //Serial.println(F("CRC correct! Moving now."));
      frameBuffer[bufferWriteIndex] = frame;
      //Serial.print("Writing frame to index "); Serial.println(bufferWriteIndex);
      bufferWriteIndex = (bufferWriteIndex + 1) % bufferSize;
      if (frameBuffer[bufferWriteIndex].VERSION == VOID) {
        sendMessage(ack);
        //Serial.println("And requesting new packet!");
      } else{
        //Serial.println("And storing ack.");
        storedAck = true;
      }
    } else {
      //Serial.println(F("CRC was wrong. Sending nack."));
      sendMessage(nack);
    }
    
  }
  
  if (storedAck && frameBuffer[bufferWriteIndex].VERSION == VOID) {
    sendMessage(ack);
    //Serial.println("Requesting new packet!");
    storedAck = false;
  }

  if (allowSerialInput) {
    parseSerialInput();
  }
  

}

// Returns true if all motion is complete, false if there are still steps to go.
boolean motion() {

  m1.run();
  m2.run();

  if (homing_state != state_homing_elbow && homing_state != state_backing_elbow) {
    checkTapeswitch();
  }
  if (homing_state != state_not_homing) {
    if (homing_state == state_homing_actuator && runActuator() == 1) {
      m1.setMaxSpeed(motor_home_speed);
      m2.setMaxSpeed(motor_home_speed);
      m1.setCurrentPosition(0);
      m2.setCurrentPosition(0);
      m1.moveTo(stepsPerRotation*m1_home_dir*m1_pos_dir);
      homing_state = state_homing_shoulder;
      Serial.println("1");
    } else if (homing_state == state_homing_shoulder && digitalRead(m1_limit_pin) == microswitch_pressed){
      homing_state = state_backing_shoulder;
      m1.setCurrentPosition(0);
      m1.moveTo(m1_home_backoff_steps*m1_home_dir*m1_pos_dir*-1);
      Serial.println("2");
    } else if (homing_state == state_backing_shoulder && m1.distanceToGo() == 0) {
      homing_state = state_homing_elbow;
      m2.moveTo(stepsPerRotation*m2_home_dir*m2_pos_dir);
      Serial.println("3");
    } else if (homing_state == state_homing_elbow && digitalRead(tapeswitch_pin) == tapeswitch_pressed) {
      homing_state = state_backing_elbow;
      m2.setCurrentPosition(0);
      m2.moveTo(m2_home_backoff_steps*m2_home_dir* m2_pos_dir * -1);
      Serial.println("4");
    } else if (homing_state == state_backing_elbow && runActuator() == 1 && m2.distanceToGo() == 0) {
      m1.setCurrentPosition((m1_home_position - m1_home_backoff_steps * m1_home_dir) * m1_pos_dir);
      m1.moveTo((m1_home_position + m1_home_backoff_steps * m1_home_dir) * m1_pos_dir);
      m2.setCurrentPosition((m2_home_position - m2_home_backoff_steps * m2_home_dir) * m2_pos_dir);
      m2.moveTo((m2_home_position + m2_home_backoff_steps * m2_home_dir) * m2_pos_dir);
      m1.setMaxSpeed(motor_move_speed);
      m2.setMaxSpeed(motor_move_speed);
      homing_state = state_not_homing;
      m1.moveTo(0.25*stepsPerRotation*m1_pos_dir);
      m2.moveTo(0);
      Serial.println("5");
      return true;
    }
    return false;
  }

  boolean done = true;
  
  if (runActuator() == 0) {
    done = false;
  }
  
  if (m1.distanceToGo() != 0 || m2.distanceToGo() != 0) {
    done = false;
  }
  
  return done;
}

int runActuator() {
  int act_pos = analogRead(act_in_pin);
  if ((actuator_moving_up && act_target >= act_pos) || (!actuator_moving_up && act_target <= act_pos)) {
    digitalWrite(act_en_pin, LOW);
    return 1;
  }
  
  if (act_pos > act_target) {
    digitalWrite(act_dir_pin, LOW);
    digitalWrite(act_en_pin, HIGH);
  } else if (act_pos < act_target) {
    digitalWrite(act_dir_pin, HIGH);
    digitalWrite(act_en_pin, HIGH);
  } 
  return 0;
}

int comm() {
  if (Serial3.available() > 0) {
    byte incomingByte = Serial3.read();
    //Serial.print(F("Raw: ")); Serial.println(incomingByte);
    if (incomingByte == start_byte && !receiving) {
      message = "";
      message = message + incomingByte + " ";
      receiving = true;
      frameBytes[0] = incomingByte;
      frameIndex = 1;
      return comm_receiving;
    } else if (receiving && frameIndex == frameSize - 1) {
      frameBytes[frameIndex] = incomingByte;
      memcpy(frameUnion.bytes, frameBytes, 11);
      receiving = false;
      return comm_finished;
    } else if (receiving) {
      message = message + incomingByte + " ";
      frameBytes[frameIndex] = incomingByte;
      frameIndex++;
      return comm_receiving;
    }
  }
  return comm_idle;
}

void executeFrame(Frame frame) {
  
  if (frame.CODE == home_byte) {
    actuator_moving_up = true;
    act_target = act_min;
    homing_state = state_homing_actuator;
    return;
  }
  
  m1.moveTo(m1_pos_dir*frame.THETA1);
  m2.moveTo(m2_pos_dir*frame.THETA2);
  if (act_target > frame.D3) {
    actuator_moving_up = true;
  } else {
    actuator_moving_up = false;
  }
  act_target = frame.D3;
}

void printFrame(Frame frame, uint16_t crc) {
  Serial.print("Version: "); Serial.println(frame.VERSION);
  Serial.print("Code: "); Serial.println(frame.CODE);
  Serial.print("Theta1: "); Serial.println(frame.THETA1);
  Serial.print("Theta2: "); Serial.println(frame.THETA2);
  Serial.print("D3: "); Serial.println(frame.D3);
  Serial.print("CRC: "); Serial.println(frame.CRC);
  Serial.print("Computed CRC: "); Serial.println(crc);
}

void sendMessage(byte b) {
  Serial3.write(start_byte);
  Serial3.write(1);
  Serial3.write(b);
  Serial3.write(end_byte);
}

void checkTapeswitch() {
  if (digitalRead(tapeswitch_pin) == tapeswitch_pressed) {
    Serial.println("Tapeswitch hit! Shutting down!");
    shutdown();
  }
}

void checkEstop() {
  if (digitalRead(estop_pin) == estop_pressed) {
    Serial.println("Estop pressed! Shutting down!");
    shutdown();
  }
 }
void shutdown() {
  sendMessage(stop_byte);
  while(digitalRead(resume_pin) != microswitch_pressed){
    
  }
}

void parseSerialInput() {
  if (Serial.available()) {
    int d = Serial.parseInt();
    Serial.print("Got "); Serial.println(d);
    if (act_target > d) {
      actuator_moving_up = true;
    } else {
      actuator_moving_up = false;
    }
    act_target = d;
  }
}

