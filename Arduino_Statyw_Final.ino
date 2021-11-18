#include "I2Cdev.h"
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

Servo servo0;
Servo servo1;
Servo servo2;

SoftwareSerial bluetooth(11,12);  //piny bluetooth TX,RX

float correct;
int j = 0;
int joyX = 0;   //joystick os X
int joyY = 1;   //joystick os Y
int joyButton = 4; //joystick przycisk
uint8_t change_state = 0; //stan zmiany 
int Position;

#define BLUETOOTH_STATE 13  //pin stanu połączenia bluetooth
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  //pin przerwania

bool blinkState = false;
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64];

Quaternion q;          
VectorInt16 aa;         
VectorInt16 aaReal;     
VectorInt16 aaWorld;    
VectorFloat gravity;    
float euler[3];         
float ypr[3];           


uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;    
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(9600);
  bluetooth.begin(9600);
  
  while (!Serial); 

  mpu.initialize();
  
  pinMode(BLUETOOTH_STATE, INPUT);
  pinMode(joyButton, INPUT_PULLUP);
  pinMode(INTERRUPT_PIN, INPUT);
  
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
  }
   servo0.attach(9);  //zdefiniowanie servomechanizmów 
   servo1.attach(10);  
   servo2.attach(8); 
}


void loop() {
  if(digitalRead(BLUETOOTH_STATE) == 0){  //sprawdzenie stanu połączenia bluetooth 
    while(digitalRead(BLUETOOTH_STATE) == 0){ 
      if (digitalRead(joyButton) == LOW) {  //spradzanie stanu przycisku z joysticka 
        delay(1000);
        change_state = 0; //ustanielnie stanu zmiany na 0
        while (change_state == 0) { //przełącznie się między między opcjami sterowania
          joystick_control();
          if (digitalRead(joyButton) == LOW) {
            change_state = 1;
            }
          if(digitalRead(BLUETOOTH_STATE) == 1){
            while(digitalRead(BLUETOOTH_STATE) == 1){
              bluetooth_control();
            }
          }
        }
      }
  if (digitalRead(joyButton) == LOW) {
    delay(1000);
    change_state = 1;
    while (change_state == 1) {
      gyroscope_control();
      if (digitalRead(joyButton) == LOW) {
        change_state = 0;
        }
      if(digitalRead(BLUETOOTH_STATE) == 1){
        while(digitalRead(BLUETOOTH_STATE) == 1){
          bluetooth_control();
        }
      }        
    }
   }
  }
 }
}

void gyroscope_control(){ //funkcja sterowania za pomoca akcelerometru 
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();

  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
    

    if (j <= 300) {
      correct = ypr[0]; 
      j++;
    }
    else {
      ypr[0] = ypr[0] - correct; 
      int servo0Value = map(ypr[0], -90, 90, 0, 180); //mapowanie danych z akcelerometru na serva 
      int servo1Value = map(ypr[1], -90, 90, 180, 0);
      int servo2Value = map(ypr[2], -97, 83, 180, 0);
      
      servo0.write(servo0Value);
      servo1.write(servo1Value);
      servo2.write(servo2Value);
    }
#endif
  }
}

void joystick_control(){  //funkcja steroania za pomocą joysticka 
    
  int valX = analogRead(joyX); //zczytanie danych z joysticka 
  int valY = analogRead(joyY); 

  valX = map(valX, 0, 1023, 180, 0);  //zmapowanie danych z joysticka na servo 
  valY = map(valY, 0, 1023, 180, 0); 
  servo0.write(valX+15);   
  servo1.write(valY);
  servo2.write(75);
  delay(15);  
}

void bluetooth_control(){  //funkcja sterowania za pomocą bluetooth 
  
  if(bluetooth.available()>= 2)  //spradzanie dostępności bluetooth
    {
    unsigned int position_read = bluetooth.read();  //zczytanie danych z Bluetooth
    unsigned int position_read_1 = bluetooth.read();
    unsigned int real_position = (position_read_1 *256) + position_read; //zmiana z binarnego na dziesiętny 
    if (real_position >= 1000 && real_position <1180){  //sprawdzenie, króry suwak jest używany 
      int X_pos = real_position;
      X_pos = map(X_pos, 1000,1180,0,180);  //zmapowanie danych z suwaka na odpowiednie servo
      servo0.write(X_pos);
      delay(15);
      }
 
   if (real_position >=2000 && real_position <2180){
      int Y_pos = real_position;
      Y_pos = map(Y_pos,2000,2180,0,180);
      servo1.write(Y_pos);
      delay(15);
    }
 
    if (real_position >=3000 && real_position < 3180){
      int Z_pos = real_position;
      Z_pos = map(Z_pos, 3000, 3180,0,180);
      servo2.write(Z_pos);
      delay(15);
    }
  }
}
