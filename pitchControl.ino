//Para comunicação I2C com o MPU6050
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "Wire.h"

//Servo Motor
#include <ServoTimer2.h>                                  //Para uso do atuador servo motor
//--------------------------------------------------------



//--------------------------------------------------------
//                      CONTROLE DA MPU6050
//--------------------------------------------------------
MPU6050 mpu; //MPU6050 mpu( I2C_ADDR = 0x68 );
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }
//--------------------------------------------------------





//--------------------------------------------------------
//                   CONTROLE DO SERVO
//--------------------------------------------------------
#define SERVO_GPIO 3                                      //Porta em uso pelo servo motor
ServoTimer2 servo1;
const unsigned int DEFAULT_PW = 1500;
const unsigned int MIN_PW     = 1250;
const unsigned int MAX_PW     = 1750;
volatile unsigned int servoAngleOut = DEFAULT_PW;
//--------------------------------------------------------





//--------------------------------------------------------
//                   PARÂMETROS DE CONTROLE
//--------------------------------------------------------
const double Kp = -1.204486;                              //Controlador Proporcional
const double Ts = 60;                                     //Tempo de Amostragem em ms
volatile double REF  = 0;                                 //Ângulo de referência em radianos
double erro[3]    = {0,0,0};                              //Vetor de Erro do modelo de controle discreto
double delta_e[3] = {0,0,0};                              //Vetor de Saída do modelo de controle discreto

float pitchAngleDegVector[4] = {0,0,0,0};                 //Vetor das últimas 4 medidas de pitch
unsigned int pitchIndex = 0;                              //Contador da posição atual do vetor a ser atualizada
volatile float pitchRadFiltered;                          //Valor de pitch em rad filtrado da parte inteira da média das últimas 4 medidas em graus
volatile float pitchAngleRad;                             //Valor de pitch lido em radianos
volatile float pitchAngleDeg;                             //Valor de pitch lido em radianos
#define GPIO_LED 10
//--------------------------------------------------------

void myTimerHandler(void);
void refreshPitch(void);
void updateControl(void);


void setup() {
  //SERIAL SETUP
  Serial.begin(115200);
  
  //PID STATUS LED SETUP
  pinMode( GPIO_LED, OUTPUT );
  digitalWrite(GPIO_LED,LOW);
  
  //SERVO SETUP
  servo1.attach( SERVO_GPIO );                            //Configura a porta em que o Servo está conectado
  //servo1.write( MIN_PW );delay(1000);
  //servo1.write( MAX_PW );delay(1000);
  servo1.write( DEFAULT_PW );delay(1000);

  //MPU6050 SETUP
  Wire.begin();TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize(); // load and configure the DMP
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();// get expected DMP packet size for later comparison
  mpu.resetFIFO();

  //WAITING MPU6050 TO WORK PROPERLY
  Serial.println("Wait 20 seconds!");
  //FIXING MPU6050 BAD WORKING AT START
  unsigned long counter = 0;
  unsigned long meanMic = 0;
  unsigned long prevMil1 = millis(); //20s wait
  unsigned long prevMil2 = millis(); //Ts estimation
  while( (millis() - prevMil1) < 20000 ){
    prevMil2 = millis();           // Count time
    refreshPitch();               // Do the job
    meanMic += millis() - prevMil2;// Average Ts

    //GET MEAN Ts
    counter++;
    if( counter == 40 ){
      Serial.println();
      Serial.print("Got: ");
      Serial.println( meanMic/40.0 );
      meanMic = 0;
      counter = 0;
      mpu.resetFIFO();
    }
  }
  digitalWrite(GPIO_LED, HIGH);
}

void loop(){
  updateControl();
}

void myTimerHandler(void){
  //A cada tempo de amostragem Ts
  
  //Kp
  erro[2]    = REF - pitchRadFiltered;                                //Error = Reference - Measure
  delta_e[2] = Kp*erro[2];                                            //Control Out [radians]
  servoAngleOut = 1500*(1 + delta_e[2]/PI );                          //Servo Out [Pulse Width]
  if(      servoAngleOut < MIN_PW ){ servoAngleOut = MIN_PW; }        //Pulse Width Saturator
  else if( servoAngleOut > MAX_PW ){ servoAngleOut = MAX_PW; }        //Pulse Width Saturator
  servo1.write( servoAngleOut );                                      //Uptadating Servo Motor Pulse Width
}

void refreshPitch(){
  // wait for MPU interrupt or extra packet(s) available
    //if( !(!mpuInterrupt && fifoCount < packetSize) )
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        pitchAngleRad = ypr[2];
        pitchAngleDeg = pitchAngleRad * 180/PI + pitchOffset;

        //UPDATE LAST 4 DATA
        pitchAngleDegVector[ pitchIndex++ ] = pitchAngleDeg;
        if( pitchIndex == 4 ){ pitchIndex = 0; }
        //GET FILTERED MEAN ANGLE IN DEG AND RAD
        pitchAngleDeg = ( pitchAngleDegVector[0] + pitchAngleDegVector[1] + pitchAngleDegVector[2] + pitchAngleDegVector[3] )/4.0;
        //Serial.print( pitchAngleDeg );Serial.print(" ");
        if( (pitchAngleDeg - (int)(pitchAngleDeg)) < 0.5 ){ pitchAngleDeg = (int)(pitchAngleDeg);     }
        else{                                               pitchAngleDeg = (int)(pitchAngleDeg) + 1; }
        pitchRadFiltered = (pitchAngleDeg)*PI/180;
        //Serial.print( (int)(pitchAngleDeg) );
    }
}

void updateControl(void){
  for( int j=0; j<4; j++ ){ refreshPitch(); }
  Serial.print( (int)(pitchRadFiltered*180/pi) );Serial.print(" ");
  myTimerHandler();
}

