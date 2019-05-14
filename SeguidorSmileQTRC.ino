#include <QTRSensors.h>  //Biblioteca dos sensores
#include <BluetoothSerial.h> //Biblioteca do bluetooth

BluetoothSerial ESP_BT; //Nome do ESP para conectar no APP

//Parametros do Bluetooth

int LE_BUILTIN = 2;
bool fim;

//Sensores
#define EDGEVAL 800
#define NUMSENSORS 8 //Quantidade dos sensores

 QTRSensorsRC qtrc((unsigned char[]){12,14,32,33,25,27},NUMSENSORS); ///Pinagem dos sensores

unsigned int sensor_values[NUMSENSORS] //Array de leitura

//ESP 

#define FREQ 5000
#define RESOLUTION 8

//MOTORES 

#define PWMB 21
#define BIN2 19
#define BIN1 18

#define PWMA 2
#define AIN2 4
#define AIN1 5

#define MA_PWMCHANNEL 0
#define MB_PWMCHANNEL 1

//SETUP 

void setup(){

  Serial.begin(115200);

  //MOTOR SETUP 

  pinMode(PWMA,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(PWMA,OUTPUT);
  ledcSetup(MA_PWMCHANNEL,FREQ,RESOLUTION);
  ledcAttachPin(PWMA,MA_PWMCHANNEL);
  ledcSetup(MB_PWMCHANNEL,FREQ,RESOLUTION);
  ledcAttachPin(PWMB,MB_PWMCHANNEL);
  
}

//Calibração

Serial.println("CALIBRANDO");
for(int i = 0; i < 350;i++){
  qtrc.calibrate();
  delay(20);
}








void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
