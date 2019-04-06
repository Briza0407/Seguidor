#include <QTRSensors.h>
#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino

BluetoothSerial ESP_BT; //Object for Bluetooth

// Parametros do Bluetooth
int LE_BUILTIN = 2;
bool fim;

//MACRO AUXILIAR
#define MIN(a,b) (a<b?a:b)
#define MAX(a,b) (a>b?a:b)
#define RAN(a,b,c) (MAX(MIN(a,c),b))
#define ABS(a) (a < 0 ? -a : a)
//----------------------------------
//SENSORES
#define EDGEVAL 800
#define NUMSENSORS 8
#define BLACKL 0
#define WHITEL 1
#define MID ((NUMSENSORS - 1)*500)
#define TIMEOUT 3000

unsigned char edgesensor[2] = {22, 23};

QTRSensorsAnalog qtra((unsigned char[]) {13,12,14,27,26,25,33,32},
NUMSENSORS, 4 , 22);

QTRSensorsRC qtrc((unsigned char[]) {23}, 1);

unsigned int sensors[NUMSENSORS], sensors_e[2];
unsigned int e_min[2];
unsigned int e_max[2];
//-----------------------------------
//MOTORES
#define PWMB 21
#define BIN2 19
#define BIN1 18

#define PWMA 15
#define AIN2 2
#define AIN1 4

#define MA_PWMCHANNEL 0
#define MB_PWMCHANNEL 1

#define FREQ 5000
#define RESOLUTION  8

void followLine(int velocidadeA, int velocidadeB, bool sa = HIGH, bool sb = HIGH);
void posLine(byte line);
void addToVec( double err);
unsigned long calibrationTime = 0;

////Variables/////
float KI  = 0;
float KP = 0;
float KD = 0;
double ajust = 0, sum = 0,rd;
long pos = 0;
double error[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte sensorsInLine;
//7.9V
bool forward = true;
int vmax = 0;
int vmin = 0;

//SETUP---------------------------
void setup() {

  Serial.begin(9600);

  pinMode (LE_BUILTIN, OUTPUT);//Specify that LED pin is output
  Serial.print("start");

  //MOTOR SETUP
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  ledcSetup(MA_PWMCHANNEL, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, MA_PWMCHANNEL);
  ledcSetup(MB_PWMCHANNEL, FREQ, RESOLUTION);
  ledcAttachPin(PWMB, MB_PWMCHANNEL);
  //-------------------------
  
  //CALIBRAÇÃO SENSORES
  Serial.println("CALIBRANDO");
    for (int i = 0; i < 800; i++) {
      qtra.calibrate();
    }
  Serial.println("CALIBRADO");
  calibrationTime = millis();
  //--------------------------


  //INICIA o Bluetooth
  ESP_BT.begin("ESP32_LED_Control"); //Name of your Bluetooth Signal
  Serial.println("Bluetooth Device is Ready to Pair");

  pinMode (LE_BUILTIN, OUTPUT);//Specify that LED pin is output
  Serial.print("start");



  //VARIAVES E CONSTANTES SEGUIDOR DE LINHA

  KP = lerBT();
  Serial.print("o KP é ");
  Serial.println(KP);
  delay(1000);

  KI = lerBT();
  Serial.print("o KI é ");
  Serial.println(KI);
  delay(1000);

  KD = lerBT();
  Serial.print("o KD é ");
  Serial.println(KD);
  delay(1000);

  vmax = lerBT();
  Serial.print("o vmáx é ");
  Serial.println(vmax);
}
//---------------------



//-----------LOOP---------------
void loop() {

  //delay(20);

  Serial.println("Saiu");

  if ((millis() - calibrationTime) > 28000) {
    followLine(0, 0);
    fim = true;
  }

  posLine(WHITEL);

  addToVec(pos - MID);

  sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += error[i];
  }
  ajust = KP * error[0] + KD * (error[0] - error[1]) + KI * sum;

  if (ABS(ajust) > 3500) {
    if (ajust > 0)followLine(vmax * .8, vmax - ajust, true, false);
    else followLine(vmax + ajust, vmax * .8, false, true);

  } else {
    forward = true;
    if (ajust > 0)followLine(vmax, vmax - ajust);
    else followLine(vmax + ajust, vmax);

  }
}
void followLine(int velocidadeA, int velocidadeB, bool sa, bool sb) {
  if (fim) return;

  velocidadeA = RAN(ABS(velocidadeA), 0, vmax);

  velocidadeB = RAN(ABS(velocidadeB), 0, vmax);


  if (velocidadeA > 0) {
    digitalWrite(AIN2, sa);
    digitalWrite(AIN1, !sa);
  } else {
    digitalWrite(AIN2, !sa);
    digitalWrite(AIN1, sa);
  }

  if (velocidadeB > 0) {
    digitalWrite(BIN2, sb);
    digitalWrite(BIN1, !sb);
  } else {
    digitalWrite(BIN2, !sb);
    digitalWrite(BIN1, sb);
  }


  ledcWrite(MA_PWMCHANNEL, velocidadeA);
  ledcWrite(MB_PWMCHANNEL, velocidadeB);
}

void Break() {
  digitalWrite(AIN2, true);
  digitalWrite(AIN1, true);
  digitalWrite(BIN2, true);
  digitalWrite(BIN1, true);

}

void addToVec(double err) {
  for (int i = 8; i >= 0; i--)error[i + 1] = error[i];
  error[0] = err;
}

void posLine(byte line) {
  // midRead();
    qtra.read(sensors);

  double w, v, sum = 0, division = 0;
  sensorsInLine = 0;

  //for(int i = 0; i < NUMSENSORS; i++){
  //Serial.print(sensors[i]);
  //Serial.print(" / ");
  //}
  //Serial.println("x");
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    for (int j = 0; j < NUMSENSORS; j++) {
      w = RAN(sensors[j], qtra.calibratedMinimumOn[j], qtra.calibratedMaximumOn[j]);
      w -= qtra.calibratedMinimumOn[j];
      w /= (qtra.calibratedMaximumOn[j] - qtra.calibratedMinimumOn[j]);
      if (line) w = 1 - w;
      if (w > 0.6) sensorsInLine ++;
      sum += 1000 * j * w;
      division += w;
    }
  
    if (!sensorsInLine) {
      if (pos > MID) {
        pos = 2 * MID;
      } else {
        pos = 0;
      }
    } else {
      pos = sum / division;
    }

}

float lerBT() {
  String numero = "";
  int dataa = 0;
  while (!ESP_BT.available()) {
    ;
  }
  while (ESP_BT.available()) { //Check if we receive anything from Bluetooth
    dataa = ESP_BT.read(); //Read what we recevive
    if (dataa == 13) {
      while (ESP_BT.available()) {
        ESP_BT.read();
      }
      break;
    }
    else {
      numero += (char)dataa;
    }
  }
  return numero.toFloat();
}
