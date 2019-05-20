#include <QTRSensors.h>  //Biblioteca dos sensores
#include "BluetoothSerial.h" //Biblioteca do bluetooth

BluetoothSerial ESP_BT; //Nome do ESP para conectar no APP

//Parametros do Bluetooth

int LE_BUILTIN = 2;
bool fim;

//Macro Auxiliar
#define MIN(a,b) (a<b?a:b)
#define MAX(a,b)  (a>b?a:b)
#define RAN(a,b,c) (MAX(MIN(a,c),b))
#define ABS(a) (a<0? -a : a)

//Sensores
#define EDGEVAL 800
#define NUMSENSORS 8 //Quantidade dos sensores
#define MID ((NUMSENSORS - 1)*500)
#define WHITEL 1
#define BLACKL 0
#define TIMEOUT 3000

//Variaveis e  constantes
float KP = 0;
float KI = 0;
float KD = 0;
double ajust = 0;
double sum = 0;
double rd = 0;
double error[10] = {0,0,0,0,0,0,0,0,0,0};
long pos = 0;
byte sensorsInLine;
int vmax = 50;
long division = 0;
bool forward = true;

QTRSensorsRC qtrc((unsigned char[]){12,14,32,33,25,27},NUMSENSORS); ///Pinagem dos sensores

unsigned int sensors[NUMSENSORS]; //Array de leitura

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
  


//Calibração

Serial.print("CALIBRANDO");
for(int i = 0; i < 350;i++){
  qtrc.calibrate();
  delay(20);
}

//Inicia o bluetooth 
ESP_BT.begin ("ESP32_LED_Control"); //Nome do Sinal do bluetooth
Serial.println("Bluetooth Device is Ready to pair");

pinMode(LE_BUILTIN,OUTPUT); //Especifica o LED pin é OUTPUT
Serial.print("Start");

//Definição do KP,KI,KD

KP = lerBT();
Serial.print("O KP é ");
Serial.println(KP);
delay(1000);

KI = lerBT();
Serial.print("O KI é ");
Serial.println(KI);
delay(1000);

KD = lerBT();
Serial.print("O KD é ");
Serial.println(KD);
delay(1000);

vmax = lerBT();
Serial.print("O vmax é ");
Serial.println(vmax);
delay(1000);

}


void loop() {

  posLine(WHITEL);

  AddToVec(pos - MID);

  sum = 0;
  for(int i = 0;i<10;i++){
    sum = sum + error[i]; //Integral
  }
  ajust = KP*error[0] + KD*(error[0] - error[1]) + KI*sum;  //Calculo do ajust

  Serial.println(ajust);
  if(ABS(ajust) > 3500){
    if(ajust > 0){
      MotorControl(vmax*.8,vmax - ajust,true,false);
    }
    else MotorControl(vmax + ajust, vmax*.8,false,true);
  }
  else{
    forward = true;
    if(ajust > 0){
      MotorControl(vmax,vmax - ajust,true,true);
    }
    else MotorControl(vmax + ajust, vmax,true,true);
  }
}


void posLine(byte line){

  qtrc.read(sensors);

  double w,v,sum = 0,division = 0;
  sensorsInLine = 0;

  for(int j = 0; j< NUMSENSORS; j++){  //Função de Regra de três para transformar a leitura em valores de até 1000
    w = RAN(sensors[j],qtrc.calibratedMinimumOn[j],qtrc.calibratedMaximumOn[j]);
    w = w - qtrc.calibratedMinimumOn[j];
    w = w/(qtrc.calibratedMaximumOn[j] - qtrc.calibratedMinimumOn[j]);
    if(line){
      w = 1 - w;
    }
    if(w > 0.6){
      sensorsInLine++;
    }
    sum = sum + 1000*j*w;
    division = division + w;
  }

  if(!sensorsInLine){
    if(pos > MID){
      pos = 2*MID;
    }
    else{
      pos = 0;
    }
  }
  else{
    pos = sum/division;
  }
}

void Break(){    //Função de parar os motores
  digitalWrite(AIN2,true);
  digitalWrite(AIN1,true);
  digitalWrite(BIN2,true);
  digitalWrite(BIN1,true);
}

void MotorControl(int velocidadeA, int velocidadeB, bool sa, bool sb){

  velocidadeA = RAN(ABS(velocidadeA),0,vmax); //Valor intermediario entre Vmax, velocidadeA e 0
  velocidadeB = RAN(ABS(velocidadeB),0,vmax); //Valor intermediario entre Vmax,velocidadeB e 0
  

  if(velocidadeA > 0){    //condição para determinar o sentido de giro do motor A
    digitalWrite(AIN2,sa);
    digitalWrite(AIN1,!sa);
  }
  else{
    digitalWrite(BIN2,sb); //condição para determinar o sentido de giro do motor B
    digitalWrite(BIN1,!sb);
  }

  ledcWrite(MA_PWMCHANNEL,velocidadeA); //Joga no PWM do motorA
  ledcWrite(MB_PWMCHANNEL,velocidadeB); //Joga no PWM do motorB
  
  }

int i = 0;

void AddToVec(double err){
  for(i = 8; i >= 0;i--){
    error[i + 1] = error[i];
  }
  error[0] = err;
} 

float lerBT(){
  String numero = "";
  int dataa = 0;
  while(!ESP_BT.available()){
  }
  while(ESP_BT.available()){
    dataa = ESP_BT.read();
    if(dataa == 13){
      while(ESP_BT.available()){
        ESP_BT.read();
      }
      break;
    }
    else{
      numero += (char)dataa;
    }
  }
  return numero.toFloat();
}
  
  
