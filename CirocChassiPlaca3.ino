#include <QTRSensors.h>

//MACRO AUXILIAR
#define MIN(a,b) (a<b?a:b)
#define MAX(a,b) (a>b?a:b)
#define RAN(a,b,c) (MAX(MIN(a,c),b))
#define ABS(a) (a < 0 ? -a : a)
//----------------------------------
//SENSORES
#define EDGEVAL 800
#define NUMSENSORS 4
#define BLACKL 0
#define WHITEL 1
#define MID ((NUMSENSORS - 1)*500)
#define TIMEOUT 3000

unsigned char edgesensor[2] = {22, 23};

QTRSensorsAnalog qtra((unsigned char[]) {14,27,26,25}, 
  NUMSENSORS, 4, 22);

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

//SETUP---------------------------
void setup() {

  Serial.begin(115200);

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
  for (int i = 0; i < 600; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  Serial.println("CALIBRADO");
  //--------------------------
}

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           bool fim = false;
//INICIA OS MOTORES
void followLine(int velocidadeA, int velocidadeB, bool sa, bool sb){
  if(fim) return;
  velocidadeA = RAN(velocidadeA, 0, 255);
  velocidadeB = RAN(velocidadeB, 0, 255);
  ledcWrite(MA_PWMCHANNEL, velocidadeA);
  digitalWrite(AIN2, sa);
  digitalWrite(AIN1, !sa);
  ledcWrite(MB_PWMCHANNEL, velocidadeB);
  digitalWrite(BIN2, sb);
  digitalWrite(BIN1, !sb);
}

//VARIAVES E CONSTANTES SEGUIDOR DE LINHA
#define DEBUG 1
#define VTURN 30
#define KP 0.95
#define KD 20
#define KI -0.002
#define KRD 0
double ajust = 0, sum, rd;
long pos = 0;
double error[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte sensorsInLine;
char borda = 0;
int lap = 0;

int vmax = 180, vmin = 100;
//---------------------//-----------LOOP---------------
void loop() {

  posLine(WHITEL);

  addToVec(pos - MID);

  Serial.println(error[0]);
  
  Serial.println(pos - MID);

  if(false){
    if (ajust > 0)followLine(VTURN, VTURN, false, true);
    else followLine(VTURN, VTURN, true, false);
    
  }else{
    
    sum = 0;
    for (int i = 0; i < 10; i++) sum += error[i];
    ajust = KP*error[0] + KD*(error[0] - error[1]) + KI*sum;
    rd = KRD * ABS(error[0]);

    if (ajust > 0)followLine(MAX(vmax - rd, vmin), vmax - ajust - rd);
    else followLine(vmax + ajust - rd, MAX(vmax - rd, vmin));
  }
   
}

void addToVec(double err){
  for(int i = 8; i >= 0; i--)error[i + 1] = error[i];
  error[0] = err;
}

void posLine(byte line){

  qtra.read(sensors);
  
  double w, v, sum = 0, division = 0;
  sensorsInLine = 0;
  
  for(int j = 0; j < NUMSENSORS; j++){
    w = RAN(sensors[j], qtra.calibratedMinimumOn[j], qtra.calibratedMaximumOn[j]);
    w -= qtra.calibratedMinimumOn[j];
    w /= (qtra.calibratedMaximumOn[j] - qtra.calibratedMinimumOn[j]);
    if (line) w = 1 - w;
    if (w > 0.6) sensorsInLine ++;
    sum += 1000*j*w;
    division += w;
  }

  if (!sensorsInLine){
    if(pos > MID){
      pos = 2*MID;
    }else{
      pos = 0;
    }
  }else{
    pos = sum/division;
  }
  
}
