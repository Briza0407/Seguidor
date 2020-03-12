#include <QTRSensors.h>

#define PWMA 9
#define PWMB 3

#define AIN1 5   //AIN1 HIGH - Direita Trás
#define AIN2 6   //AIN2 HIGH - Direita Frente 
                                                   
#define BIN1 4  //BIN1 HIGH - Esquerda Frente
#define BIN2 2  //BIN2 HIGH - Esquerda Trás             

/*Combinações: Sentido virar para direita =  AIN1(HIGH),AIN2(LOW),BIN1(HIGH),BIN2(LOW)
               Sentido virar para esquerda = AIN1(LOW),AIN2(HIGH),BIN1(LOW),BIN2(HIGH) 
*/

double Kp = 0.27,Kd = 1.18, Ki = 0; //Parâmetros do Controle PID - Kp(Proporcional),Kd(Derivada),Ki(Integral)
int PWM = 160; //Constante de velocidade caso o robô esteja reto na linha(Sem Erro)

double erro,p,d,erroAnterior,i,integral,Turn = 0; 

double MotorA,MotorB //Velocidades que serão mandadas para as portas dos motores
double leitura1,leitura2; //Leituras de cada metade dos sensores(Leitura 1 - Lado Esquerdo, Leitura 2 - Lado Direito)

int ExcessoA,ExcessoB; //Parâmetros de Excesso nos motores


int contador; acionador; //Parâmetros do número de bordas e acionadores


QTRSensorsRC borda((unsigned char[]) {A3},1} //Sensor de Borda
QTRSensorsAnalog qtra ((unsigned char[]) {A2,A1,A0,A7,A6,A4},6); //Sensores Frontais


unsigned int sensor_values[6];
unsigned int BordaValor[1]

void setup() { 

  Serial.begin(9600);
  //Pinagem dos Sensores
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);

  //Pinagem do sentido dos Motores
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  //Pinagem dos PWMs de cada motor
  pinMode(MotorA, OUTPUT);
  pinMode(MotorB, OUTPUT);

  Calibração();

}

void Calibração(){

  Serial.println("Iniciando Calibração do Sensor de Borda");

  for(int i = 0; i < 100; i++){
    borda.calibrate();
    delay(5);
}

  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);

  Serial.println("Iniciando Calibração do Sensor Frontal");

  for(int i = 0; i < 150; i++){
    qtra.calibrate();
    delay(5);
  }

  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
  delay(500);
  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
}

void loop() {

  Error();

}

void Error(){

  qtra.readCalibrated(sensor_values);

  //Media Ponderada do sensor frontal, Sensores Externos tem PESOS maiores no calculo do Erro, no fim dividimos o resultado pela soma dos Pesos 
  LeftRead = ((3000*sensor_values[0]) + (2000*sensor_values[1]) + (1000*sensor_values[2]))/6000;   //Soma o valor dos 3 sensores do lado esquerdo do sensor multiplicado pelos seus respectivos pesos
  RightRead =((3000*sensor_values[5])+ (2000*sensor_values[4]) + (1000*sensor_values[3]))/6000;   //Soma o valor dos 3 sensores do lado direito do sensor multiplicado pelos seus respectivos pesos 

  Error = LeftRead - RightRead; //Aplica a lógica de cálculo do Erro por simetria, sem importar a questão de Referencia, ou seja, a lógica é (Leitura do Sensor Esquerdo) - (Leitura do Sensor Direito) = (ERROR)
 
