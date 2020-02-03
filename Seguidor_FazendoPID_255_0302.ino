// Este código tem como objetivo ser a continuação do ReadLine manual
// utilizando a ideia da linha ser branca e o fundo ser preto.                    
// 

// PWM         50     -----       60     -----       70     -----       80    -----    90       -----     110  
// Kp:      0.043     -----     0.04     -----     0.043    -----     0.19    -----    0.065    -----     0.08  
// Ki:          0     -----        0     -----     0.00001  -----             -----    0.3      -----     0.3   
// Kd:       0.20     -----      0.3     -----      0.24    -----      0.9    -----    0.0001   -----    0.0001 
// Tensão:      -     -----     7.97     -----     7.67     -----     7.32    -----    7.6      -----     7.7   

// PWM            130      ----   150      ----   170      ----   190       ----   210       ----  230        ----   255
// Kp:            0.13     ----   0.14     ----   0.21     ----   0.18      ----   0.18      ----  0.2        ----   0.16
// Ki:            0.43     ----   0.35     ----   0.42     ----   0.51      ----   0.51      ----  0.52       ----   0.49
// Kd:            0.000015 ----   0.00002  ----   0.00003  ----   0.000024  ----   0.000026  ----  0.000027   ----   0.000025
// Tensão:        7.67     ----   7.79     ----   7.83     ----   7.61      ----   7.61      ----  7.59       ----   7.55


#include <QTRSensors.h>

#define SaidaA 9
#define SaidaB 3

//Lado direito
#define AIN2 6 // Quando em HIGH, roda direita anda para frente
#define AIN1 5 

//Lado Esquerdo
#define BIN1 4 // Quando em HIGH, roda esquerda anda para frente
#define BIN2 2

double Kp = 0.16, Kd = 0.49, Ki =0.000025; // Variáveis que são modificadas no PID
int PWM = 255; // valor da força do motor em linha reta

double erro, p, d, erroAnterior = 0, i, integral = 0, Turn = 0; //Área PID
double MotorA, MotorB; // Tensao enviada pelo PID

int position_line = 0; // Método com o Read Line;

int contador = 0, acionador = 0; // Borda

QTRSensorsRC borda ((unsigned char[]) {A3}, 1);                         //Sensor de borda
QTRSensorsAnalog qtra  ((unsigned char[]) {A2, A1, A0, A7, A6, A4}, 6); //Sensores frontais

unsigned int sensor_values[6];
unsigned int BordaValor[1];

void setup() {
  Serial.begin(9600); // Comunicacao com o Serial
  
  pinMode(A0,      INPUT);
  pinMode(A1,      INPUT);
  pinMode(A2,      INPUT);
  pinMode(A3,      INPUT);
  pinMode(A4,      INPUT);
  pinMode(A6,      INPUT);
  pinMode(A7,      INPUT);
  pinMode(AIN1,   OUTPUT);
  pinMode(AIN2,   OUTPUT);
  pinMode(BIN1,   OUTPUT);
  pinMode(BIN2,   OUTPUT);
  pinMode(MotorA, OUTPUT);
  pinMode(MotorB, OUTPUT);

  //----> Calibração do sensor de borda <----\\

  for (int i = 0; i < 70; i++)
    {
      borda.calibrate(); 
      delay(5);
    }

  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  
  //----> Calibração dos Sensores frontais <----\\

  for (int i = 0; i < 120; i++)
    {
      qtra.calibrate();
      delay(5);
    }

  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(500);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
 
}


void loop() {
  
  borda.readCalibrated(BordaValor);

  position_line = qtra.readLine(sensor_values,QTR_EMITTERS_ON, 1);
  erro = position_line - 2500;

  if ((BordaValor[0] < 550) && (acionador == 0))
    {
      contador++;
      acionador = 1;
    }

  if ((BordaValor[0] > 550) && (acionador == 1))
    {
      acionador = 0;
    }

//    Serial.print("Contador: ");
//    Serial.print(contador);
//    Serial.print("| ");
//    Serial.print("Valor da borda: ");
//    Serial.println(BordaValor[0]);

  while (contador == 2) 
    {
      analogWrite(SaidaA, LOW);
      analogWrite(SaidaB, LOW);
    }

  //--------------->AREA DO PID<---------------\\ 

  p = erro * Kp; // Proporcao
  
  integral += erro; // Integral
  i = Ki * integral;
 
  d = Kd * (erro - erroAnterior); // Derivada
  erroAnterior = erro;
  
  Turn = p + i + d;
  
  MotorA = PWM - Turn;
  MotorB = PWM + Turn;

  //--------------->AREA DO SENTIDO DAS RODAS<---------------\\ 

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, LOW);
  digitalWrite(BIN1, HIGH);

  if (MotorA < 0) // Giro para a direita
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
  if (MotorB < 0) // Giro para a esquerda
    {
      digitalWrite(BIN2, HIGH);
      digitalWrite(BIN1, LOW);
    }
    
  analogWrite(SaidaA, MotorA);
  analogWrite(SaidaB, MotorB); 
  
}
