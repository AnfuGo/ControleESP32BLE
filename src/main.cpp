#include <Arduino.h>

const int Sensor_PINS[3] = {5, 18, 19}; // definir pinos para os sensores (ADC)
const int PWM_PINS[3] = {1, 2, 3}; //  Potência de saída para as resistências - Definir pinos

const int pwmChannel[3] = {0, 1 ,2};
const int pwmFreq = 1000;      // Frequência PWM a escolher...
const int pwmResolution = 8;   // 8 bits → 0-255
const float Vref = 3.3; // tensão de referencia a mudar....
const int ADC_MAX = 4095;

unsigned long ultimoTempo[3] = {0, 0, 0}; // contador 
const unsigned long intervalo = 500; // intervalo de aquisição do controle

// Parâmetros FOPDT do sistema a definir
float K[3] = {1, 1, 1}; //ganho
float tau[3] = {3, 3, 3}; //integral
float theta[3] = {4, 4, 4}; //derivativo
float medida;

float setpoint[3] = {25, 25, 25} ; // inicial
bool controleAtivo[3] = {false, false, false};

// Variáveis PID
float Kp[3], Ti[3], Td[3];
float DeltaSaida[3];
float saidaAnterior[3];
float saida[3]; // porcentagem
int output[3]; // saida em 0-255 para o pino-ADC
float errk2[3] = {0, 0, 0};
float errk1[3] = {0, 0, 0};
float errk[3] = {0, 0, 0};


float temperatura[3] = {25, 25, 25};
float a0[3], a1[3], a2[3];
const unsigned long Ts[3] = {500, 500, 500};

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  for (int i = 0; i < 3; i++){
    ledcSetup(pwmChannel[i], pwmFreq, pwmResolution);
    ledcAttachPin(PWM_PINS[i], pwmChannel[i]);
    ledcWrite(pwmChannel[i], 0);
    } 
     

  // Cálculo parâmetros PID, forma incremental...
  for (int i = 0; i < 3; i++){
    Kp[i] = 1.2 * (tau[i] / (K[i] * theta[i]));
    Ti[i] = 2.0 * theta[i];
    Td[i] = 0.5 * theta[i];
    a0[i] = Kp[i] * (1 + Ts[i]/(2*Ti[i]) + Td[i] / Ts[i]);
    a1[i] = -Kp[i] * (1 - Ts[i] / (2*Ti[i]) + 2 * Td[i]/Ts[i]);
    a2[i] = Kp[i] * Td[i] / Ts[i];
  }
  

}

void loop() {
  unsigned long agora = millis();
  for (int i = 0; i < 3; i++){
    if (controleAtivo[i] && (agora - ultimoTempo[i] >= intervalo)){
      ultimoTempo[i] = agora;
      medida = analogRead(Sensor_PINS[i]); // necessário tranformar a medida de temperatura do sensor
      temperatura[i] = medida / ADC_MAX; // mudar a depender do sensor
      errk[i] = setpoint[i] - temperatura[i]; 
      DeltaSaida[i] = a0[i]*errk[i] + a1[i]*errk1[i] + a2[i]*errk2[i];
      saida[i] = DeltaSaida[i] + saidaAnterior[i]; //porcentagem do DC
      saidaAnterior[i] = saida[i];
      saida[i] = saida[i] * 255 / 100; 
      output[i] = constrain(saida[i], 0, 255); // limita a saida 0-255
      ledcWrite(pwmChannel[i], (int)output[i]);
      errk2[i] = errk1[i];
      errk1[i] = errk[i];

    }else if (!controleAtivo[i]){
      ledcWrite(pwmChannel[i], 0); // desativa a saida completa
    }
  }
}
