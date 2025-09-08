#include <Arduino.h>

#define LM35_PIN 4
#define PWM_PIN 5

const int pwmChannel = 0;
const int pwmFreq = 1000;      // Frequência PWM 1kHz
const int pwmResolution = 8;   // 8 bits → 0-255
const float Vref = 3.3;
const int ADC_MAX = 4095;

unsigned long ultimoTempo = 0;
const unsigned long intervalo = 500; // intervalo controle em ms

// Parâmetros FOPDT do sistema
float K = 40.0;
float tau = 60.0;
float theta = 10.0;

float setpoint = 0.0;
unsigned long tempoSubida = 0;      // em segundos
unsigned long tempoManutencao = 0;  // em segundos
bool controleAtivo = false;
unsigned long tempoInicio = 0;

// Variáveis PID
float erroAnterior = 0.0;
float integral = 0.0;

float Kp, Ki, Kd;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(PWM_PIN, pwmChannel);
  ledcWrite(pwmChannel, 0);

  // Cálculo parâmetros PID (Ziegler-Nichols para FOPDT)
  Kp = 1.2 * (tau / (K * theta));
  float Ti = 2 * theta;
  float Td = 0.5 * theta;
  Ki = Kp / Ti;
  Kd = Kp * Td;

  Serial.println("Digite o setpoint (°C):");
}

void loop() {
  // Lê setpoint e tempos via serial (se controle não ativo)
  if (Serial.available()) {
    String entrada = Serial.readStringUntil('\n');
    entrada.trim();
    float val = entrada.toFloat();

    if (!controleAtivo && val > 0) {
      setpoint = val;
      Serial.println("Setpoint recebido: " + String(setpoint));

      Serial.println("Quanto tempo para atingir o setpoint? (min):");
      while (!Serial.available()) delay(10);
      tempoSubida = Serial.readStringUntil('\n').toInt() * 60;  // minutos -> segundos

      Serial.println("Quanto tempo manter o setpoint? (min):");
      while (!Serial.available()) delay(10);
      tempoManutencao = Serial.readStringUntil('\n').toInt() * 60;  // minutos -> segundos

      controleAtivo = true;
      tempoInicio = millis();
      integral = 0.0;
      erroAnterior = 0.0;
    }
  }

  unsigned long agora = millis();
  if (controleAtivo && (agora - ultimoTempo >= intervalo)) {
    ultimoTempo = agora;

    int adcValue = analogRead(LM35_PIN);
    float temperatura = (adcValue * Vref / ADC_MAX) * 100.0;  // LM35: 10mV/°C e Vref=3.3V

    // Cálculo PID
    float erro = setpoint - temperatura;
    integral += erro * (intervalo / 1000.0);                // intervalo em segundos
    float derivada = (erro - erroAnterior) / (intervalo / 1000.0);
    erroAnterior = erro;

    float output = Kp * erro + Ki * integral + Kd * derivada;
    output = constrain(output, 0, 255);

    ledcWrite(pwmChannel, (int)output);

    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.print(" °C | PWM: ");
    Serial.println((int)output);

    unsigned long tempoDecorrido = (agora - tempoInicio) / 1000;  // em segundos
    if (tempoDecorrido >= (tempoSubida + tempoManutencao)) {
      ledcWrite(pwmChannel, 0);
      Serial.println("Tempo encerrado. Controle finalizado.");
      Serial.println("Digite um novo setpoint:");
      controleAtivo = false;
    }
  }
}
