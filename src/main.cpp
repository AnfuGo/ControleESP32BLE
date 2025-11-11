#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include <BLE2902.h> // desnecessário
#include <OneWire.h> // sensor
#include <DallasTemperature.h> //sensor
#include <cctype> // isdigit

// UUID-bluetooth-----128bits gerados através do gerador online de UUID.
const char* CIA_CONTROL_ENABLE = "22086d8b-57c2-4eb4-b82d-4b7936413e78";
const char* CONTROLE_TEMP      = "2f7d55d9-acce-4a1d-9871-dd3e4ec73eec";
const char* ENABLE_1           = "1b4a81b4-abf5-450d-9d4d-81b4e951baa7";
const char* ENABLE_2           = "d2b757c1-9fb1-4166-af83-467a0a0a55f3";
const char* ENABLE_3           = "765170c6-24e5-4890-a346-10195b57ca7c";
const char* AlTemp_1           = "28b66ce2-882e-46e2-bc17-1332f33072db";
const char* AlTemp_2           = "83ec522b-14b0-4b42-8a8b-0924993f5490";
const char* AlTemp_3           = "e49b8658-6f76-43dd-a36e-db7f4b1aa546";

#define PID_INCREMENTAL 0
#define PID_IDEAL       1
#define PID_MODE PID_IDEAL
#define lamda 0

const char* SERVICE_UUID[2] = { CIA_CONTROL_ENABLE, CONTROLE_TEMP };
const char* CHARACTERISTIC_UUID[6] = { ENABLE_1, ENABLE_2, ENABLE_3, AlTemp_1, AlTemp_2, AlTemp_3 };

const int Sensor_PINS[3] = { 27, 26, 25 };  
const int PWM_PINS[3]    = { 18, 19, 21 };  // potência de saída para resistências, definir pinos GPIO

const int pwmChannel[3]  = { 0, 1, 2 };
const int pwmFreq        = 1000;     // frequência PWM
const int pwmResolution  = 8;        // 8 bits → 0-255
const float Vref         = 3.3;      // tensão de referência
//const int ADC_MAX      = 4095;

// Rampa de setpoint
const int rampSteps = 5;
float setpointQueue[100];   // fila com até 100 steps futuros
int queueHead = 0, queueTail = 0;
unsigned long lastRampUpdate[3] = { 0, 0, 0 };
const unsigned long rampInterval = 1000;  // tempo (ms) entre steps da rampa
unsigned long ultimoTempo[3] = { 0, 0, 0 }; 
const unsigned long intervalo = 500; // intervalo de aquisição

// Parâmetros FOPDT do sistema
float K[3]     = { 1.453, 1.599, 1.967 };
float tau[3]   = { 286.2705, 268.2375, 370.4385 };
float theta[3] = { 25.8135, 22.1845, 13.7885 };
float medida;

int setpoint[3]       = { 25, 25, 25 };
bool controleAtivo[3] = { false, false, false };

// Variáveis PID
float Td[3] = { 0, 0, 0 };
float Kp[3] = { 0.135242033, 0.1230080565, 0.1009334582 };
float Ti[3] = { 286.27, 268.24, 370.44 };

float DeltaSaida[3];
float saidaAnterior[3];
float saida[3];     // porcentagem
float bias[3] = { 2, 2, 2 };
int output[3];      // saída em 0-255
float errk2[3] = { 0, 0, 0 };
float errk1[3] = { 0, 0, 0 };
float errk[3]  = { 0, 0, 0 };
float integral[3] = { 0, 0, 0};
float erroAnterior[3] = { 0, 0, 0};
float temperatura[3] = { 25, 25, 25 };
float a0[3], a1[3], a2[3];
const float Ts[3] = { 0.5, 0.5, 0.5 };

// Cria uma instância do barramento OneWire
OneWire oneWire0(Sensor_PINS[0]);
OneWire oneWire1(Sensor_PINS[1]);
OneWire oneWire2(Sensor_PINS[2]);

DallasTemperature sensors[3] = {
  DallasTemperature(&oneWire0),
  DallasTemperature(&oneWire1),
  DallasTemperature(&oneWire2)
};

// ---------------------- Classes BLE ----------------------

class enableServer : public BLEServerCallbacks {
  void onConnect(BLEServer* servidorCIA) override {
    Serial.println("Conectado!");
  }

  void onDisconnect(BLEServer* servidorCIA) override {
    Serial.println("Desconectado! Desligando saídas para segurança.");

    // Desativar controle de todos os canais
    for (int i = 0; i < 3; i++) {
      controleAtivo[i] = false;
      ledcWrite(pwmChannel[i], 0); // Zera PWM
      saida[i] = 0;
      saidaAnterior[i] = 0;
      errk[i] = errk1[i] = errk2[i] = 0; // Zera erros (evita windup residual)
      queueHead = queueTail = 0;  // limpa rampa
      setpoint[i] = 25; // trava setpoint no valor da temp Ambiente
    }

    // começa a anunciar o serviço novamente
    BLEDevice::startAdvertising();
  }
};

class enableControl : public BLECharacteristicCallbacks {
  bool &controle;

public:
  enableControl(bool &control) : controle(control) {}

  void onWrite(BLECharacteristic* ENABLE_CONTROL) override {
    const std::string value = ENABLE_CONTROL->getValue();

    if (!value.empty()) {
      uint8_t b = static_cast<uint8_t>(value[0]);

      if ((b == '0') || (b == '1')) {
        controle = (b == '1');
      } else {
        controle = (b != 0);
      }

      Serial.print("enableControl onWrite - raw first byte: ");
      Serial.println(b);
      Serial.print("controle set to: ");
      Serial.println(controle ? "1" : "0");
    } else {
      Serial.println("enableControl onWrite - valor vazio recebido");
    }
  }
};

class functionsControl : public BLECharacteristicCallbacks {
  int &set_point;
  int sensorIndex;

public:
  functionsControl(int &st, int idx) : set_point(st), sensorIndex(idx) {}

  void onWrite(BLECharacteristic* variaveisBLE) override {
    const std::string value = variaveisBLE->getValue();

    if (!value.empty()) {
      bool allAsciiDigits = true;
      for (char c : value) {
        if (!(std::isdigit((unsigned char)c) || c == '-' || c == '+')) {
          allAsciiDigits = false;
          break;
        }
      }

      if (allAsciiDigits) {
        int parsed = atoi(value.c_str());
        set_point = parsed;
        Serial.print("functionsControl onWrite - ASCII parsed: ");
        Serial.println(parsed);
      } else {
        uint8_t b = static_cast<uint8_t>(value[0]);
        set_point = b;
        Serial.print("functionsControl onWrite - byte value: ");
        Serial.println(b);
      }

      Serial.print("Novo setpoint: ");
      Serial.println(set_point);
    } else {
      Serial.println("functionsControl onWrite - valor vazio recebido");
    }
  }

  void onRead(BLECharacteristic* variaveisBLE) override {
    std::string out = std::to_string(temperatura[sensorIndex]);
    variaveisBLE->setValue(out);
    Serial.print("functionsControl onRead - enviando temperatura sensor ");
    Serial.print(sensorIndex);
    Serial.print(": ");
    Serial.println(out.c_str());
  }
};

// ---------------------- Setup ----------------------

void setup() {
  Serial.begin(115200);
  BLEDevice::init("ESP32S_CIA");
  BLEServer *servidorCIA = BLEDevice::createServer();
  servidorCIA->setCallbacks(new enableServer());

  for (int i = 0; i < 3; i++) {
    sensors[i].begin();
    sensors[i].setResolution(12);
    // sensors[i].setWaitForConversion(false); 
    // ↑ Deixa o requestTemperatures() assíncrono (não bloqueia).
  }

  Serial.println("Servidor Criado");

  BLEService *controlTemp[2];
  for (int i = 0; i < 2; i++) {
    controlTemp[i] = servidorCIA->createService(SERVICE_UUID[i]);
  }

  BLECharacteristic *variaveisBLE[6];
  int k = 0, l = 0;

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      variaveisBLE[k] = controlTemp[i]->createCharacteristic(
        CHARACTERISTIC_UUID[l],
        BLECharacteristic::PROPERTY_READ   |
        BLECharacteristic::PROPERTY_WRITE  |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
      );
      k++;
      l++;
    }
  }

  Serial.println("Características criadas");

  for (int i = 0; i < 2; i++) {
    controlTemp[i]->start();
  }

  for (int i = 0; i < 3; i++) {
    variaveisBLE[i]->setCallbacks(new enableControl(controleAtivo[i]));
  }

  for (int i = 3; i < 6; i++) {
    // passamos o índice do sensor correspondente (0, 1 ou 2)
    variaveisBLE[i]->setCallbacks(new functionsControl(setpoint[i - 3], i - 3));
  }

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();

  Serial.println("Anunciando serviço");

  for (int i = 0; i < 3; i++) {
    ledcSetup(pwmChannel[i], pwmFreq, pwmResolution);
    ledcAttachPin(PWM_PINS[i], pwmChannel[i]);
    ledcWrite(pwmChannel[i], 0);
  }

  for (int i = 0; i < 3; i++) {
    if (lamda) {
      tuneLambda(i, K[i], tau[i], theta[i]);
    }

    a0[i] = Kp[i] * (1 + Ts[i] / (2 * Ti[i]) + Td[i] / Ts[i]);
    a1[i] = -Kp[i] * (1 - Ts[i] / (2 * Ti[i]) + 2 * Td[i] / Ts[i]);
    a2[i] = Kp[i] * Td[i] / Ts[i];
  }
}

// ---------------------- Loop ----------------------

void loop() {
  unsigned long agora = millis();

  for (int i = 0; i < 3; i++) {
    if (controleAtivo[i] && (agora - ultimoTempo[i] >= intervalo)) {
      ultimoTempo[i] = agora;
      Serial.print("Tempo antes do request: ");
      Serial.println(millis());
      sensors[i].requestTemperatures();
      Serial.print("Tempo após o request e antes do index: ");
      Serial.println(millis());
      medida = sensors[i].getTempCByIndex(0);
      Serial.print("Tempo após o index: ");
      Serial.println(millis());
      temperatura[i] = medida; 
      Serial.print("Temperatura (ºC):");
      Serial.println(medida);

      if (millis() - lastRampUpdate[i] >= rampInterval) {
        float newSp;
        if (popSetpoint(&newSp)) {
          setpoint[i] = newSp;
          lastRampUpdate[i] = millis();
        }
      }

      errk[i] = setpoint[i] - temperatura[i];
      
      if (PID_MODE == PID_INCREMENTAL) {
        float DeltaSaida = PID_Incremental(i);
        saida[i] = bias[i] + DeltaSaida + saidaAnterior[i];
        saidaAnterior[i] = saida[i] - bias[i];
      } else { // PID Ideal
        saida[i] = PID_Ideal(i);
      }

      if (saida[i] > 100) saida[i] = 100;
      if (saida[i] < 0)   saida[i] = 0;

      saida[i] = saida[i] * 255 / 100;
      output[i] = constrain(saida[i], 0, 255);
      ledcWrite(pwmChannel[i], (int)output[i]);

      Serial.print("Saída em DC(0-255):");
      Serial.print((int)output[i]);
      Serial.print(" || sensor: ");
      Serial.println(i);

      errk2[i] = errk1[i];
      errk1[i] = errk[i];
    } 
    else if (!controleAtivo[i]) {
      ledcWrite(pwmChannel[i], 0); // desativa saída
      saidaAnterior[i] = 0;
    }
  }
}

// ---------------------- Funções auxiliares ----------------------

void pushSetpoint(float sp) {
  setpointQueue[queueTail] = sp;
  queueTail = (queueTail + 1) % 50;
}

bool popSetpoint(float *sp) {
  if (queueHead == queueTail) return false;
  *sp = setpointQueue[queueHead];
  queueHead = (queueHead + 1) % 50;
  return true;
}

void addRamp(float spOld, float spNew) {
  float step = (spNew - spOld) / rampSteps;
  for (int i = 1; i <= rampSteps; i++) {
    pushSetpoint(spOld + i * step);
  }
}

void setSetpoint(int canal, float novo) {
  float antigo = setpoint[canal];
  addRamp(antigo, novo);
}

float PID_Ideal(int i) {
  float erro = errk[i];
  integral[i] += erro * Ts[i] / Ti[i];
  float deriv = (erro - erroAnterior[i]) * Td[i] / Ts[i];
  float u = bias[i] + Kp[i] * (erro + integral[i] + deriv);
  erroAnterior[i] = erro;
  return u;
}

float PID_Incremental(int i) {
  return a0[i]*errk[i] + a1[i]*errk1[i] + a2[i]*errk2[i];
}

void tuneLambda(int i, float KpProc, float tauProc, float thetaProc) {
  float lambda = 0.5f * tauProc;
  Kp[i] = tauProc / (KpProc * (lambda + thetaProc));
  Ti[i] = 0.25 * tauProc;
  Td[i] = 0;
}
