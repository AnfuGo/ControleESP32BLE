#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include <BLE2902.h> // desnecessário
#include <OneWire.h> // sensor
#include <DallasTemperature.h> //sensor

// UUID-bluetooth-----128bits gerados através do gerador online de UUID.
const char* CIA_CONTROL_ENABLE = "22086d8b-57c2-4eb4-b82d-4b7936413e78";
const char* CONTROLE_TEMP     = "2f7d55d9-acce-4a1d-9871-dd3e4ec73eec";
const char* ENABLE_1          = "1b4a81b4-abf5-450d-9d4d-81b4e951baa7";
const char* ENABLE_2          = "d2b757c1-9fb1-4166-af83-467a0a0a55f3";
const char* ENABLE_3          = "765170c6-24e5-4890-a346-10195b57ca7c";
const char* AlTemp_1          = "28b66ce2-882e-46e2-bc17-1332f33072db";
const char* AlTemp_2          = "83ec522b-14b0-4b42-8a8b-0924993f5490";
const char* AlTemp_3          = "e49b8658-6f76-43dd-a36e-db7f4b1aa546";

const char* SERVICE_UUID[2] = { CIA_CONTROL_ENABLE, CONTROLE_TEMP };
const char* CHARACTERISTIC_UUID[6] = { ENABLE_1, ENABLE_2, ENABLE_3, AlTemp_1, AlTemp_2, AlTemp_3 };

const int Sensor_PINS[3] = { 5, 18, 19 };  // definir pinos para os sensores
const int PWM_PINS[3]    = { 1,  2,  3 };  // potência de saída para resistências, definir pinos GPIO

const int pwmChannel[3]  = { 0, 1, 2 };
const int pwmFreq        = 1000;     // frequência PWM
const int pwmResolution  = 8;        // 8 bits → 0-255
const float Vref         = 3.3;      // tensão de referência
//const int ADC_MAX        = 4095;

unsigned long ultimoTempo[3] = { 0, 0, 0 }; 
const unsigned long intervalo = 500; // intervalo de aquisição

// Parâmetros FOPDT do sistema
float K[3]     = { 1, 1, 1 };
float tau[3]   = { 3, 3, 3 };
float theta[3] = { 4, 4, 4 };
float medida;

int setpoint[3]      = { 25, 25, 25 };
bool controleAtivo[3] = { false, false, false };

// Variáveis PID
float Kp[3], Ti[3], Td[3];
float DeltaSaida[3];
float saidaAnterior[3];
float saida[3];     // porcentagem
int output[3];      // saída em 0-255
float errk2[3] = { 0, 0, 0 };
float errk1[3] = { 0, 0, 0 };
float errk[3]  = { 0, 0, 0 };

float temperatura[3] = { 25, 25, 25 };
float a0[3], a1[3], a2[3];
const unsigned long Ts[3] = { 500, 500, 500 };

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
    Serial.println("Desconectado!");
  }
};

class enableControl : public BLECharacteristicCallbacks {
  bool &controle;
public:
  enableControl(bool &control) : controle(control) {}
  void onWrite(BLECharacteristic* ENABLE_CONTROL) override {
    std::string value = ENABLE_CONTROL->getValue();
    bool enable_control = (value == "1");
    controle = enable_control;

    if (enable_control) {
      Serial.println("Controle habilitado");
    } else {
      Serial.println("Controle desabilitado");
    }
  }
};

class functionsControl : public BLECharacteristicCallbacks {
  int &set_point;
public:
  functionsControl(int &st) : set_point(st) {}
  void onWrite(BLECharacteristic* variaveisBLE) override {
    std::string value = variaveisBLE->getValue();
    set_point = atoi(value.c_str());
    Serial.println(value.c_str());
  }
  void onRead(BLECharacteristic* variaveisBLE) override {
    std::string value = variaveisBLE->getValue();
    Serial.print("Valor lido: ");
    Serial.println(value.c_str());
  }
};

// ---------------------- Setup ----------------------

void setup() {
  Serial.begin(115200);
  BLEDevice::init("ESP32S_CIA");
  BLEServer *servidorCIA = BLEDevice::createServer();
  servidorCIA->setCallbacks(new enableServer());
  for (int i = 0; i < 3; i++){
    sensors[i].begin();
    sensors[i].setResolution(12);
  }
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

  for (int i = 0; i < 2; i++) {
    controlTemp[i]->start();
  }

  for (int i = 0; i < 3; i++) {
    variaveisBLE[i]->setCallbacks(new enableControl(controleAtivo[i]));
  }
  for (int i = 3; i < 6; i++) {
    variaveisBLE[i]->setCallbacks(new functionsControl(setpoint[i - 3]));
  }

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();

  for (int i = 0; i < 3; i++) {
    ledcSetup(pwmChannel[i], pwmFreq, pwmResolution);
    ledcAttachPin(PWM_PINS[i], pwmChannel[i]);
    ledcWrite(pwmChannel[i], 0);
  }

  // Cálculo parâmetros PID incremental
  for (int i = 0; i < 3; i++) {
    Kp[i] = 1.2 * (tau[i] / (K[i] * theta[i]));
    Ti[i] = 2.0 * theta[i];
    Td[i] = 0.5 * theta[i];
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
      sensors[i].requestTemperatures();
      medida = sensors[i].getTempCByIndex(0);
      temperatura[i] = medida; 

      errk[i] = setpoint[i] - temperatura[i];
      DeltaSaida[i] = a0[i] * errk[i] + a1[i] * errk1[i] + a2[i] * errk2[i];

      saida[i] = DeltaSaida[i] + saidaAnterior[i]; // porcentagem do DC
      saidaAnterior[i] = saida[i];
      saida[i] = saida[i] * 255 / 100;

      output[i] = constrain(saida[i], 0, 255);
      ledcWrite(pwmChannel[i], (int)output[i]);

      errk2[i] = errk1[i];
      errk1[i] = errk[i];
    } 
    else if (!controleAtivo[i]) {
      ledcWrite(pwmChannel[i], 0); // desativa saída
    }
  }
}
