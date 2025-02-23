#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>

// Configuração dos sensores SHT31
const int numSensores = 25; // Total de 25 sensores
Adafruit_SHT31 sensor[numSensores];
float umidade[numSensores], temperatura[numSensores];
bool sensorAtivo[numSensores]; // Vetor para verificar sensores ativos

// Endereços dos multiplexadores
const uint8_t muxAddr1 = 0x71; // Multiplexador 1
const uint8_t muxAddr2 = 0x70; // Multiplexador 2

// Rede Wi-Fi e URL do script Google
const char* ssid = "UFOB Estudos";
const char* password = "#Uf0bE$tud0$#";
const char* serverName = "https://script.google.com/macros/s/AKfycbwD_ADTMlNeWurE_RutVxfeJSzuHbYNdEljE7tuRvfTzCVI_J7OJWPJ2jK8MmwpL2Loiw/exec";

// Pinos para os sensores de velocidade e direção do vento
float digital_anemometro; // Velocidade
int digital_direcao; // Direção
float velocidade;
int direcao;
float pressao;

#define RXD2 36 // GPIO36 (Sensor VP) para RX
#define TXD2 14 // GPIO14 para TX

// Declaração das funções usadas no código
void TCA9548A(uint8_t muxAddr, uint8_t id);
void desativarMultiplexadores();
void Leitura_SHT31();
void Leitura_Vento();
void Leitura_Direcao();
void print_Serial();
void enviarDados();
float calculaVelocidade(float digital_anemometro);

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Configura a UART2

  Wire.begin();

  // Adiciona um atraso de 2 segundos
  delay(2000);

  // Conecta-se ao Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando ao Wi-Fi...");
  }
  Serial.println("Conectado ao Wi-Fi");

  // Inicializa os sensores SHT31
  for (int i = 0; i < numSensores; i++) {
    uint8_t muxAddr;
    uint8_t channel;
    uint8_t sensorAddr;

    // Configura o multiplexador e o canal
    if (i < 13) {
      muxAddr = muxAddr1; // Primeiro PCA
      if (i == 0) {
        channel = 1; sensorAddr = 0x44; // Canal 1, Sensor 0x44
      } else {
        channel = (i - 1) / 2 + 2; // Canais 2 a 7
        sensorAddr = ((i - 1) % 2 == 0) ? 0x44 : 0x45; // Alterna entre 0x44 e 0x45
      }
    } if (i >= 13) { // Sensores no segundo multiplexador
    muxAddr = muxAddr2; // Segundo PCA
    channel = (i - 13) / 2 + 2; // Canais 2 a 8 no segundo PCA
    sensorAddr = ((i - 13) % 2 == 0) ? 0x44 : 0x45; // Alterna entre 0x44 e 0x45
      }

    // Seleciona o canal
    TCA9548A(muxAddr, channel);

    // Inicializa o sensor e verifica se está ativo
    if (sensor[i].begin(sensorAddr)) {
      sensorAtivo[i] = true; // Sensor ativo
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(" inicializado com sucesso.");
    } else {
      sensorAtivo[i] = false; // Sensor inativo
      Serial.print("Erro ao identificar o sensor ");
      Serial.println(i + 1);
    }
  }

  Serial.println("Configuração dos sensores concluída!");
}

void loop() {
  // Desativa todos os multiplexadores no início do loop
  desativarMultiplexadores();

  // Leitura dos sensores
  Leitura_SHT31();
  Leitura_Vento();
  Leitura_Direcao();

  // Recebe dados do sensor de pressão via comunicação serial com o Arduino Uno
  if (Serial2.available()) {
    String input = Serial2.readStringUntil('\n');
    Serial2.print("Dados recebidos do Arduino: ");
    Serial2.println(input); // Exibe a string recebida
    pressao = input.toFloat();
  } else {
    Serial2.println("Nenhum dado disponível do Arduino.");
  }

  // Envia os dados para o Google Sheets a cada 3 segundos
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 3000) {
    lastUpdate = millis();
    print_Serial();
    enviarDados();
  }
}

void TCA9548A(uint8_t muxAddr, uint8_t id) {
  // Desativa todos os canais do multiplexador antes de selecionar um
  desativarMultiplexadores();

  // Seleciona o canal específico
  Wire.beginTransmission(muxAddr);
  Wire.write(1 << id);
  Wire.endTransmission();
}

void desativarMultiplexadores() {
  // Desativa todos os canais do primeiro multiplexador
  Wire.beginTransmission(muxAddr1);
  Wire.write(0);  // Desativa todos os canais
  Wire.endTransmission();

  // Desativa todos os canais do segundo multiplexador
  Wire.beginTransmission(muxAddr2);
  Wire.write(0);  // Desativa todos os canais
  Wire.endTransmission();
}

float offsetTemperatura[numSensores] = {
  0.54, 0.09, -0.56, -0.11, -0.42, 0.24, -0.54, 0.12, -0.2, -0.21, -0.47,
  -0.24, -0.44, -0.08, 0.14, -0.19, 0.27, -0.19, 0.03, -0.38, 0.22, -0.17,
  0.49, -0.19, 0.45
};

float offsetUmidade[numSensores] = {
  -2.06, -5.21, -5.18, -2.94, -1.24, -4.43, -1.43, -3.28, -1.62, -1.94,
  -1.84, -1.09, -2.18, -1.76, -3.16, -2.97, -3.39, -2.4, -2.47, -1.97,
  -4.02, -3.84, -4.42, -2.39, -4.64
};



void Leitura_SHT31() {
  for (int i = 0; i < numSensores; i++) {
    if (!sensorAtivo[i]) continue; // Pula sensores inativos

    uint8_t muxAddr;
    uint8_t channel;

    // Configuração do multiplexador e sensor
    if (i < 13) {
      muxAddr = muxAddr1;
      channel = (i == 0) ? 1 : (i - 1) / 2 + 2; // Canais
    } else {
      muxAddr = muxAddr2;
      channel = (i - 13) / 2 + 2; // Canais
    }

    // Seleciona o canal
    TCA9548A(muxAddr, channel);

    // Realiza a leitura
    temperatura[i] = sensor[i].readTemperature() + offsetTemperatura[i];
    umidade[i] = sensor[i].readHumidity() + offsetUmidade[i];

    if (isnan(temperatura[i]) || isnan(umidade[i])) {
      temperatura[i] = 0;
      umidade[i] = 0;
    }
  }
}

float calculaVelocidade(float digital_anemometro) {
  float x = 1.0; // Chute inicial
  for (int i = 0; i < 100; i++) {
    float fx = 0.5069 * pow(x, 4) - 11.617 * pow(x, 3) + 85.377 * pow(x, 2) - 65.364 * x + 4.1321 - digital_anemometro;
    float dfx = 4 * 0.5069 * pow(x, 3) - 3 * 11.617 * pow(x, 2) + 2 * 85.377 * x - 65.364;
    x = x - fx / dfx;
  }
  return x;
}

void Leitura_Vento() {
  digital_anemometro = analogRead(39);
  velocidade = calculaVelocidade(digital_anemometro);
}

void Leitura_Direcao() {
  digital_direcao = analogRead(34); // Direção do vento

  if (digital_direcao >= 0 && digital_direcao <= 100) {
    direcao = 0; // 0°
  } else if (digital_direcao >= 400 && digital_direcao <= 500) {
    direcao = 45; // 45°
  } else if (digital_direcao >= 900 && digital_direcao <= 1100) {
    direcao = 90;
  } else if (digital_direcao >= 1400 && digital_direcao <= 1700) {
    direcao = 135;
  } else if (digital_direcao >= 2000 && digital_direcao <= 2300) {
    direcao = 180;
  } else if (digital_direcao >= 2800 && digital_direcao <= 3000) {
    direcao = 225;
  } else if (digital_direcao >= 3400 && digital_direcao <= 3700) {
    direcao = 270;
  } else if (digital_direcao >= 4000 && digital_direcao <= 4200) {
    direcao = 315;
  }
}

void print_Serial() {
  for (int i = 0; i < numSensores; i++) {
    if (!sensorAtivo[i]) continue; // Pula sensores inativos

    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": Temperatura = ");
    Serial.print(temperatura[i]);
    Serial.print(" ºC, Umidade = ");
    Serial.print(umidade[i]);
    Serial.println(" %");
  }
  Serial.print("Pressão: ");
  Serial.print(pressao);
  Serial.println(" psi");
  Serial.print("Velocidade do Vento: ");
  Serial.print(velocidade);
  Serial.println(" m/s");
  Serial.print("Direção do Vento: ");
  Serial.println(direcao);
  Serial.println("---------------------------");
}



void enviarDados() {

  HTTPClient http;
  String url = String(serverName) + "?";

  // Adiciona os dados de temperatura e umidade
  for (int i = 0; i < numSensores; i++) {
    if (sensorAtivo[i]) {
      url += "temperatura" + String(i) + "=" + String(temperatura[i]) + "&";
      url += "umidade" + String(i) + "=" + String(umidade[i]) + "&";
    }
  }

  // Adiciona os dados adicionais
  url += "pressao=" + String(pressao) + "&";
  url += "velocidade=" + String(velocidade) + "&";
  url += "direcao=" + String(direcao);

  http.begin(url);
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Resposta do servidor:");
    Serial.println(response);
  } else {
    Serial.print("Erro ao enviar dados. Código HTTP: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}