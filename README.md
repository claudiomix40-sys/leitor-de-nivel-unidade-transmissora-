
//unidade TRANSMISSORA//

// autor: Claudio Augusto R. Betoni

#include <SPI.h>
#include <LoRa.h>

// =====================================================
// PINOS DO SENSOR RCWL-1655
// =====================================================

const int PINO_TRIG = 32;
const int PINO_ECHO = 33;

// =====================================================
// PINOS DO LORA SX1278
// =====================================================

const int LORA_SCK  = 18;
const int LORA_MISO = 19;
const int LORA_MOSI = 23;
const int LORA_CS   = 5;
const int LORA_RST  = 14;
const int LORA_DIO0 = 26;

// Frequência do módulo SX1278
const long FREQUENCIA_LORA = 433E6;

// =====================================================
// CONFIGURAÇÃO DAS MEDIÇÕES
// =====================================================

const int TOTAL_AMOSTRAS = 11;

const float DISTANCIA_MINIMA_CM = 20.0;
const float DISTANCIA_MAXIMA_CM = 70.0;

// Tempo máximo aguardando o ECHO
const unsigned long TIMEOUT_ECHO_US = 30000;

// Intervalo entre leituras individuais
const unsigned long INTERVALO_AMOSTRAS_MS = 60;

// Intervalo entre transmissões
const unsigned long INTERVALO_ENVIO_MS = 2000;

// Filtro exponencial:
// menor = mais estável e mais lento
// maior = mais rápido e mais instável
const float ALFA_FILTRO = 0.20;

// =====================================================
// VARIÁVEIS
// =====================================================

float distanciaFiltrada = -1.0;

unsigned long ultimoEnvio = 0;
unsigned long numeroPacote = 0;

// =====================================================
// FAZ UMA LEITURA DO SENSOR
// =====================================================

float medirDistanciaUnica() {
  digitalWrite(PINO_TRIG, LOW);
  delayMicroseconds(5);

  digitalWrite(PINO_TRIG, HIGH);
  delayMicroseconds(12);

  digitalWrite(PINO_TRIG, LOW);

  unsigned long duracao = pulseIn(
    PINO_ECHO,
    HIGH,
    TIMEOUT_ECHO_US
  );

  if (duracao == 0) {
    return -1.0;
  }

  float distancia = duracao * 0.0343 / 2.0;

  if (
    distancia < DISTANCIA_MINIMA_CM ||
    distancia > DISTANCIA_MAXIMA_CM
  ) {
    return -1.0;
  }

  return distancia;
}

// =====================================================
// ORDENA AS LEITURAS
// =====================================================

void ordenarValores(float valores[], int quantidade) {
  for (int i = 0; i < quantidade - 1; i++) {
    for (int j = i + 1; j < quantidade; j++) {
      if (valores[j] < valores[i]) {
        float temporario = valores[i];
        valores[i] = valores[j];
        valores[j] = temporario;
      }
    }
  }
}

// =====================================================
// CALCULA A MEDIANA
// =====================================================

float medirDistanciaMediana() {
  float valores[TOTAL_AMOSTRAS];
  int quantidadeValida = 0;

  for (int i = 0; i < TOTAL_AMOSTRAS; i++) {
    float leitura = medirDistanciaUnica();

    if (leitura >= 0) {
      valores[quantidadeValida] = leitura;
      quantidadeValida++;
    }

    delay(INTERVALO_AMOSTRAS_MS);
  }

  // Exige pelo menos cinco leituras válidas
  if (quantidadeValida < 5) {
    return -1.0;
  }

  ordenarValores(valores, quantidadeValida);

  int meio = quantidadeValida / 2;

  if (quantidadeValida % 2 == 0) {
    return (
      valores[meio - 1] +
      valores[meio]
    ) / 2.0;
  }

  return valores[meio];
}

// =====================================================
// APLICA SUAVIZAÇÃO
// =====================================================

void atualizarFiltro(float novaDistancia) {
  if (novaDistancia < 0) {
    return;
  }

  if (distanciaFiltrada < 0) {
    distanciaFiltrada = novaDistancia;
    return;
  }

  distanciaFiltrada =
    ALFA_FILTRO * novaDistancia +
    (1.0 - ALFA_FILTRO) * distanciaFiltrada;
}

// =====================================================
// ENVIA PELO LORA
// =====================================================

void enviarDadosLoRa(float distancia) {
  numeroPacote++;

  /*
    Formato transmitido:

    NIVEL,numeroPacote,distancia

    Exemplo:
    NIVEL,25,83.4
  */

  LoRa.beginPacket();

  LoRa.print("NIVEL,");
  LoRa.print(numeroPacote);
  LoRa.print(",");
  LoRa.print(distancia, 1);

  LoRa.endPacket();

  Serial.print("Pacote enviado: ");
  Serial.print(numeroPacote);

  Serial.print(" | Distancia: ");
  Serial.print(distancia, 1);
  Serial.println(" cm");
}

// =====================================================
// INICIALIZA O LORA
// =====================================================

void iniciarLoRa() {
  SPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_CS
  );

  LoRa.setPins(
    LORA_CS,
    LORA_RST,
    LORA_DIO0
  );

  Serial.println("Iniciando LoRa 433 MHz...");

  if (!LoRa.begin(FREQUENCIA_LORA)) {
    Serial.println("Falha ao iniciar o LoRa.");
    Serial.println("Verifique as ligacoes.");

    while (true) {
      delay(1000);
    }
  }

  /*
    Configuração simples e confiável para curta distância.
    Os dois módulos precisam ter as mesmas configurações.
  */

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(17);
  LoRa.enableCrc();

  Serial.println("LoRa iniciado.");
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(PINO_TRIG, OUTPUT);
  pinMode(PINO_ECHO, INPUT);

  digitalWrite(PINO_TRIG, LOW);

  delay(1000);

  Serial.println();
  Serial.println("============================");
  Serial.println("TRANSMISSOR DE NIVEL");
  Serial.println("============================");

  iniciarLoRa();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  if (
    millis() - ultimoEnvio >=
    INTERVALO_ENVIO_MS
  ) {
    ultimoEnvio = millis();

    float distanciaMediana =
      medirDistanciaMediana();

    if (distanciaMediana >= 0) {
      atualizarFiltro(distanciaMediana);

      enviarDadosLoRa(distanciaFiltrada);
    } else {
      Serial.println(
        "Nao foi possivel medir a distancia."
      );
    }
  }
}
