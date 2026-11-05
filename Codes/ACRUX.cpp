/*
  Firmware de Voo - Foguete (Versão com BME280 Primário para Altura/Velocidade e MPU6050 Secundário)
  ESP32-S3 Super Mini + MPU6050 + BME280 + Cartão Micro SD
  V6: CODIGO ORIGINAL + correcao SOMENTE do armazenamento.
  Toda linha alterada/adicionada esta marcada com  // [ARMAZENAMENTO]
  Lancamento, apice, paraquedas, pouso, buzzer e LED: IGUAIS ao original.
*/

#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <LittleFS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>

// ---------- Pinagem ----------
#define I2C_SDA        2
#define I2C_SCL        3

#define SD_CS          4
#define SD_SCK         5
#define SD_MOSI        6
#define SD_MISO        7

#define SQUIB_PIN      1
#define BUZZER_PIN     8
#define LED_EJECAO_PIN 48

#define BME280_ADDR    0x76

// ---------- Parâmetros de voo ----------
const bool  MODO_TESTE_ATIVO = true;   // false no voo real!

const float LIMIAR_VELOCIDADE_LANCAMENTO = 4.0;   // m/s (baseado na variação barométrica)
const float GRAVIDADE = 9.80665;                   // m/s^2 (mantido para checagem de pouso)
const unsigned long DURACAO_PULSO_SQUIB_MS = 1500;

// Ápice
const unsigned long TEMPO_MINIMO_ANTES_APICE_MS = 1000;
const int   CONFIRMACOES_APICE = 3;                // leituras seguidas confirmando queda
const float QUEDA_ALTURA_APICE_M = 10.0;           // 10 metros de queda para acionar o paraquedas
const unsigned long TEMPO_MAXIMO_VOO_MS = 30000;   // ejeção de reserva

// Pouso
const float TOLERANCIA_ACEL_PARADO = 0.3;          // m/s^2
const float VARIACAO_ALTURA_PARADO_M = 1.0;        // m
const unsigned long TEMPO_PARADO_PARA_POUSO_MS = 5000;

// Amostragem e calibração
const unsigned long INTERVALO_LEITURA_MS = 20;     // 50 Hz
const int AMOSTRAS_CALIBRACAO_PRESSAO = 20;
const int AMOSTRAS_CALIBRACAO_ACEL = 100;

// Gravação
const int    TAMANHO_BUFFER_PRE_LANCAMENTO = 100;
const size_t TAMANHO_BUFFER_TEXTO = 2048;
const unsigned long INTERVALO_FLUSH_MS = 500;
const size_t FLASH_MINIMA_LIVRE = 4096;
const unsigned long INTERVALO_SERIAL_MS = 200;
const unsigned long INTERVALO_RETENTATIVA_SD_MS = 10000;

// ---------- Objetos ----------
Adafruit_MPU6050 mpu;
Adafruit_BME280  bme;
File arquivoFlash;
File arquivoSD;                 // [ARMAZENAMENTO] gravacao direta no SD durante o voo
bool sdGravando = false;        // [ARMAZENAMENTO]

const char* ARQUIVO_FLASH    = "/voo.csv";
const char* ARQUIVO_ANTERIOR = "/voo_anterior.csv";

// ---------- Estados ----------
enum EstadoVoo { AGUARDANDO, VOO, DESCIDA, POUSADO };
EstadoVoo estado = AGUARDANDO;
const char* NOMES_ESTADO[] = { "AGUARDANDO", "VOO", "DESCIDA", "POUSADO" };

// ---------- Estrutura de Amostra ----------
struct Amostra {
  uint32_t tempoUs;
  uint8_t  estado;
  float vel, alt, altMax;
  // Dados secundários (Acelerômetro / Giroscópio)
  float ax, ay, az;
  float gx, gy, gz;
  float tMpu, tBme, umid, pres;
};

// ---------- Variáveis de controle ----------
float velocidadeVertical = 0.0;
unsigned long ultimoTempoMicros = 0;
unsigned long ultimaLeituraMs = 0;

float pressaoReferenciaHpa = 1013.25;
float alturaAtual = 0.0;
float alturaAnterior = 0.0;
float alturaMaxima = 0.0;

bool squibLigado = false;
unsigned long inicioSquibMs = 0;

unsigned long inicioVooMs = 0;
int contadorApice = 0;

bool contandoParado = false;
unsigned long tempoParadoInicioMs = 0;
float alturaInicioParado = 0.0;

bool pousoProcessado = false;
bool dadosTransferidos = false;
bool tentouTransferir = false;
unsigned long ultimaTentativaTransferenciaMs = 0;

Amostra bufferPre[TAMANHO_BUFFER_PRE_LANCAMENTO];
int  indicePre = 0;
bool bufferPreCheio = false;

char   bufferTexto[TAMANHO_BUFFER_TEXTO];
size_t tamanhoBufferTexto = 0;
unsigned long ultimoFlushMs = 0;
bool flashCheia = false;

unsigned long ultimoSerialMs = 0;

bool mpuDetectado = false;
bool bmeDetectado = false;
bool flashDetectada = false;
bool sdDetectado = false;

String comandoSerial = "";

// ---------- Protótipos ----------
void inicializarSensores();
void mostrarResumoDeteccao();
void calibrarPressaoReferencia();
void calibrarAcelerometro();
void inicializarFlash();
void inicializarSD();
float calcularAltura(float pressaoHpa);
void iniciarVoo();
void acionarSquib();
void acionarApice();
void processarComandoSerial();
void atualizarPulsoSquib();
void verificarPouso(float ax, float ay, float az, unsigned long agoraMs);
void guardarNoBufferPre(const Amostra &a);
void descarregarBufferPreNaFlash();
void registrarAmostra(const Amostra &a);
void descarregarBufferNaFlash();
void tratarPouso();
bool transferirFlashParaSD();
void proximoNomeLivre(fs::FS &fs, const char* prefixo, char* saida, size_t tam);          // [ARMAZENAMENTO]
bool copiarArquivo(fs::FS &de, const char* origem, fs::FS &para, const char* destino);   // [ARMAZENAMENTO]
void enviarVoosAntigosParaSD();                                                           // [ARMAZENAMENTO]

void setup() {
  Serial.begin(115200);
  unsigned long inicioEspera = millis();
  while (!Serial && millis() - inicioEspera < 3000) { delay(10); }

  Serial.println();
  Serial.println("=== Firmware de Voo - BME Primario / MPU Secundario ===");

  pinMode(SQUIB_PIN, OUTPUT);
  digitalWrite(SQUIB_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_EJECAO_PIN, OUTPUT);
  digitalWrite(LED_EJECAO_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);

  inicializarSensores();
  calibrarAcelerometro(); // Calibração secundária apenas para referência
  calibrarPressaoReferencia();
  inicializarFlash();
  inicializarSD();

  mostrarResumoDeteccao();

  ultimoTempoMicros = micros();
  ultimaLeituraMs = millis();
  ultimoFlushMs = millis();

  Serial.println("Pronto. Aguardando lancamento...");
}

void loop() {
  processarComandoSerial();
  atualizarPulsoSquib();

  if (estado == POUSADO) {
    tratarPouso();
    return;
  }

  unsigned long agoraMs = millis();
  if (agoraMs - ultimaLeituraMs < INTERVALO_LEITURA_MS) return;
  ultimaLeituraMs = agoraMs;

  unsigned long agoraMicros = micros();
  float dt = (agoraMicros - ultimoTempoMicros) / 1000000.0;
  ultimoTempoMicros = agoraMicros;

  // --- Leituras dos Sensores ---
  sensors_event_t accel, giro, tempMpu;
  mpu.getEvent(&accel, &giro, &tempMpu); // Dados secundários

  float temperatura = bme.readTemperature();
  float pressao = bme.readPressure() / 100.0F;
  float umidade = bme.readHumidity();

  // --- Altura e Velocidade baseadas EXCLUSIVAMENTE no BME280 ---
  alturaAnterior = alturaAtual;
  float h = calcularAltura(pressao);
  if (!isnan(h)) {
    alturaAtual = h;
    if (dt > 0) {
      velocidadeVertical = (alturaAtual - alturaAnterior) / dt; // Derivada da altura barométrica
    }
  }

  if (alturaAtual > alturaMaxima) {
    alturaMaxima = alturaAtual;
  }

  // --- Máquina de Estados ---
  switch (estado) {
    case AGUARDANDO:
      if (velocidadeVertical > LIMIAR_VELOCIDADE_LANCAMENTO) {
        iniciarVoo();
      }
      break;

    case VOO: {
      unsigned long tempoVoo = agoraMs - inicioVooMs;

      // Ápice confirmado quando a altura cair 10 metros abaixo do máximo
      bool apiceAlturaQueda = (alturaMaxima - alturaAtual) >= QUEDA_ALTURA_APICE_M;

      if (apiceAlturaQueda) {
        contadorApice++;
      } else {
        contadorApice = 0;
      }

      if (tempoVoo >= TEMPO_MINIMO_ANTES_APICE_MS && contadorApice >= CONFIRMACOES_APICE) {
        acionarApice();
      } else if (tempoVoo >= TEMPO_MAXIMO_VOO_MS) {
        Serial.println("!!! TEMPO MAXIMO DE VOO - EJECAO DE RESERVA !!!");
        acionarApice();
      }
      break;
    }

    case DESCIDA:
      // O acelerômetro é usado aqui de forma secundária apenas para detectar o pouso
      verificarPouso(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, agoraMs);
      break;

    case POUSADO:
      break;
  }

  // --- Montagem da Amostra (Registrando dados do MPU como secundários) ---
  Amostra a;
  a.tempoUs = agoraMicros;
  a.estado = (uint8_t)estado;
  a.vel = velocidadeVertical;
  a.alt = alturaAtual;
  a.altMax = alturaMaxima;
  a.ax = accel.acceleration.x;
  a.ay = accel.acceleration.y;
  a.az = accel.acceleration.z;
  a.gx = giro.gyro.x;
  a.gy = giro.gyro.y;
  a.gz = giro.gyro.z;
  a.tMpu = tempMpu.temperature;
  a.tBme = temperatura;
  a.umid = umidade;
  a.pres = pressao;

  if (estado == AGUARDANDO) {
    guardarNoBufferPre(a);
  } else {
    registrarAmostra(a);
    if (millis() - ultimoFlushMs >= INTERVALO_FLUSH_MS) {
      descarregarBufferNaFlash();
    }
  }

  // --- Serial ---
  if (agoraMs - ultimoSerialMs >= INTERVALO_SERIAL_MS) {
    ultimoSerialMs = agoraMs;
    Serial.print("Estado: ");
    Serial.print(NOMES_ESTADO[estado]);
    Serial.print(" | Vel (Baro): ");
    Serial.print(velocidadeVertical, 2);
    Serial.print(" m/s | Alt: ");
    Serial.print(alturaAtual, 1);
    Serial.print(" m | Max: ");
    Serial.print(alturaMaxima, 1);
    Serial.println(" m");
  }
}

// =====================================================================
// Funções Auxiliares
// =====================================================================

float calcularAltura(float pressaoHpa) {
  if (isnan(pressaoHpa) || pressaoHpa <= 0) return NAN;
  return 44330.0 * (1.0 - pow(pressaoHpa / pressaoReferenciaHpa, 0.1903));
}

void iniciarVoo() {
  estado = VOO;
  inicioVooMs = millis();
  alturaMaxima = alturaAtual;
  contadorApice = 0;
  Serial.println(">>> LANCAMENTO DETECTADO (BME) <<<");
  descarregarBufferPreNaFlash();
}

void inicializarSensores() {
  mpuDetectado = mpu.begin();
  if (mpuDetectado) {
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  }

  bmeDetectado = bme.begin(BME280_ADDR, &Wire);
  if (bmeDetectado) {
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X4,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_X2,
                    Adafruit_BME280::STANDBY_MS_0_5);
  }
}

void mostrarResumoDeteccao() {
  Serial.println("======== RESUMO DOS SENSORES ========");
  Serial.print("MPU6050 (Secundario): "); Serial.println(mpuDetectado ? "OK" : "FALHOU");
  Serial.print("BME280  (Primario):   "); Serial.println(bmeDetectado ? "OK" : "FALHOU");
  Serial.print("Flash:                "); Serial.println(flashDetectada ? "OK" : "FALHOU");   // [ARMAZENAMENTO]
  Serial.print("Cartao SD:            "); Serial.println(sdGravando ? "OK (gravando direto)" : "FALHOU"); // [ARMAZENAMENTO]
  Serial.println("======================================");
}

void calibrarAcelerometro() {
  // Apenas informativo no modo secundário
}

void calibrarPressaoReferencia() {
  if (!bmeDetectado) return;
  float soma = 0;
  for (int i = 0; i < AMOSTRAS_CALIBRACAO_PRESSAO; i++) {
    soma += bme.readPressure() / 100.0F;
    delay(50);
  }
  pressaoReferenciaHpa = soma / AMOSTRAS_CALIBRACAO_PRESSAO;
  Serial.print("P0 Calibrado (BME): "); Serial.print(pressaoReferenciaHpa); Serial.println(" hPa");
}

void inicializarFlash() {
  if (!LittleFS.begin(true)) {
    flashDetectada = false;
    return;
  }
  // [ARMAZENAMENTO] Antes: voo.csv -> voo_anterior.csv e o voo_anterior antigo era APAGADO.
  // Agora o voo anterior vira /voo_NNN.csv na flash (nada e apagado) e e enviado ao SD no boot.
  const char* antigos[] = { ARQUIVO_ANTERIOR, ARQUIVO_FLASH };                 // [ARMAZENAMENTO]
  for (int i = 0; i < 2; i++) {                                               // [ARMAZENAMENTO]
    if (LittleFS.exists(antigos[i])) {                                        // [ARMAZENAMENTO]
      char novo[24];                                                          // [ARMAZENAMENTO]
      proximoNomeLivre(LittleFS, "voo", novo, sizeof(novo));                  // [ARMAZENAMENTO]
      LittleFS.rename(antigos[i], novo);                                      // [ARMAZENAMENTO]
    }                                                                         // [ARMAZENAMENTO]
  }                                                                           // [ARMAZENAMENTO]
  arquivoFlash = LittleFS.open(ARQUIVO_FLASH, "w");
  if (arquivoFlash) {
    arquivoFlash.println("tempo_us,estado,vel_vertical,altura_m,altura_maxima_m,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z,temp_mpu_c,temp_bme_c,umidade_pct,pressao_hpa");
    arquivoFlash.flush();
    flashDetectada = true;
  }
}

void inicializarSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdDetectado = SD.begin(SD_CS, SPI);
  if (!sdDetectado) return;                                                   // [ARMAZENAMENTO]

  enviarVoosAntigosParaSD();   // [ARMAZENAMENTO] voos que ficaram na flash vao para o SD

  // [ARMAZENAMENTO] Arquivo novo no SD gravado AO MESMO TEMPO que a flash:
  // os dados chegam ao SD mesmo se o pouso nunca for detectado.
  char nome[24];                                                              // [ARMAZENAMENTO]
  proximoNomeLivre(SD, "voo", nome, sizeof(nome));                            // [ARMAZENAMENTO]
  arquivoSD = SD.open(nome, FILE_WRITE);                                      // [ARMAZENAMENTO]
  if (arquivoSD) {                                                            // [ARMAZENAMENTO]
    arquivoSD.println("tempo_us,estado,vel_vertical,altura_m,altura_maxima_m,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z,temp_mpu_c,temp_bme_c,umidade_pct,pressao_hpa"); // [ARMAZENAMENTO]
    arquivoSD.flush();                                                        // [ARMAZENAMENTO]
    sdGravando = true;                                                        // [ARMAZENAMENTO]
    Serial.print("Gravando no SD em "); Serial.println(nome);                 // [ARMAZENAMENTO]
  }                                                                           // [ARMAZENAMENTO]
}

void acionarSquib() {
  digitalWrite(SQUIB_PIN, HIGH);
  squibLigado = true;
  inicioSquibMs = millis();
}

void acionarApice() {
  estado = DESCIDA;
  Serial.println(">>> APICE DETECTADO (QUEDA DE 10M) - ACIONANDO EJECAO <<<");
  acionarSquib();
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_EJECAO_PIN, HIGH);
}

void atualizarPulsoSquib() {
  if (squibLigado && (millis() - inicioSquibMs >= DURACAO_PULSO_SQUIB_MS)) {
    digitalWrite(SQUIB_PIN, LOW);
    squibLigado = false;
  }
}

void processarComandoSerial() {
  if (!MODO_TESTE_ATIVO) return;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      comandoSerial.trim();
      comandoSerial.toUpperCase();
      if (comandoSerial == "LANCAR" && estado == AGUARDANDO) iniciarVoo();
      else if (comandoSerial == "APICE" && estado == VOO) acionarApice();
      comandoSerial = "";
    } else {
      comandoSerial += c;
    }
  }
}

void verificarPouso(float ax, float ay, float az, unsigned long agoraMs) {
  float modulo = sqrt(ax * ax + ay * ay + az * az);
  bool acelParada = fabs(modulo - GRAVIDADE) < TOLERANCIA_ACEL_PARADO;

  if (!acelParada) {
    contandoParado = false;
    return;
  }

  if (!contandoParado) {
    contandoParado = true;
    tempoParadoInicioMs = agoraMs;
    alturaInicioParado = alturaAtual;
  } else if (fabs(alturaAtual - alturaInicioParado) > VARIACAO_ALTURA_PARADO_M) {
    tempoParadoInicioMs = agoraMs;
    alturaInicioParado = alturaAtual;
  } else if (agoraMs - tempoParadoInicioMs >= TEMPO_PARADO_PARA_POUSO_MS) {
    estado = POUSADO;
    Serial.println(">>> POUSO DETECTADO <<<");
  }
}

void tratarPouso() {
  if (!pousoProcessado) {
    pousoProcessado = true;
    descarregarBufferNaFlash();
    if (arquivoFlash) arquivoFlash.close();
    if (sdGravando) { arquivoSD.close(); sdGravando = false; }               // [ARMAZENAMENTO]
  }
  // [ARMAZENAMENTO] antes tentava sem parar (criando voo_001..voo_999); agora a cada 10 s
  if (!dadosTransferidos && sdDetectado &&
      (!tentouTransferir || millis() - ultimaTentativaTransferenciaMs >= INTERVALO_RETENTATIVA_SD_MS)) { // [ARMAZENAMENTO]
    tentouTransferir = true;                                                  // [ARMAZENAMENTO]
    ultimaTentativaTransferenciaMs = millis();                                // [ARMAZENAMENTO]
    dadosTransferidos = transferirFlashParaSD();
  }
}

void guardarNoBufferPre(const Amostra &a) {
  bufferPre[indicePre] = a;
  indicePre = (indicePre + 1) % TAMANHO_BUFFER_PRE_LANCAMENTO;
  if (indicePre == 0) bufferPreCheio = true;
}

void descarregarBufferPreNaFlash() {
  int inicio = bufferPreCheio ? indicePre : 0;
  int quantidade = bufferPreCheio ? TAMANHO_BUFFER_PRE_LANCAMENTO : indicePre;
  for (int i = 0; i < quantidade; i++) {
    registrarAmostra(bufferPre[(inicio + i) % TAMANHO_BUFFER_PRE_LANCAMENTO]);
  }
}

void registrarAmostra(const Amostra &a) {
  if ((!flashDetectada || flashCheia) && !sdGravando) return;   // [ARMAZENAMENTO] SD continua mesmo com a flash cheia
  char linha[220];
  int n = snprintf(linha, sizeof(linha),
                   "%lu,%u,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f\n",
                   (unsigned long)a.tempoUs, (unsigned)a.estado,
                   a.vel, a.alt, a.altMax,
                   a.ax, a.ay, a.az, a.gx, a.gy, a.gz,
                   a.tMpu, a.tBme, a.umid, a.pres);
  if (n <= 0) return;
  if (tamanhoBufferTexto + n > TAMANHO_BUFFER_TEXTO) {
    descarregarBufferNaFlash();
  }
  memcpy(bufferTexto + tamanhoBufferTexto, linha, n);
  tamanhoBufferTexto += n;
}

void descarregarBufferNaFlash() {
  ultimoFlushMs = millis();
  if (tamanhoBufferTexto == 0) return;
  if (arquivoFlash && !flashCheia) {                                          // [ARMAZENAMENTO]
    size_t escrito = arquivoFlash.write((const uint8_t*)bufferTexto, tamanhoBufferTexto); // [ARMAZENAMENTO]
    arquivoFlash.flush();
    if (escrito != tamanhoBufferTexto) flashCheia = true;   // [ARMAZENAMENTO] antes falhava em silencio
  }                                                                           // [ARMAZENAMENTO]
  if (sdGravando) {                                                           // [ARMAZENAMENTO]
    size_t escritoSD = arquivoSD.write((const uint8_t*)bufferTexto, tamanhoBufferTexto); // [ARMAZENAMENTO]
    arquivoSD.flush();                                                        // [ARMAZENAMENTO]
    if (escritoSD != tamanhoBufferTexto) { arquivoSD.close(); sdGravando = false; } // [ARMAZENAMENTO] SD com erro: segue so na flash
  }                                                                           // [ARMAZENAMENTO]
  tamanhoBufferTexto = 0;
}

bool transferirFlashParaSD() {
  File origem = LittleFS.open(ARQUIVO_FLASH, "r");
  if (!origem) return false;
  size_t tamanhoOrigem = origem.size();

  char nomeSD[24];
  proximoNomeLivre(SD, "copia_flash", nomeSD, sizeof(nomeSD));   // [ARMAZENAMENTO] nome distinto do arquivo direto

  File destino = SD.open(nomeSD, FILE_WRITE);
  if (!destino) {
    origem.close();
    return false;
  }

  uint8_t buffer[256];
  size_t bytesLidos;
  size_t totalEscrito = 0;
  while ((bytesLidos = origem.read(buffer, sizeof(buffer))) > 0) {
    totalEscrito += destino.write(buffer, bytesLidos);
  }
  origem.close();
  destino.close();
  return (totalEscrito == tamanhoOrigem);
}

// =====================================================================
// [ARMAZENAMENTO] Funcoes novas
// =====================================================================

// Primeiro nome "/<prefixo>_NNN.csv" que ainda nao existe
void proximoNomeLivre(fs::FS &fs, const char* prefixo, char* saida, size_t tam) {
  for (int i = 1; i < 1000; i++) {
    snprintf(saida, tam, "/%s_%03d.csv", prefixo, i);
    if (!fs.exists(saida)) return;
  }
}

// Copia um arquivo e confere o tamanho; true se a copia esta completa
bool copiarArquivo(fs::FS &de, const char* origem, fs::FS &para, const char* destino) {
  File o = de.open(origem, "r");
  if (!o) return false;
  File d = para.open(destino, FILE_WRITE);
  if (!d) { o.close(); return false; }
  uint8_t buffer[512];
  size_t lidos, total = 0, tamanho = o.size();
  while ((lidos = o.read(buffer, sizeof(buffer))) > 0) total += d.write(buffer, lidos);
  o.close();
  d.close();
  return total == tamanho;
}

// Copia para o SD os voos que ficaram na flash (/voo_NNN.csv) e so entao apaga da flash
void enviarVoosAntigosParaSD() {
  if (!flashDetectada) return;
  char nomes[20][24];
  int qtd = 0;
  File raiz = LittleFS.open("/");
  File f = raiz.openNextFile();
  while (f && qtd < 20) {
    const char* n = f.name();
    if (n[0] == '/') n++;
    if (strncmp(n, "voo_", 4) == 0) snprintf(nomes[qtd++], 24, "/%s", n);
    f = raiz.openNextFile();
  }
  raiz.close();
  for (int i = 0; i < qtd; i++) {
    char nomeSD[24];
    proximoNomeLivre(SD, "copia_flash", nomeSD, sizeof(nomeSD));
    if (copiarArquivo(LittleFS, nomes[i], SD, nomeSD)) {
      LittleFS.remove(nomes[i]);
      Serial.print("Voo antigo "); Serial.print(nomes[i]); Serial.print(" -> SD "); Serial.println(nomeSD);
    }
  }
}