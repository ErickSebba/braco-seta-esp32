/*
  ==========================================================================================
  Braco robotico com ESP32 + PCA9685 - controle pelo celular (pagina web)
  ==========================================================================================

  O QUE ESTE CODIGO FAZ
  ---------------------
  1. O ESP32 cria uma rede Wi-Fi propria (modo Access Point). Nao precisa de roteador.
     Conecte o celular na rede "BracoESP32" (senha "braco1234") e abra http://192.168.4.1
  2. O ESP32 serve a pagina de controle (arquivo pagina.h, gerado a partir de pagina.html).
  3. A pagina conversa com o ESP32 por pedidos HTTP simples (/api/...), sem bibliotecas extras.
  4. O ESP32 comanda os 6 servos pelo PCA9685 (placa de 16 canais PWM ligada no I2C).
  5. O movimento e SUAVIZADO: cada servo anda em direcao ao alvo com velocidade maxima
     limitada (graus/s), em vez de pular direto. A logica disso esta em motion.h.
  6. Ate 10 poses podem ser salvas na memoria flash do ESP32 (continuam la depois de
     desligar) e executadas em sequencia, com pausa e repeticao.

  ARQUIVOS
  --------
  braco_esp32.ino  -> este arquivo: Wi-Fi, rotas HTTP, PCA9685, memoria de poses, loop.
  motion.h         -> logica pura de movimento (rampa de velocidade, limites, sequenciador).
                      Nao depende do Arduino, por isso foi testada no PC.
  pagina.h         -> a pagina web inteira (HTML+CSS+JS) dentro de uma string na flash.
  pagina.html      -> a mesma pagina em arquivo separado, para editar com facilidade.
                      Depois de editar, gere de novo o pagina.h (ver comentario no topo dele).

  COMO GRAVAR
  -----------
  - Arduino IDE: instale o pacote de placas "esp32 by Espressif" e a biblioteca
    "Adafruit PWM Servo Driver Library" (Gerenciador de Bibliotecas).
  - Placa: "ESP32 Dev Module". Desligue a chave da caixa antes de plugar o USB.

  LIGACAO
  -------
  ESP32 GPIO21 -> PCA9685 SDA        ESP32 GPIO22 -> PCA9685 SCL
  ESP32 3V3    -> PCA9685 VCC        ESP32 GPIO4  -> PCA9685 OE (opcional)
  GND do ESP32, do PCA9685 e do conversor: TODOS juntos.
  Conversor (saida 6,0 V, minimo 5 A) -> PCA9685 V+ (borne) e ESP32 VIN.
  Jack DC (7-12 V) -> chave -> entrada do conversor. Capacitor 470-1000 uF no V+ do PCA9685.
  Canais do PCA9685: 0 Base, 1 Ombro, 2 Cotovelo, 3 Punho, 4 Giro do punho, 5 Garra.

  ANTES DE LIGAR OS SERVOS PELA PRIMEIRA VEZ
  ------------------------------------------
  - Deixe o braco na posicao home (90 graus em tudo, garra semiaberta). O firmware assume
    que o braco comeca nessa posicao, porque um servo comum nao informa onde esta.
  - Ajuste os limites min/max da tabela CFG abaixo conforme o curso real do seu braco.
    Os valores atuais sao conservadores, NAO foram medidos no braco montado.

  O QUE AINDA NAO FOI TESTADO: compilacao com o pacote real do ESP32 e o braco funcionando.
  (Testado: logica de motion.h no PC, sintaxe deste arquivo, e a pagina no navegador.)
*/
#include <WiFi.h>                     // Wi-Fi do ESP32 (modo Access Point)
#include <WebServer.h>                // servidor HTTP simples do pacote ESP32
#include <Preferences.h>              // memoria flash nao volatil (NVS) para as poses
#include <Wire.h>                     // I2C para falar com o PCA9685
#include <Adafruit_PWMServoDriver.h>  // driver do PCA9685
#include "motion.h"                   // rampa de movimento + sequenciador de poses
#include "pagina.h"                   // pagina web embutida

// ========================================================================================
// CONFIGURACAO - e aqui que voce ajusta o braco
// ========================================================================================
const char* AP_SSID = "BracoESP32";       // nome da rede Wi-Fi criada pelo ESP32
const char* AP_PASS = "braco1234";        // senha (minimo 8 caracteres)

#define SDA_PIN 21                        // I2C SDA -> PCA9685 SDA
#define SCL_PIN 22                        // I2C SCL -> PCA9685 SCL
#define OE_PIN   4                        // pino OE do PCA9685: HIGH desliga todas as saidas.
                                          // Usado para os servos NAO pularem no boot. -1 se nao ligar.
#define PCA_ADDR 0x40                     // endereco I2C padrao do PCA9685

// Uma linha por junta:
// nome no app, canal no PCA9685, angulo min, angulo max, home, pulso em 0 grau (us),
// pulso em 180 graus (us), velocidade maxima (graus/s, a 100% no app).
// AJUSTE min/max conforme o curso mecanico real (va com calma, em passos pequenos).
// Se um servo vibrar/zumbir nos extremos, estreite o pulso (ex.: 600..2400).
const JointCfg CFG[] = {
  {"Base",        0,   0, 180,  90, 500, 2500, 90},   // MG996R - giro da cintura
  {"Ombro",       1,  30, 150,  90, 500, 2500, 60},   // MG996R (mais lento: carrega o braco todo)
  {"Cotovelo",    2,  30, 150,  90, 500, 2500, 60},   // MG996R
  {"Punho",       3,   0, 180,  90, 500, 2500, 120},  // micro 9g - inclinacao
  {"Giro punho",  4,   0, 180,  90, 500, 2500, 120},  // micro 9g - rotacao
  {"Garra",       5,  20, 110,  60, 500, 2500, 120},  // micro 9g - abre/fecha
};
const int NJ = sizeof(CFG) / sizeof(CFG[0]);   // numero de juntas (6)
const int NPOSES = 10;                         // quantas poses cabem na memoria
// ========================================================================================

Adafruit_PWMServoDriver pwm(PCA_ADDR, Wire);
WebServer server(80);
Preferences prefs;

Joint J[NJ];                 // estado de cada junta (angulo atual, alvo, ligada ou nao)
bool used[NPOSES];           // quais slots de pose estao ocupados
float poseA[NPOSES][NJ];     // angulos de cada pose salva
bool engaged = false;        // true = servos com torque
bool pcaOk = false;          // o PCA9685 respondeu no I2C?
float speedPct = 40;         // velocidade global escolhida no app (5..100 %)
Player player;               // executor da sequencia de poses
uint32_t lastTick = 0;       // ultimo passo do controle de movimento (ms)

// ========================================================================================
// SAIDAS DOS SERVOS
// ========================================================================================
// Envia ao PCA9685 o pulso correspondente ao angulo atual da junta i.
void writeJoint(int i) { pwm.writeMicroseconds(CFG[i].ch, J[i].pulseUs()); }

// Desliga o canal da junta i (pulso "full off"): o servo fica solto, sem torque.
void releaseJoint(int i) { J[i].on = false; pwm.setPWM(CFG[i].ch, 0, 4096); }

// Liga o torque de todos os servos. Cada um comeca no ultimo angulo conhecido
// (no boot, o home), por isso o braco precisa estar no home ao ligar.
void engageAll() {
  if (OE_PIN >= 0) digitalWrite(OE_PIN, LOW);          // habilita as saidas do PCA9685
  for (int i = 0; i < NJ; i++) {
    if (!J[i].on) { J[i].on = true; writeJoint(i); }
  }
  engaged = true;
}

// Botao "Soltar servos": para tudo e tira o torque (da para mover o braco com a mao).
void relaxAll() {
  player.stop();
  for (int i = 0; i < NJ; i++) { J[i].tgt = J[i].cur; releaseJoint(i); }
  engaged = false;
}

// Botao "PARAR": congela cada servo onde esta agora (alvo = posicao atual).
void stopMotion() {
  player.stop();
  for (int i = 0; i < NJ; i++) J[i].tgt = J[i].cur;
}

// Manda todas as juntas para a pose salva k (com a rampa de velocidade normal).
void goPose(int k) {
  if (k < 0 || k >= NPOSES || !used[k]) return;
  if (!engaged) engageAll();
  for (int i = 0; i < NJ; i++) J[i].setTarget(poseA[k][i]);
}

// ========================================================================================
// POSES NA MEMORIA FLASH (Preferences / NVS)
// Cada pose vira uma chave "p0".."p9" com os 6 angulos em float.
// ========================================================================================
void savePoseSlot(int k) {
  for (int i = 0; i < NJ; i++) poseA[k][i] = J[i].cur;   // salva onde o braco ESTA
  used[k] = true;
  char key[8]; snprintf(key, sizeof key, "p%d", k);
  prefs.putBytes(key, poseA[k], sizeof(float) * NJ);
}
void deletePoseSlot(int k) {
  used[k] = false;
  char key[8]; snprintf(key, sizeof key, "p%d", k);
  prefs.remove(key);
}
// Le as poses gravadas no boot. Se o numero de juntas mudar, poses antigas sao ignoradas.
void loadPoses() {
  for (int k = 0; k < NPOSES; k++) {
    char key[8]; snprintf(key, sizeof key, "p%d", k);
    used[k] = (prefs.getBytesLength(key) == sizeof(float) * NJ);
    if (used[k]) prefs.getBytes(key, poseA[k], sizeof(float) * NJ);
  }
}

// ========================================================================================
// API HTTP - a pagina chama estas rotas. Todas respondem com o estado em JSON.
//
//   GET /                 -> pagina de controle
//   GET /api/state        -> estado atual (angulos, poses, velocidade...)
//   GET /api/move?j0=120  -> novo alvo para uma ou mais juntas (j0..j5)
//   GET /api/speed?v=60   -> velocidade global em %
//   GET /api/home         -> todas as juntas para o home
//   GET /api/engage       -> liga o torque     GET /api/relax -> solta os servos
//   GET /api/stop         -> para onde esta
//   GET /api/save         -> salva a pose atual no primeiro slot livre
//   GET /api/go?i=2       -> vai para a pose 2  GET /api/del?i=2 -> apaga a pose 2
//   GET /api/play?loop=1&dwell=600 -> executa as poses em ordem (pausa em ms)
// ========================================================================================

// Monta o JSON de estado na mao (evita depender da biblioteca ArduinoJson).
void sendState() {
  String s; s.reserve(700);
  s += F("{\"pca\":"); s += pcaOk ? '1' : '0';
  s += F(",\"engaged\":"); s += engaged ? '1' : '0';
  s += F(",\"speed\":"); s += (int)speedPct;
  s += F(",\"playing\":"); s += player.playing ? '1' : '0';
  s += F(",\"cur\":"); s += player.idx;                  // pose em execucao (-1 = nenhuma)
  s += F(",\"j\":[");
  for (int i = 0; i < NJ; i++) {
    if (i) s += ',';
    s += F("{\"n\":\""); s += CFG[i].name;               // nome
    s += F("\",\"min\":"); s += (int)CFG[i].minDeg;      // limites (a pagina monta os sliders)
    s += F(",\"max\":"); s += (int)CFG[i].maxDeg;
    s += F(",\"a\":"); s += String(J[i].cur, 1);         // angulo atual
    s += F(",\"t\":"); s += String(J[i].tgt, 1);         // angulo alvo
    s += '}';
  }
  s += F("],\"poses\":[");
  bool first = true;
  for (int k = 0; k < NPOSES; k++) if (used[k]) { if (!first) s += ','; s += k; first = false; }
  s += F("]}");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", s);
}

// /api/move: aceita qualquer combinacao de j0..j5. Mexer um slider liga os servos
// automaticamente e interrompe uma sequencia em execucao.
void handleMove() {
  for (int i = 0; i < NJ; i++) {
    String name = "j" + String(i);
    if (server.hasArg(name)) {
      float a = server.arg(name).toFloat();
      if (!engaged) engageAll();
      player.stop();
      J[i].setTarget(a);                 // setTarget ja respeita min/max da junta
    }
  }
  sendState();
}

void setupRoutes() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", PAGINA); });
  server.on("/api/state", HTTP_GET, sendState);
  server.on("/api/move", HTTP_GET, handleMove);
  server.on("/api/speed", HTTP_GET, []() {
    float v = server.arg("v").toFloat();
    speedPct = v < 5 ? 5 : (v > 100 ? 100 : v);
    sendState();
  });
  server.on("/api/home", HTTP_GET, []() {
    player.stop();
    if (!engaged) engageAll();
    for (int i = 0; i < NJ; i++) J[i].setTarget(CFG[i].homeDeg);
    sendState();
  });
  server.on("/api/engage", HTTP_GET, []() { engageAll(); sendState(); });
  server.on("/api/relax", HTTP_GET, []() { relaxAll(); sendState(); });
  server.on("/api/stop", HTTP_GET, []() { stopMotion(); sendState(); });
  server.on("/api/save", HTTP_GET, []() {
    int k = -1;
    for (int i = 0; i < NPOSES; i++) if (!used[i]) { k = i; break; }
    if (k >= 0) savePoseSlot(k);         // se a memoria estiver cheia, nao salva
    sendState();
  });
  server.on("/api/go", HTTP_GET, []() { player.stop(); goPose(server.arg("i").toInt()); sendState(); });
  server.on("/api/del", HTTP_GET, []() {
    int k = server.arg("i").toInt();
    if (k >= 0 && k < NPOSES) deletePoseSlot(k);
    sendState();
  });
  server.on("/api/play", HTTP_GET, []() {
    if (!engaged) engageAll();
    uint32_t dw = (uint32_t)server.arg("dwell").toInt();
    if (dw > 10000) dw = 10000;          // pausa maxima de 10 s entre poses
    player.start(server.arg("loop").toInt() == 1, dw);
    sendState();
  });
  server.onNotFound([]() { server.send(404, "text/plain", "nao encontrado"); });
}

// ========================================================================================
// SETUP: roda uma vez ao ligar
// ========================================================================================
void setup() {
  Serial.begin(115200);

  // Saidas do PCA9685 desligadas ate o usuario apertar "Ligar servos" ou mexer num slider.
  if (OE_PIN >= 0) { pinMode(OE_PIN, OUTPUT); digitalWrite(OE_PIN, HIGH); }

  // Procura o PCA9685 no I2C. Se nao achar, a pagina mostra um aviso.
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(PCA_ADDR);
  pcaOk = (Wire.endTransmission() == 0);
  if (pcaOk) {
    pwm.begin();
    Wire.setClock(400000);                      // I2C rapido (400 kHz)
    pwm.setOscillatorFrequency(27000000);       // calibracao tipica do oscilador do PCA9685
    pwm.setPWMFreq(50);                         // servos usam 50 Hz (periodo de 20 ms)
    for (int i = 0; i < 16; i++) pwm.setPWM(i, 0, 4096);   // todos os canais desligados
  } else {
    Serial.println("PCA9685 NAO encontrado - confira SDA/SCL/alimentacao");
  }

  for (int i = 0; i < NJ; i++) J[i].init(CFG[i]);   // todas as juntas comecam no home
  prefs.begin("braco", false);
  loadPoses();

  // Rede Wi-Fi propria. O IP do ESP32 nessa rede e sempre 192.168.4.1
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Wi-Fi: "); Serial.print(AP_SSID);
  Serial.print("  IP: "); Serial.println(WiFi.softAPIP());

  setupRoutes();
  server.begin();
  lastTick = millis();
}

// ========================================================================================
// LOOP: atende a pagina e, a cada 20 ms (50 Hz), avanca o movimento dos servos
// ========================================================================================
void loop() {
  server.handleClient();                         // responde aos pedidos do celular

  uint32_t now = millis();
  if (now - lastTick >= 20) {
    float dt = (now - lastTick) / 1000.0f;       // tempo desde o ultimo passo (s)
    lastTick = now;
    float scale = speedPct / 100.0f;
    bool all = true;                             // todos chegaram ao alvo?
    for (int i = 0; i < NJ; i++) {
      if (J[i].step(dt, scale) && pcaOk) writeJoint(i);   // so escreve se o angulo mudou
      if (J[i].on && !J[i].atTarget()) all = false;
    }
    // Sequencia de poses: o Player decide quando ir para a proxima pose.
    int g = player.update(now, all, used, NPOSES);
    if (g >= 0) goPose(g);
  }
}
