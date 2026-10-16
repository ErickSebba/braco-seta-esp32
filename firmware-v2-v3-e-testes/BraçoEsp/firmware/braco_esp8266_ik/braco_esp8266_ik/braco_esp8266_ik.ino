/*
  ==========================================================================================
  Braco ESP - versao 2 PARA ESP8266 (NodeMCU): visualizacao, calibracao, cinematica e
  "pegar e levar"
  ==========================================================================================

  ESTA E A ADAPTACAO PARA ESP8266 da pasta braco_esp32_ik. A logica (kin.h, tasks.h,
  motion.h) e a pagina sao as mesmas. O que mudou:
  - Wi-Fi/servidor: ESP8266WiFi + ESP8266WebServer (no lugar de WiFi + WebServer do ESP32).
  - Memoria: calibracao e pontos ficam na EEPROM emulada (no lugar de Preferences).
  - Pinos I2C: SDA = D2 (GPIO4), SCL = D1 (GPIO5). OE do PCA9685 = D5 (GPIO14).
  - Rede Wi-Fi: "BracoESP8266".
  Quando o ESP32 chegar, use a pasta braco_esp32_ik (a calibracao precisa ser refeita la).

  LIGACAO (NodeMCU)
    D2 (GPIO4)  -> PCA9685 SDA        D1 (GPIO5) -> PCA9685 SCL
    3V3         -> PCA9685 VCC        D5 (GPIO14) -> PCA9685 OE (opcional)
    GND comum entre NodeMCU, PCA9685 e conversor.
    Conversor (6,0 V) -> PCA9685 V+  e  NodeMCU VIN.
    Nao use D3, D4 e D8 para nada: eles decidem como a placa liga.

  O QUE MUDOU EM RELACAO A VERSAO 1 (pasta braco_esp32)
  ------------------------------------------------------
  - A pagina desenha o braco em 3 vistas (lado, cima e 3/4) acompanhando os sliders.
  - CALIBRACAO guiada: voce posiciona o braco em poses conhecidas (todo em pe, base para a
    frente, ombro na horizontal...) e o ESP32 aprende a relacao entre o comando de cada servo
    e o angulo real da junta. Fica salva na memoria.
  - CINEMATICA: com a calibracao, o ESP32 sabe onde esta a ponta da garra (x, y, z em mm) e
    consegue o caminho inverso: dado um ponto, calcula os angulos com a garra apontando para
    BAIXO (pegar de cima). Arquivo kin.h.
  - MOVIMENTO EM XYZ: botoes para mover a ponta em X, Y e Z em linha reta (jog), usados para
    ENSINAR pontos: leve a garra ate o lugar e toque "Salvar ponto".
  - TAREFA "pegar e levar": escolha o ponto de pegar (A) e o de soltar (B). O braco abre a garra,
    vai por cima de A, desce reto, fecha, sobe, vai por cima de B, desce, solta e sobe.
    Arquivo tasks.h.
  - As poses por juntas da versao 1 sairam desta versao (os pontos ensinados as substituem).

  LIMITES IMPORTANTES
  -------------------
  - Servos de hobby nao informam a posicao real: o desenho e a posicao XYZ sao o que o braco
    DEVERIA estar fazendo. Se ele travar ou ceder com peso, nada percebe.
  - O giro do punho vem ANTES da inclinacao na montagem deste braco. Com a garra para baixo,
    o giro fica fixo no neutro e quem orienta os dedos e a base: os dedos sempre abrem
    perpendiculares a direcao do braco.
  - Ao ligar, o firmware ainda assume que o braco esta na pose "home" (ver versao 1).
  - Precisao esperada com MG996R: alguns mm ate ~1 cm.

  ARQUIVOS
  --------
  braco_esp32_ik.ino -> este: Wi-Fi, rotas HTTP, servos, calibracao, pontos, loop.
  motion.h           -> rampa de velocidade das juntas (igual a versao 1).
  kin.h              -> cinematica direta/inversa e mapeamento servo <-> junta (testado no PC).
  tasks.h            -> movimentos em linha reta e sequencia "pegar e levar" (testado no PC).
  pagina.h / .html   -> pagina de controle com as 3 vistas.

  GRAVACAO: placa "NodeMCU 1.0 (ESP-12E Module)" no Arduino IDE.
  Bibliotecas: pacote "esp8266 by ESP8266 Community" + "Adafruit PWM Servo Driver Library".
*/
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "motion.h"
#include "kin.h"
#include "tasks.h"
#include "pagina.h"

// ========================================================================================
// CONFIGURACAO
// ========================================================================================
const char* AP_SSID = "BracoESP8266";
const char* AP_PASS = "braco1234";
#define SDA_PIN 4      // D2
#define SCL_PIN 5      // D1
#define OE_PIN  14     // D5 (-1 se nao ligar o OE)
#define PCA_ADDR 0x40

// Juntas: nome, canal PCA9685, min, max (graus de SERVO), home, pulso 0 grau, pulso 180, graus/s.
// Na versao 2 os limites ficam abertos (0..180) para a calibracao conseguir alcancar as poses
// de referencia. Depois de calibrar, aperte-os se alguma junta bater em algo.
const JointCfg CFG[] = {
  {"Base",        0,   0, 180,  90, 500, 2500, 90},
  {"Ombro",       1,   0, 180,  90, 500, 2500, 60},
  {"Cotovelo",    2,   0, 180,  90, 500, 2500, 60},
  {"Punho",       3,   0, 180,  90, 500, 2500, 120},
  {"Giro punho",  4,   0, 180,  90, 500, 2500, 120},
  {"Garra",       5,   0, 180,  60, 500, 2500, 120},
};
const int NJ = 6;
enum { J_BASE = 0, J_OMBRO, J_COTOVELO, J_PUNHO, J_GIRO, J_GARRA };

ArmModel MODEL;                // medidas do braco (kin.h), tiradas do Fusion

// Caixa da base no sistema do robo (mm) - usada para nao deixar a garra descer em cima dela
const float BOX_X0 = -70, BOX_X1 = 84, BOX_Y = 56, BOX_H = 56, BOX_MARGEM = 15;
const int NPTS = 12;           // quantos pontos ensinados cabem na memoria
// ========================================================================================

Adafruit_PWMServoDriver pwm(PCA_ADDR, Wire);
ESP8266WebServer server(80);

// EEPROM (emulada na flash): [assinatura][calibracao][pontos]
const uint32_t EE_MAGIC = 0x5E7A0002;
const int EE_CAL = 4, EE_PTS = 128, EE_SIZE = 1024;

Joint J[NJ];
bool engaged = false, pcaOk = false;
float speedPct = 40;
uint32_t lastTick = 0;
String lastErr = "";

// ---------------------------- calibracao ----------------------------
// Valores de servo registrados em cada passo do assistente. NAN = ainda nao registrado.
struct CalRaw {
  float vert[4];        // passo 1: braco todo em pe -> ombro, cotovelo, punho e giro (neutro)
  float baseFrente;     // passo 2: base apontando para a frente        (q0 = 0)
  float baseEsq;        // passo 3: base apontando para a esquerda      (q0 = +90)
  float ombroHoriz;     // passo 4: braco (ombro->cotovelo) na horizontal (q1 = 0)
  float cotov90;        // passo 5: antebraco dobrado 90 graus para a frente (q2 = -90)
  float punho90;        // passo 6: garra dobrada 90 graus para a frente (q3 = -90)
  float garraAberta;    // passo 7
  float garraFechada;   // passo 8
} calRaw;
JointCal cal[4];        // mapeamento calculado para base, ombro, cotovelo, punho
bool calOk[4] = {false, false, false, false};
float giroNeutro = 90, garraAberta = 60, garraFechada = 110;

void calCompute() {
  calOk[0] = !isnan(calRaw.baseFrente) && !isnan(calRaw.baseEsq) && cal[0].fromTwo(calRaw.baseFrente, 0, calRaw.baseEsq, 90);
  calOk[1] = !isnan(calRaw.vert[0]) && !isnan(calRaw.ombroHoriz) && cal[1].fromTwo(calRaw.vert[0], 90, calRaw.ombroHoriz, 0);
  calOk[2] = !isnan(calRaw.vert[1]) && !isnan(calRaw.cotov90) && cal[2].fromTwo(calRaw.vert[1], 0, calRaw.cotov90, -90);
  calOk[3] = !isnan(calRaw.vert[2]) && !isnan(calRaw.punho90) && cal[3].fromTwo(calRaw.vert[2], 0, calRaw.punho90, -90);
  // sem calibracao: palpite (servo 90 = pose em pe, sinal positivo) so para o desenho funcionar
  if (!calOk[0]) { cal[0].sA = 90; cal[0].qA = 0;  cal[0].k = 1; }
  if (!calOk[1]) { cal[1].sA = 90; cal[1].qA = 90; cal[1].k = 1; }
  if (!calOk[2]) { cal[2].sA = 90; cal[2].qA = 0;  cal[2].k = 1; }
  if (!calOk[3]) { cal[3].sA = 90; cal[3].qA = 0;  cal[3].k = 1; }
  if (!isnan(calRaw.vert[3])) giroNeutro = calRaw.vert[3];
  if (!isnan(calRaw.garraAberta)) garraAberta = calRaw.garraAberta;
  if (!isnan(calRaw.garraFechada)) garraFechada = calRaw.garraFechada;
}
bool calibrado() { return calOk[0] && calOk[1] && calOk[2] && calOk[3]; }
void calReset() {
  for (int i = 0; i < 4; i++) calRaw.vert[i] = NAN;
  calRaw.baseFrente = calRaw.baseEsq = calRaw.ombroHoriz = calRaw.cotov90 = calRaw.punho90 = NAN;
  calRaw.garraAberta = calRaw.garraFechada = NAN;
}
void calSave() { EEPROM.put(0, EE_MAGIC); EEPROM.put(EE_CAL, calRaw); EEPROM.commit(); }
void calLoad() {
  calReset();
  uint32_t m = 0; EEPROM.get(0, m);
  if (m == EE_MAGIC) EEPROM.get(EE_CAL, calRaw);
  calCompute();
}

// ---------------------------- pontos ensinados ----------------------------
struct Ponto { char nome[16]; float x, y, z; bool usado; };
Ponto pts[NPTS];
void ptsSave() { EEPROM.put(0, EE_MAGIC); EEPROM.put(EE_CAL, calRaw); EEPROM.put(EE_PTS, pts); EEPROM.commit(); }
void ptsLoad() {
  memset(pts, 0, sizeof(pts));
  uint32_t m = 0; EEPROM.get(0, m);
  if (m == EE_MAGIC) EEPROM.get(EE_PTS, pts);
}

// ---------------------------- saidas dos servos ----------------------------
void writeJoint(int i) { pwm.writeMicroseconds(CFG[i].ch, J[i].pulseUs()); }
void releaseJoint(int i) { J[i].on = false; pwm.setPWM(CFG[i].ch, 0, 4096); }
Sequencer seq;

void engageAll() {
  if (OE_PIN >= 0) digitalWrite(OE_PIN, LOW);
  for (int i = 0; i < NJ; i++) if (!J[i].on) { J[i].on = true; writeJoint(i); }
  engaged = true;
}
void stopAll() { seq.stop(); for (int i = 0; i < NJ; i++) J[i].tgt = J[i].cur; }
void relaxAll() { stopAll(); for (int i = 0; i < NJ; i++) releaseJoint(i); engaged = false; }

// ---------------------------- cinematica com o braco real ----------------------------
void qAtual(float q[4]) { for (int i = 0; i < 4; i++) q[i] = cal[i].toQ(J[i].cur); }

// Implementacao do ArmIface (tasks.h) usando os servos de verdade.
struct BracoReal : ArmIface {
  bool atTarget() override {
    for (int i = 0; i < NJ; i++) if (J[i].on && !J[i].atTarget(0.6f)) return false;
    return true;
  }
  Vec3 tcp() override { float q[4]; qAtual(q); return fk(MODEL, q); }
  bool moveTcp(const Vec3& p) override {
    if (!calibrado()) { lastErr = "calibre o braco antes de mover em XYZ"; return false; }
    if (p.z < 0) { lastErr = "ponto abaixo da mesa"; return false; }
    if (p.x > BOX_X0 - BOX_MARGEM && p.x < BOX_X1 + BOX_MARGEM && fabsf(p.y) < BOX_Y + BOX_MARGEM && p.z < BOX_H + BOX_MARGEM) {
      lastErr = "ponto em cima da caixa da base"; return false;
    }
    float q[4];
    if (!ikDown(MODEL, p, q)) { lastErr = "ponto fora do alcance do braco"; return false; }
    float s[4];
    for (int i = 0; i < 4; i++) {
      s[i] = cal[i].toS(q[i]);
      if (s[i] < CFG[i].minDeg || s[i] > CFG[i].maxDeg) {
        lastErr = String("limite do servo: ") + CFG[i].name + " precisaria de " + String(s[i], 0) + " graus";
        return false;
      }
    }
    if (!engaged) engageAll();
    for (int i = 0; i < 4; i++) J[i].setTarget(s[i]);
    J[J_GIRO].setTarget(giroNeutro);                       // com a garra para baixo, giro no neutro
    return true;
  }
  void setGrip(bool fechada) override { if (!engaged) engageAll(); J[J_GARRA].setTarget(fechada ? garraFechada : garraAberta); }
} braco;

float velMmS() { return speedPct * 1.2f; }                  // 100% = 120 mm/s

// Inicia uma sequencia ja montada em "seq"
void iniciaSeq() { lastErr = ""; if (!engaged) engageAll(); seq.start(braco, millis(), velMmS()); if (seq.error) lastErr = seq.error; }

// ========================================================================================
// API HTTP
//   /api/state, /api/model                         estado e medidas/calibracao (JSON)
//   /api/move?j0=..j5=   /api/speed?v=   /api/home   /api/engage   /api/relax   /api/stop
//   /api/jog?dx=&dy=&dz=  move a ponta em linha reta (mm, relativo)
//   /api/jogb?d=          gira a base d graus mantendo distancia e altura
//   /api/movel?x=&y=&z=   vai em linha reta ate um ponto absoluto
//   /api/grip?c=1|0       fecha / abre a garra
//   /api/pt/save?nome=    salva a posicao atual da ponta como ponto ensinado
//   /api/pt/del?i=   /api/pt/go?i=&h=   (go: vai por cima do ponto e desce reto)
//   /api/pick?a=&b=&h=    pegar no ponto a e levar ao ponto b (h = altura de aproximacao)
//   /api/cal/mark?p=1..8  registra o passo p da calibracao   /api/cal/reset
// ========================================================================================
void addF(String& s, const char* k, float v, int d = 1) { s += '"'; s += k; s += "\":"; s += isnan(v) ? String("null") : String(v, d); }

void sendState() {
  float q[4]; qAtual(q); float phi; Vec3 t = fk(MODEL, q, nullptr, &phi);
  String s; s.reserve(1600);
  s += "{\"pca\":"; s += pcaOk ? '1' : '0';
  s += ",\"engaged\":"; s += engaged ? '1' : '0';
  s += ",\"speed\":"; s += (int)speedPct;
  s += ",\"cal\":"; s += calibrado() ? '1' : '0';
  s += ",\"calOk\":["; for (int i = 0; i < 4; i++) { if (i) s += ','; s += calOk[i] ? '1' : '0'; } s += ']';
  s += ",\"tcp\":{"; addF(s, "x", t.x); s += ','; addF(s, "y", t.y); s += ','; addF(s, "z", t.z); s += ','; addF(s, "phi", phi); s += '}';
  s += ",\"q\":["; for (int i = 0; i < 4; i++) { if (i) s += ','; s += String(q[i], 1); } s += ']';
  s += ",\"seq\":{\"run\":"; s += seq.running ? '1' : '0'; s += ",\"i\":"; s += seq.cur; s += ",\"n\":"; s += seq.n; s += '}';
  s += ",\"err\":\""; s += lastErr; s += '"';
  s += ",\"j\":[";
  for (int i = 0; i < NJ; i++) {
    if (i) s += ',';
    s += "{\"n\":\""; s += CFG[i].name; s += "\",\"min\":"; s += (int)CFG[i].minDeg; s += ",\"max\":"; s += (int)CFG[i].maxDeg;
    s += ",\"a\":"; s += String(J[i].cur, 1); s += ",\"t\":"; s += String(J[i].tgt, 1); s += '}';
  }
  s += "],\"pts\":[";
  bool first = true;
  for (int i = 0; i < NPTS; i++) if (pts[i].usado) {
    if (!first) s += ',';
    first = false;
    s += "{\"i\":"; s += i; s += ",\"n\":\""; s += pts[i].nome; s += "\",";
    addF(s, "x", pts[i].x); s += ','; addF(s, "y", pts[i].y); s += ','; addF(s, "z", pts[i].z); s += '}';
  }
  s += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", s);
}

void sendModel() {
  String s; s.reserve(900);
  s += "{"; addF(s, "a", MODEL.a); s += ','; addF(s, "h1", MODEL.h1); s += ',';
  addF(s, "L1", MODEL.L1); s += ','; addF(s, "L2", MODEL.L2); s += ','; addF(s, "L3", MODEL.L3);
  s += ",\"box\":["; s += String(BOX_X0, 0); s += ','; s += String(BOX_X1, 0); s += ','; s += String(BOX_Y, 0); s += ','; s += String(BOX_H, 0); s += ']';
  s += ",\"cal\":[";
  for (int i = 0; i < 4; i++) { if (i) s += ','; s += '{'; addF(s, "sA", cal[i].sA, 2); s += ','; addF(s, "qA", cal[i].qA, 2); s += ','; addF(s, "k", cal[i].k, 4); s += ",\"ok\":"; s += calOk[i] ? '1' : '0'; s += '}'; }
  s += "],\"raw\":[";
  float r[] = {calRaw.vert[0], calRaw.baseFrente, calRaw.baseEsq, calRaw.ombroHoriz, calRaw.cotov90, calRaw.punho90, calRaw.garraAberta, calRaw.garraFechada};
  for (int i = 0; i < 8; i++) { if (i) s += ','; s += isnan(r[i]) ? String("null") : String(r[i], 1); }
  s += "]";
  s += ','; addF(s, "giro", giroNeutro); s += ','; addF(s, "gA", garraAberta); s += ','; addF(s, "gF", garraFechada);
  s += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", s);
}

float argF(const char* k, float def = 0) { return server.hasArg(k) ? server.arg(k).toFloat() : def; }

void handleMove() {
  for (int i = 0; i < NJ; i++) {
    String k = "j" + String(i);
    if (server.hasArg(k)) { if (!engaged) engageAll(); seq.stop(); J[i].setTarget(server.arg(k).toFloat()); }
  }
  sendState();
}

// move em linha reta ate p (uma sequencia de 1 passo)
void linha(Vec3 p) { seq.clear(); seq.add({ST_LIN, p, false, 0}); iniciaSeq(); }

void setupRoutes() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", PAGINA); });
  server.on("/api/state", HTTP_GET, sendState);
  server.on("/api/model", HTTP_GET, sendModel);
  server.on("/api/move", HTTP_GET, handleMove);
  server.on("/api/speed", HTTP_GET, []() { float v = argF("v", 40); speedPct = v < 5 ? 5 : (v > 100 ? 100 : v); sendState(); });
  server.on("/api/home", HTTP_GET, []() { seq.stop(); if (!engaged) engageAll(); for (int i = 0; i < NJ; i++) J[i].setTarget(CFG[i].homeDeg); sendState(); });
  server.on("/api/engage", HTTP_GET, []() { engageAll(); sendState(); });
  server.on("/api/relax", HTTP_GET, []() { relaxAll(); sendState(); });
  server.on("/api/stop", HTTP_GET, []() { stopAll(); sendState(); });

  server.on("/api/jog", HTTP_GET, []() {
    Vec3 t = braco.tcp();
    linha(v3(t.x + argF("dx"), t.y + argF("dy"), t.z + argF("dz")));
    sendState();
  });
  server.on("/api/jogb", HTTP_GET, []() {      // gira a base mantendo raio e altura
    Vec3 t = braco.tcp(); float r = sqrtf(t.x * t.x + t.y * t.y), a = atan2f(t.y, t.x) + deg2rad(argF("d"));
    seq.clear(); seq.add({ST_JNT, v3(r * cosf(a), r * sinf(a), t.z), false, 0}); iniciaSeq();
    sendState();
  });
  server.on("/api/movel", HTTP_GET, []() { linha(v3(argF("x"), argF("y"), argF("z"))); sendState(); });
  server.on("/api/grip", HTTP_GET, []() { braco.setGrip(argF("c") > 0.5f); sendState(); });

  server.on("/api/pt/save", HTTP_GET, []() {
    int k = -1; for (int i = 0; i < NPTS; i++) if (!pts[i].usado) { k = i; break; }
    if (k < 0) lastErr = "memoria de pontos cheia";
    else if (!calibrado()) lastErr = "calibre antes de ensinar pontos";
    else {
      Vec3 t = braco.tcp();
      String nome = server.hasArg("nome") && server.arg("nome").length() ? server.arg("nome") : String("P") + String(k + 1);
      strncpy(pts[k].nome, nome.c_str(), 15); pts[k].nome[15] = 0;
      for (char* c = pts[k].nome; *c; c++) if (*c == '"' || *c == '\\') *c = '_';   // nao quebrar o JSON
      pts[k].x = t.x; pts[k].y = t.y; pts[k].z = t.z; pts[k].usado = true; ptsSave(); lastErr = "";
    }
    sendState();
  });
  server.on("/api/pt/del", HTTP_GET, []() { int k = (int)argF("i", -1); if (k >= 0 && k < NPTS) { pts[k].usado = false; ptsSave(); } sendState(); });
  server.on("/api/pt/go", HTTP_GET, []() {
    int k = (int)argF("i", -1); float h = argF("h", 40);
    if (k >= 0 && k < NPTS && pts[k].usado) {
      Vec3 p = v3(pts[k].x, pts[k].y, pts[k].z);
      seq.clear(); seq.add({ST_JNT, v3(p.x, p.y, p.z + h), false, 0}); seq.add({ST_LIN, p, false, 0}); iniciaSeq();
    }
    sendState();
  });
  server.on("/api/pick", HTTP_GET, []() {
    int a = (int)argF("a", -1), b = (int)argF("b", -1); float h = argF("h", 40);
    if (a < 0 || b < 0 || a >= NPTS || b >= NPTS || !pts[a].usado || !pts[b].usado) lastErr = "escolha os pontos A e B";
    else {
      buildPickPlace(seq, v3(pts[a].x, pts[a].y, pts[a].z), v3(pts[b].x, pts[b].y, pts[b].z), h, 500);
      iniciaSeq();
    }
    sendState();
  });

  server.on("/api/cal/mark", HTTP_GET, []() {
    int p = (int)argF("p", 0);
    switch (p) {
      case 1: for (int i = 0; i < 3; i++) calRaw.vert[i] = J[J_OMBRO + i].cur; calRaw.vert[3] = J[J_GIRO].cur; break;
      case 2: calRaw.baseFrente = J[J_BASE].cur; break;
      case 3: calRaw.baseEsq = J[J_BASE].cur; break;
      case 4: calRaw.ombroHoriz = J[J_OMBRO].cur; break;
      case 5: calRaw.cotov90 = J[J_COTOVELO].cur; break;
      case 6: calRaw.punho90 = J[J_PUNHO].cur; break;
      case 7: calRaw.garraAberta = J[J_GARRA].cur; break;
      case 8: calRaw.garraFechada = J[J_GARRA].cur; break;
    }
    calCompute(); calSave(); sendModel();
  });
  server.on("/api/cal/reset", HTTP_GET, []() { calReset(); calCompute(); calSave(); sendModel(); });
  server.onNotFound([]() { server.send(404, "text/plain", "nao encontrado"); });
}

// ========================================================================================
// SETUP / LOOP
// ========================================================================================
void setup() {
  Serial.begin(115200);
  if (OE_PIN >= 0) { pinMode(OE_PIN, OUTPUT); digitalWrite(OE_PIN, HIGH); }
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(PCA_ADDR);
  pcaOk = (Wire.endTransmission() == 0);
  if (pcaOk) {
    pwm.begin(); Wire.setClock(400000); pwm.setOscillatorFrequency(27000000); pwm.setPWMFreq(50);
    for (int i = 0; i < 16; i++) pwm.setPWM(i, 0, 4096);
  } else Serial.println("PCA9685 NAO encontrado - confira SDA/SCL/alimentacao");

  for (int i = 0; i < NJ; i++) J[i].init(CFG[i]);
  EEPROM.begin(EE_SIZE);
  static_assert(EE_CAL + sizeof(CalRaw) <= EE_PTS, "calibracao nao cabe antes dos pontos");
  static_assert(EE_PTS + sizeof(pts) <= EE_SIZE, "pontos nao cabem na EEPROM");
  calLoad();
  ptsLoad();

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Wi-Fi: "); Serial.print(AP_SSID); Serial.print("  IP: "); Serial.println(WiFi.softAPIP());
  setupRoutes();
  server.begin();
  lastTick = millis();
}

void loop() {
  server.handleClient();
  uint32_t now = millis();
  if (now - lastTick >= 20) {                          // 50 Hz
    float dt = (now - lastTick) / 1000.0f; lastTick = now;
    seq.update(braco, now, dt, velMmS());              // novos alvos (linha reta / tarefa)
    if (seq.error && lastErr.length() == 0) lastErr = seq.error;
    // Em linha reta a velocidade ja vem do proprio caminho, entao as juntas podem ir no maximo.
    // Nos outros casos (sliders, deslocamentos JNT) vale a velocidade escolhida no app.
    bool emLinha = seq.running && seq.cur >= 0 && seq.steps[seq.cur].type == ST_LIN;
    float scale = emLinha ? 1.0f : speedPct / 100.0f;
    for (int i = 0; i < NJ; i++) if (J[i].step(dt, scale) && pcaOk) writeJoint(i);
  }
}
