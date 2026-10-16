/*
  motion.h - logica de movimento do braco, SEM dependencia do Arduino.

  O QUE TEM AQUI
  - JointCfg : configuracao fixa de cada junta (limites, home, pulsos, velocidade).
  - Joint    : estado de uma junta. A cada passo de 20 ms, step() aproxima o angulo atual
               do alvo sem passar da velocidade maxima -> movimento suave, sem trancos.
               pulseUs() converte o angulo (0..180) no pulso do servo em microssegundos.
  - Player   : executa as poses salvas em ordem. So avanca para a proxima pose quando
               todos os servos chegaram ao alvo e a pausa (dwell) terminou. Pode repetir.

  Por nao usar nada do Arduino, este arquivo foi compilado e testado no PC (g++), com
  testes da rampa de velocidade, dos limites, da conversao de pulso e da sequencia.
*/
#pragma once
#include <math.h>
#include <stdint.h>

struct JointCfg {
  const char* name;   // nome mostrado no app
  uint8_t ch;         // canal no PCA9685 (0..15)
  float minDeg;       // limite mecanico minimo (graus)
  float maxDeg;       // limite mecanico maximo (graus)
  float homeDeg;      // posicao de repouso
  uint16_t usMin;     // pulso em 0 grau (microssegundos)
  uint16_t usMax;     // pulso em 180 graus
  float maxDps;       // velocidade maxima (graus/s) a 100%
};

struct Joint {
  JointCfg cfg;
  float cur = 0, tgt = 0;
  bool on = false;    // false = canal desligado (servo "solto")

  void init(const JointCfg& c) { cfg = c; cur = tgt = c.homeDeg; on = false; }

  float clampDeg(float a) const {
    if (a < cfg.minDeg) return cfg.minDeg;
    if (a > cfg.maxDeg) return cfg.maxDeg;
    return a;
  }
  void setTarget(float a) { tgt = clampDeg(a); }

  // Avanca um passo de dt segundos. speedScale: 0.05..1.0. Retorna true se mexeu.
  bool step(float dt, float speedScale) {
    if (!on) return false;
    float maxStep = cfg.maxDps * speedScale * dt;
    float d = tgt - cur;
    if (fabsf(d) < 1e-4f) return false;
    if (fabsf(d) <= maxStep) cur = tgt;
    else cur += (d > 0 ? maxStep : -maxStep);
    return true;
  }

  uint16_t pulseUs() const {
    float a = cur < 0 ? 0 : (cur > 180 ? 180 : cur);
    return (uint16_t)(cfg.usMin + (a / 180.0f) * (float)(cfg.usMax - cfg.usMin) + 0.5f);
  }
  bool atTarget(float tol = 0.4f) const { return fabsf(tgt - cur) <= tol; }
};

// Proximo slot usado a partir de 'from' (circular). Retorna -1 se nao houver nenhum.
inline int nextUsedSlot(const bool* used, int n, int from) {
  for (int k = 0; k < n; k++) {
    int i = (from + k) % n;
    if (used[i]) return i;
  }
  return -1;
}

// Sequenciador de poses: vai de pose em pose, esperando chegar + pausa.
struct Player {
  bool playing = false, loop = false, waiting = false;
  int idx = -1;                 // pose atual
  uint32_t dwellMs = 600;
  uint32_t waitUntil = 0;

  // Chame em cada ciclo. Devolve o indice da pose para a qual se deve ir agora (ou -1).
  // allAtTarget: todos os servos chegaram ao alvo. used[]: slots salvos.
  int update(uint32_t nowMs, bool allAtTarget, const bool* used, int n) {
    if (!playing) return -1;
    if (idx < 0) {                       // inicio
      int i = nextUsedSlot(used, n, 0);
      if (i < 0) { playing = false; return -1; }
      idx = i; waiting = false;
      return idx;
    }
    if (!waiting) {
      if (allAtTarget) { waiting = true; waitUntil = nowMs + dwellMs; }
      return -1;
    }
    if ((int32_t)(nowMs - waitUntil) < 0) return -1;
    // proxima pose
    int nxt = nextUsedSlot(used, n, (idx + 1) % n);
    bool wrapped = (nxt <= idx);
    if (nxt < 0 || (wrapped && !loop)) { playing = false; idx = -1; waiting = false; return -1; }
    idx = nxt; waiting = false;
    return idx;
  }
  void start(bool lp, uint32_t dwell) { playing = true; loop = lp; dwellMs = dwell; idx = -1; waiting = false; }
  void stop() { playing = false; idx = -1; waiting = false; }
};
