/*
  tasks.h - movimentos no espaco (XYZ) e a tarefa "pegar e levar". SEM dependencia do Arduino.

  Aqui ficam:
  - ArmIface: o que o sequenciador precisa do braco (implementado no .ino e no teste do PC).
  - Sequencer: executa uma lista de passos, um de cada vez:
        LIN   - vai em LINHA RETA ate um ponto, mantendo a garra para baixo
                (o ponto e recalculado pela cinematica inversa a cada 20 ms)
        JNT   - vai ate um ponto pelo caminho mais curto das juntas (usado nos deslocamentos
                longos, mais seguro que linha reta entre pontos distantes)
        GRIP  - abre ou fecha a garra e espera o tempo indicado
        WAIT  - so espera
  - buildPickPlace(): monta a sequencia "pegar em A e levar para B".
*/
#pragma once
#include <stdint.h>
#include <math.h>
#include "kin.h"

struct ArmIface {
  virtual bool atTarget() = 0;                 // todas as juntas chegaram ao alvo?
  virtual Vec3 tcp() = 0;                      // posicao atual da ponta (cinematica direta)
  virtual bool moveTcp(const Vec3& p) = 0;     // IK + novos alvos das juntas (false = fora do alcance)
  virtual void setGrip(bool closed) = 0;       // garra fechada/aberta
  virtual ~ArmIface() {}
};

enum StepType : uint8_t { ST_LIN, ST_JNT, ST_GRIP, ST_WAIT };
struct Step { StepType type; Vec3 p; bool closed; uint32_t ms; };

static inline Vec3 v3(float x, float y, float z) { Vec3 v; v.x = x; v.y = y; v.z = z; return v; }
static inline float dist3(const Vec3& a, const Vec3& b) {
  float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z; return sqrtf(dx * dx + dy * dy + dz * dz);
}

struct Sequencer {
  static const int MAXS = 16;
  Step steps[MAXS];
  int n = 0, cur = -1;
  bool running = false;
  const char* error = nullptr;                 // motivo da parada, se houve erro
  // estado do passo atual
  Vec3 p0, p1; float T = 0, t = 0; uint32_t waitUntil = 0; bool phaseDone = false;

  void clear() { n = 0; cur = -1; running = false; error = nullptr; }
  bool add(const Step& s) { if (n >= MAXS) return false; steps[n++] = s; return true; }
  void start(ArmIface& arm, uint32_t now, float speedMmS) {
    if (n == 0) return;
    running = true; error = nullptr; cur = 0;
    beginStep(arm, now, speedMmS);
  }
  void stop() { running = false; cur = -1; }

  // velocidade em mm/s para os passos LIN
  void update(ArmIface& arm, uint32_t now, float dt, float speedMmS) {
    if (!running) return;
    Step& s = steps[cur];
    switch (s.type) {
      case ST_LIN: {
        if (t < T) {
          t += dt; float u = t >= T ? 1.0f : t / T;
          u = u * u * (3 - 2 * u);             // acelera e desacelera suave (smoothstep)
          Vec3 p = v3(p0.x + (p1.x - p0.x) * u, p0.y + (p1.y - p0.y) * u, p0.z + (p1.z - p0.z) * u);
          if (!arm.moveTcp(p)) { fail("ponto fora do alcance no caminho"); return; }
        } else if (arm.atTarget()) advance(&arm, now, speedMmS);
        break;
      }
      case ST_JNT:
        if (arm.atTarget()) advance(&arm, now, speedMmS);
        break;
      case ST_GRIP:
      case ST_WAIT:
        if ((int32_t)(now - waitUntil) >= 0 && arm.atTarget()) advance(&arm, now, speedMmS);
        break;
    }
  }

  void fail(const char* why) { error = why; running = false; }

  // inicia o proximo passo (ou termina)
  void advance(ArmIface* arm, uint32_t now, float speedMmS) {
    cur++;
    if (cur >= n) { running = false; cur = -1; return; }
    beginStep(*arm, now, speedMmS);
  }

  void beginStep(ArmIface& arm, uint32_t now, float speedMmS) {
    Step& s = steps[cur];
    if (s.type == ST_LIN) {
      p0 = arm.tcp(); p1 = s.p; t = 0;
      T = dist3(p0, p1) / (speedMmS < 5 ? 5 : speedMmS);
      if (T < 0.05f) T = 0.05f;
    } else if (s.type == ST_JNT) {
      if (!arm.moveTcp(s.p)) fail("ponto fora do alcance");
    } else if (s.type == ST_GRIP) {
      arm.setGrip(s.closed); waitUntil = now + s.ms;
    } else {
      waitUntil = now + s.ms;
    }
  }
};

// Monta "pegar em A e levar para B". h = altura de aproximacao acima dos pontos (mm).
inline void buildPickPlace(Sequencer& sq, Vec3 A, Vec3 B, float h, uint32_t gripMs) {
  sq.clear();
  Vec3 Au = v3(A.x, A.y, A.z + h), Bu = v3(B.x, B.y, B.z + h);
  sq.add({ST_GRIP, Vec3(), false, gripMs});   // 1. abre a garra
  sq.add({ST_JNT, Au, false, 0});             // 2. vai para cima do objeto
  sq.add({ST_LIN, A, false, 0});              // 3. desce reto
  sq.add({ST_GRIP, Vec3(), true, gripMs});    // 4. fecha a garra
  sq.add({ST_LIN, Au, false, 0});             // 5. sobe reto
  sq.add({ST_JNT, Bu, false, 0});             // 6. vai para cima do destino
  sq.add({ST_LIN, B, false, 0});              // 7. desce reto
  sq.add({ST_GRIP, Vec3(), false, gripMs});   // 8. solta
  sq.add({ST_LIN, Bu, false, 0});             // 9. sobe reto
}
