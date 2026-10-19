/*
  kin.h - cinematica do braco (direta e inversa) + calibracao. SEM dependencia do Arduino.

  SISTEMA DE COORDENADAS DO ROBO (mm)
    origem: no eixo de giro da base, na altura da MESA (fundo da caixa)
    x: para a frente (lado da garra / logo UFMT)     y: para a esquerda     z: para cima

  ANGULOS DAS JUNTAS (graus, "q")
    q0: giro da base. 0 = braco apontando para a frente, positivo = gira para a esquerda.
    q1: ombro. Angulo do braco (ombro->cotovelo) medido da horizontal. 90 = braco em pe.
    q2: cotovelo, RELATIVO ao braco. 0 = antebraco alinhado com o braco. Negativo = dobra
        para a frente/baixo.
    q3: punho (inclinacao), RELATIVO ao antebraco. 0 = garra alinhada com o antebraco.
    Direcao absoluta da garra no plano do braco: phi = q1 + q2 + q3.
    Garra apontando para baixo (pegar de cima): phi = -90.

  MEDIDAS (tiradas do modelo do Fusion, entre os eixos dos servos)
    ombro: 13,8 mm a frente do eixo da base e 97,5 mm acima da mesa
    L1 (ombro -> cotovelo) 120,0   L2 (cotovelo -> eixo do punho) 119,7
    L3 (eixo do punho -> centro entre as pontas dos dedos) 119,2

  CALIBRACAO (servo <-> junta)
    Cada servo recebe um angulo de comando "s" (0..180). A junta real gira q:
        q = qA + k * (s - sA)
    (sA, qA) e o ponto A da calibracao e k (graus de junta por grau de servo, com sinal)
    vem do ponto B: k = (qB - qA) / (sB - sA). Sem calibrar, usa k = 1 e sA = 90.
*/
#pragma once
#include <math.h>

struct ArmModel {
  float a = 13.8f, h1 = 97.5f;          // posicao do eixo do ombro (frente, altura)
  float L1 = 120.0f, L2 = 119.7f, L3 = 119.2f;
};

struct JointCal {                        // mapeamento servo <-> junta de UMA junta
  float sA = 90, qA = 0, k = 1;
  float toQ(float s) const { return qA + k * (s - sA); }
  float toS(float q) const { return sA + (q - qA) / k; }
  // define a partir de dois pontos medidos (retorna false se os pontos forem iguais)
  bool fromTwo(float s1, float q1, float s2, float q2) {
    if (fabsf(s2 - s1) < 3.0f) return false;          // precisa de pelo menos 3 graus de diferenca
    sA = s1; qA = q1; k = (q2 - q1) / (s2 - s1);
    return true;
  }
};

struct Vec3 { float x = 0, y = 0, z = 0; };

static inline float deg2rad(float d) { return d * 0.01745329252f; }
static inline float rad2deg(float r) { return r * 57.2957795131f; }

// Cinematica direta: angulos q[0..3] -> posicao da ponta (TCP) e dos pontos do esqueleto.
// pts (opcional, 5 pontos): base no ombro, ombro, cotovelo, punho, ponta.
inline Vec3 fk(const ArmModel& m, const float q[4], Vec3* pts = nullptr, float* phiOut = nullptr) {
  float c0 = cosf(deg2rad(q[0])), s0 = sinf(deg2rad(q[0]));
  float a1 = deg2rad(q[1]), a2 = deg2rad(q[1] + q[2]), a3 = deg2rad(q[1] + q[2] + q[3]);
  // no plano do braco: r = distancia horizontal ao eixo da base, z = altura
  float rS = m.a, zS = m.h1;
  float rE = rS + m.L1 * cosf(a1), zE = zS + m.L1 * sinf(a1);
  float rP = rE + m.L2 * cosf(a2), zP = zE + m.L2 * sinf(a2);
  float rT = rP + m.L3 * cosf(a3), zT = zP + m.L3 * sinf(a3);
  auto P = [&](float r, float z) { Vec3 v; v.x = r * c0; v.y = r * s0; v.z = z; return v; };
  if (pts) { pts[0] = P(0, zS); pts[1] = P(rS, zS); pts[2] = P(rE, zE); pts[3] = P(rP, zP); pts[4] = P(rT, zT); }
  if (phiOut) *phiOut = q[1] + q[2] + q[3];
  return P(rT, zT);
}

// Distancia horizontal (mm, no plano do braco, + = para a frente) entre o eixo da junta j
// (1 = ombro, 2 = cotovelo) e o centro de massa de tudo que vem depois dela.
// Perto de zero = a gravidade quase nao puxa a junta -> a folga das engrenagens fica solta e o
// servo oscila (hunting). As massas sao relativas e aproximadas: so o sinal e a ordem de grandeza
// importam aqui.
inline float comOffset(const ArmModel& m, const float q[4], int j) {
  float a1 = deg2rad(q[1]), a2 = deg2rad(q[1] + q[2]), a3 = deg2rad(q[1] + q[2] + q[3]);
  float rS = m.a;
  float rE = rS + m.L1 * cosf(a1);
  float rP = rE + m.L2 * cosf(a2);
  const float wBraco = 1.0f, wCotovelo = 0.8f, wAnte = 0.6f, wPunho = 0.8f, wGarra = 0.6f;
  float r0 = (j == 1) ? rS : rE, sw = 0, sm = 0;
  auto add = [&](float w, float r) { sw += w; sm += w * (r - r0); };
  if (j == 1) { add(wBraco, rS + 0.5f * m.L1 * cosf(a1)); add(wCotovelo, rE); }
  add(wAnte, rE + 0.5f * m.L2 * cosf(a2));
  add(wPunho, rP);
  add(wGarra, rP + 0.5f * m.L3 * cosf(a3));
  return sm / sw;
}

// Cinematica inversa com a garra apontando para baixo (phi = -90), solucao "cotovelo para cima".
// Retorna false se o ponto estiver fora do alcance.
inline bool ikDown(const ArmModel& m, Vec3 t, float q[4]) {
  float r = sqrtf(t.x * t.x + t.y * t.y);
  if (r < 1.0f) return false;                           // em cima do eixo da base: indefinido
  q[0] = rad2deg(atan2f(t.y, t.x));
  float wr = r, wz = t.z + m.L3;                        // punho fica L3 acima da ponta
  float dx = wr - m.a, dz = wz - m.h1;
  float D = (dx * dx + dz * dz - m.L1 * m.L1 - m.L2 * m.L2) / (2 * m.L1 * m.L2);
  if (D > 1.0f || D < -1.0f) return false;              // longe ou perto demais
  float a2 = -acosf(D);                                 // cotovelo para cima (antebraco dobra para baixo)
  float a1 = atan2f(dz, dx) - atan2f(m.L2 * sinf(a2), m.L1 + m.L2 * cosf(a2));
  q[1] = rad2deg(a1);
  q[2] = rad2deg(a2);
  q[3] = -90.0f - q[1] - q[2];
  return true;
}
