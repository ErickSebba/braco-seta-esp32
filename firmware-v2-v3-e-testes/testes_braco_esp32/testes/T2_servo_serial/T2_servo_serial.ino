/*
  TESTE 2 - Os servos mexem pelo PCA9685? (sem Wi-Fi, comandos pelo Monitor Serial)
  ------------------------------------------------------------------------------------
  Separa o problema "motor nao mexe" do Wi-Fi e da pagina: se aqui funcionar, o defeito
  esta no firmware completo ou na alimentacao com Wi-Fi ligado; se nao funcionar, e
  ligacao/alimentacao dos servos.

  COMO RODAR
  - ESP32 pelo USB, fio do VIN DESCONECTADO.
  - Conversor ligado: 6 V no borne V+ do PCA9685 (MECA com multimetro no borne: ~6,0 V).
  - GND do conversor, do PCA e do ESP32 juntos.
  - Servos: fio marrom/preto na fileira GND, vermelho na V+, laranja/amarelo na PWM.
  - Deixe o braco perto da pose em pe antes do primeiro comando (o 1o pulso faz o servo pular).
  - Monitor Serial 115200, final de linha "Nova linha".

  COMANDOS
    ?          ajuda
    c N        escolhe o canal N (0 Base, 1 Ombro, 2 Cotovelo, 3 Punho, 4 Giro, 5 Garra)
    a G        manda o canal escolhido para G graus (0..180) com rampa suave
    p US       manda um pulso de US microssegundos (500..2500), sem rampa
    + / -      +5 / -5 graus no canal escolhido
    v          varredura curta no canal escolhido: 90 -> 75 -> 105 -> 90
    t          testa os canais 0..5 em sequencia com a varredura curta
    x          solta todos (sem pulso: servo fica mole)
    o 0 / o 1  OE em LOW (saidas ligadas) / HIGH (saidas desligadas)
    i          mostra o estado
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SDA_PIN  21
#define SCL_PIN  22
#define OE_PIN    4         // -1 se nao estiver ligado
#define PCA_ADDR 0x40
#define US_MIN  500         // mesmos valores do firmware completo
#define US_MAX 2500

Adafruit_PWMServoDriver pwm(PCA_ADDR, Wire);
const char* NOMES[6] = {"Base", "Ombro", "Cotovelo", "Punho", "Giro punho", "Garra"};
int canal = 0;
float angulo[16];           // ultimo angulo mandado; NAN = canal solto
bool pcaOk = false;

int grausParaUs(float g) { return (int)(US_MIN + g / 180.0f * (US_MAX - US_MIN) + 0.5f); }
const char* nome(int c) { return c < 6 ? NOMES[c] : "-"; }

void mandaGraus(int c, float g) {
  g = constrain(g, 0, 180);
  pwm.writeMicroseconds(c, grausParaUs(g));
  angulo[c] = g;
}

// Rampa de ~60 graus/s. Se o canal estava solto, vai direto (nao ha como saber onde ele esta).
void moveSuave(int c, float alvo) {
  alvo = constrain(alvo, 0, 180);
  if (isnan(angulo[c])) { mandaGraus(c, alvo); delay(400); }
  float a = angulo[c];
  while (fabsf(alvo - a) > 0.01f) {
    float passo = 1.2f;                                 // 1,2 grau a cada 20 ms = 60 graus/s
    a += constrain(alvo - a, -passo, passo);
    mandaGraus(c, a);
    delay(20);
  }
  Serial.printf("  canal %d (%s) = %.0f graus  (%d us)\n", c, nome(c), alvo, grausParaUs(alvo));
}

void varredura(int c) {
  Serial.printf("Varredura no canal %d (%s)\n", c, nome(c));
  moveSuave(c, 90); delay(300);
  moveSuave(c, 75); delay(300);
  moveSuave(c, 105); delay(300);
  moveSuave(c, 90);
}

void soltaTodos() {
  for (int i = 0; i < 16; i++) { pwm.setPWM(i, 0, 4096); angulo[i] = NAN; }
  Serial.println("Todos os canais soltos.");
}

void estado() {
  Serial.printf("PCA9685: %s | OE: %s | canal escolhido: %d (%s)\n",
                pcaOk ? "ok" : "NAO ENCONTRADO",
                OE_PIN < 0 ? "nao ligado" : (digitalRead(OE_PIN) ? "HIGH (saidas OFF)" : "LOW (saidas ON)"),
                canal, nome(canal));
  for (int i = 0; i < 6; i++) {
    if (isnan(angulo[i])) Serial.printf("  %d %-10s solto\n", i, NOMES[i]);
    else Serial.printf("  %d %-10s %.0f graus\n", i, NOMES[i], angulo[i]);
  }
}

void ajuda() {
  Serial.println("Comandos: ?  c N  a G  p US  +  -  v  t  x  o 0|1  i");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n================ TESTE 2: servos pelo Serial ================");
  if (OE_PIN >= 0) { pinMode(OE_PIN, OUTPUT); digitalWrite(OE_PIN, LOW); }
  for (int i = 0; i < 16; i++) angulo[i] = NAN;

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(PCA_ADDR);
  pcaOk = (Wire.endTransmission() == 0);
  if (!pcaOk) { Serial.println("PCA9685 nao encontrado. Volte ao Teste 1."); return; }
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  soltaTodos();
  estado();
  ajuda();
  Serial.println("Comece com:  c 5  e depois  v   (a garra e a que menos assusta)");
}

void loop() {
  if (!pcaOk || !Serial.available()) return;
  String l = Serial.readStringUntil('\n');
  l.trim();
  if (!l.length()) return;
  char cmd = l.charAt(0);
  String arg = l.substring(1); arg.trim();
  float v = arg.toFloat();

  switch (cmd) {
    case '?': ajuda(); break;
    case 'i': estado(); break;
    case 'c':
      if (v >= 0 && v <= 15) { canal = (int)v; Serial.printf("Canal %d (%s)\n", canal, nome(canal)); }
      break;
    case 'a': moveSuave(canal, v); break;
    case 'p': {
      int us = constrain((int)v, US_MIN, US_MAX);
      pwm.writeMicroseconds(canal, us);
      angulo[canal] = (us - US_MIN) * 180.0f / (US_MAX - US_MIN);
      Serial.printf("  canal %d = %d us\n", canal, us);
      break;
    }
    case '+': moveSuave(canal, (isnan(angulo[canal]) ? 90 : angulo[canal]) + 5); break;
    case '-': moveSuave(canal, (isnan(angulo[canal]) ? 90 : angulo[canal]) - 5); break;
    case 'v': varredura(canal); break;
    case 't':
      for (int c = 0; c < 6; c++) {
        varredura(c);
        delay(500);
      }
      Serial.println("Fim do teste de todos os canais. Anote quais mexeram.");
      break;
    case 'x': soltaTodos(); break;
    case 'o':
      if (OE_PIN >= 0) { digitalWrite(OE_PIN, v > 0.5f ? HIGH : LOW); estado(); }
      break;
    default: Serial.println("Comando desconhecido."); ajuda();
  }
}
