/*
  TESTE 1 - O ESP32 enxerga o PCA9685?
  ------------------------------------------------------------------
  Nao usa Wi-Fi nem a biblioteca da Adafruit: fala direto com o chip pelo I2C.

  COMO RODAR
  - ESP32 alimentado SO pelo USB (fio do VIN desconectado).
  - O conversor de 6 V pode ficar desligado neste teste (so a logica do PCA e testada).
  - PCA9685: VCC no 3V3 do ESP32, GND comum, SDA no GPIO21, SCL no GPIO22.
  - Monitor Serial em 115200, final de linha "Nova linha". Digite "s" para repetir.

  O QUE DEVE APARECER
  - "SDA=1 SCL=1" antes de iniciar o I2C (os resistores de pull-up do PCA puxam para 3,3 V).
    Se der 0, o VCC do PCA nao esta chegando ou ha um curto.
  - Dispositivos em 0x40 (o PCA) e 0x70 (endereco "all call" do mesmo chip).
  - PRESCALE lido = 121 depois de gravar 121. Isso prova que escrita e leitura funcionam.
*/
#include <Wire.h>

#define SDA_PIN  21
#define SCL_PIN  22
#define OE_PIN    4      // -1 se o OE do PCA nao estiver ligado
#define PCA_ADDR 0x40

uint8_t lerReg(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((int)PCA_ADDR, 1) != 1) return 0xFF;
  return Wire.read();
}

bool gravarReg(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

void rodarTeste() {
  Serial.println();
  Serial.println("================ TESTE 1: I2C + PCA9685 ================");

  // 1) Nivel das linhas antes do I2C: com o PCA alimentado, os pull-ups deixam ambas em 1
  Wire.end();
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  delay(5);
  int sda = digitalRead(SDA_PIN), scl = digitalRead(SCL_PIN);
  Serial.printf("Linhas em repouso: SDA=%d SCL=%d  %s\n", sda, scl,
                (sda && scl) ? "(ok)" : "(PROBLEMA: VCC do PCA, curto ou fio solto)");

  // 2) Varredura do barramento
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  int n = 0;
  bool achouPca = false;
  Serial.print("Varredura I2C: ");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", a);
      n++;
      if (a == PCA_ADDR) achouPca = true;
    }
  }
  Serial.println(n ? "" : "nenhum dispositivo");

  if (!achouPca) {
    Serial.println("PCA9685 NAO respondeu em 0x40.");
    Serial.println("  Confira: VCC->3V3, GND comum, SDA->21, SCL->22, jumpers A0..A5 abertos.");
    return;
  }

  // 3) Registros: MODE1, MODE2, PRESCALE
  Serial.printf("MODE1=0x%02X  MODE2=0x%02X  PRESCALE=%u\n", lerReg(0x00), lerReg(0x01), lerReg(0xFE));

  // 4) Escreve o prescale de 50 Hz (so pode com o chip em SLEEP) e le de volta
  gravarReg(0x00, 0x10);            // MODE1: SLEEP
  gravarReg(0xFE, 121);             // 25 MHz / (4096 * 50 Hz) - 1 = 121
  uint8_t ps = lerReg(0xFE);
  gravarReg(0x00, 0x00);            // acorda
  delay(1);
  gravarReg(0x00, 0xA0);            // RESTART + AUTO-INCREMENT
  Serial.printf("PRESCALE gravado=121, lido=%u  %s\n", ps, ps == 121 ? "(ok, I2C funcionando)" : "(FALHA)");

  // 5) Pino OE
  if (OE_PIN >= 0) {
    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, LOW);
    Serial.printf("OE (GPIO%d) em LOW = saidas habilitadas.\n", OE_PIN);
  }
  Serial.println("Teste 1 concluido. Proximo: T2_servo_serial.");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  rodarTeste();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') rodarTeste();
  }
}
