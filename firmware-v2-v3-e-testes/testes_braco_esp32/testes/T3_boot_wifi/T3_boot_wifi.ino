/*
  TESTE 3 - Por que o Wi-Fi some quando o ESP32 e alimentado pelos 6 V?
  ------------------------------------------------------------------------
  Quando o ESP32 esta no VIN nao da para ver o Monitor Serial (nao ligue USB e VIN juntos).
  Entao este sketch GRAVA UM DIARIO NA FLASH: a cada boot anota o motivo do reset e ate
  onde chegou (iniciou, ligou o Wi-Fi, rede no ar, 10 s estavel, celular conectado).
  Depois voce liga pelo USB (com o VIN desconectado) e le o diario.

  O LED azul da placa (GPIO2) mostra o estado sem precisar do Serial:
    3 piscadas rapidas  = acabou de dar boot
    piscando rapido     = ligando o Wi-Fi
    piscando devagar    = rede "BracoTeste" no ar
    aceso direto        = celular conectado
  Se ele fica repetindo as 3 piscadas, o ESP32 esta reiniciando em loop (quase sempre brownout).

  ROTEIRO
  1. Grave pelo USB. Abra o Monitor Serial: confira que a rede aparece no celular.
  2. Desligue o USB. Ligue o fio do VIN e ligue o conversor de 6 V. Observe o LED e o celular
     por ~30 s. Se a rede nao aparecer, aperte o botao EN da placa uma vez e observe de novo.
  3. Desligue o conversor, tire o fio do VIN, ligue o USB, abra o Monitor Serial: o diario
     aparece sozinho (ou digite "l"). "z" apaga o diario.
  4. Repita o passo 2 com TX_POTENCIA = WIFI_POWER_19_5dBm (potencia maxima, igual ao firmware
     completo) para ver se a diferenca e a corrente do radio.

  LEITURA DO DIARIO
    BROWNOUT     -> a tensao do ESP32 caiu durante o pico do radio: problema de alimentacao.
    POWERON e fase 4/5 -> ligou normal; se o celular nao via a rede, o problema e outro.
    POWERON e fase 1 ou 2 sem nada depois -> travou na partida (rampa lenta do conversor);
                    o botao EN resolver confirma isso.
*/
#include <WiFi.h>
#include <Preferences.h>
#include "esp_system.h"

// ---------------------------- ajustes do teste ----------------------------
#define LED_PIN        2                    // LED azul da maioria das ESP32 DevKit
#define OE_PIN         4                    // mantem o PCA desligado (-1 se nao ligado)

#define ATRASO_WIFI_MS 1500                 // espera antes de ligar o radio (deixa a fonte assentar)
#define TX_POTENCIA    WIFI_POWER_8_5dBm    // teste tambem WIFI_POWER_19_5dBm
const char* SSID = "BracoTeste";
const char* SENHA = "braco1234";
// --------------------------------------------------------------------------

enum Fase : uint8_t { F_INICIO = 1, F_WIFI = 2, F_AP = 3, F_ESTAVEL = 4, F_CLIENTE = 5 };
const int NLOG = 12;
struct Registro { uint8_t motivo, fase; uint32_t boot; };
Registro diario[NLOG];
uint32_t bootNum = 0;
Preferences prefs;
uint8_t faseAtual = 0;
uint32_t tAp = 0;

const char* nomeMotivo(uint8_t r) {
  switch ((esp_reset_reason_t)r) {
    case ESP_RST_POWERON:  return "POWERON (ligou a energia)";
    case ESP_RST_EXT:      return "EXT (pino EN)";
    case ESP_RST_SW:       return "SW (reinicio por software)";
    case ESP_RST_PANIC:    return "PANIC (travou/excecao)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "WATCHDOG";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT  <<< tensao caiu";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "desconhecido";
  }
}
const char* nomeFase(uint8_t f) {
  switch (f) {
    case F_INICIO:  return "1 iniciou";
    case F_WIFI:    return "2 ligando Wi-Fi";
    case F_AP:      return "3 rede no ar";
    case F_ESTAVEL: return "4 rede no ar ha 10 s";
    case F_CLIENTE: return "5 celular conectou";
    default:        return "-";
  }
}

void salvaDiario() { prefs.putBytes("log", diario, sizeof(diario)); }

void marcaFase(uint8_t f) {                 // grava so quando avanca (poucas escritas na flash)
  if (f <= faseAtual) return;
  faseAtual = f;
  diario[NLOG - 1].fase = f;
  salvaDiario();
  Serial.printf("[fase] %s\n", nomeFase(f));
}

void mostraDiario() {
  Serial.println("\n----- DIARIO DE BOOTS (mais antigo -> mais recente) -----");
  for (int i = 0; i < NLOG; i++) {
    if (diario[i].boot == 0) continue;
    Serial.printf("boot %4lu | %-28s | chegou ate: %s%s\n", (unsigned long)diario[i].boot,
                  nomeMotivo(diario[i].motivo), nomeFase(diario[i].fase),
                  i == NLOG - 1 ? "   (este boot)" : "");
  }
  Serial.println("---------------------------------------------------------");
}

void pisca(int n, int ms) {
  for (int i = 0; i < n; i++) { digitalWrite(LED_PIN, HIGH); delay(ms); digitalWrite(LED_PIN, LOW); delay(ms); }
}

void setup() {
  if (OE_PIN >= 0) { pinMode(OE_PIN, OUTPUT); digitalWrite(OE_PIN, HIGH); }
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  prefs.begin("diag", false);
  memset(diario, 0, sizeof(diario));
  if (prefs.getBytesLength("log") == sizeof(diario)) prefs.getBytes("log", diario, sizeof(diario));
  bootNum = prefs.getUInt("n", 0) + 1;
  prefs.putUInt("n", bootNum);
  memmove(diario, diario + 1, sizeof(Registro) * (NLOG - 1));          // empurra o historico
  diario[NLOG - 1] = {(uint8_t)esp_reset_reason(), 0, bootNum};
  marcaFase(F_INICIO);

  pisca(3, 80);
  delay(500);
  mostraDiario();

  // ---- Wi-Fi ----
  uint32_t t0 = millis();
  while (millis() - t0 < ATRASO_WIFI_MS) { digitalWrite(LED_PIN, (millis() / 60) % 2); delay(10); }
  marcaFase(F_WIFI);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(TX_POTENCIA);
  bool ok = WiFi.softAP(SSID, SENHA);
  WiFi.setTxPower(TX_POTENCIA);              // alguns pacotes so aceitam depois do softAP
  Serial.printf("softAP: %s  IP %s  potencia=%d (x0,25 dBm)\n", ok ? "ok" : "FALHOU",
                WiFi.softAPIP().toString().c_str(), (int)WiFi.getTxPower());
  if (ok) { marcaFase(F_AP); tAp = millis(); }
  Serial.println("Digite l para ver o diario, z para apagar.");
}

void loop() {
  uint32_t now = millis();
  int clientes = WiFi.softAPgetStationNum();
  if (faseAtual >= F_AP && now - tAp > 10000) marcaFase(F_ESTAVEL);
  if (clientes > 0) marcaFase(F_CLIENTE);

  if (clientes > 0) digitalWrite(LED_PIN, HIGH);
  else if (faseAtual >= F_AP) digitalWrite(LED_PIN, (now / 500) % 2);
  else digitalWrite(LED_PIN, (now / 60) % 2);

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'l') mostraDiario();
    if (c == 'z') { memset(diario, 0, sizeof(diario)); salvaDiario(); prefs.putUInt("n", 0); Serial.println("Diario apagado."); }
  }
  delay(10);
}
