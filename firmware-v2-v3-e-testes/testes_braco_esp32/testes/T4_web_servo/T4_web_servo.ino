/*
  TESTE 4 - Pagina minima -> ESP32 -> PCA9685 -> servo
  ------------------------------------------------------------------
  So rode depois que o Teste 2 mexer os servos e o Teste 3 mostrar a rede estavel.
  E o caminho completo (celular -> Wi-Fi -> HTTP -> servo) com o minimo de codigo:
  6 sliders, um por canal, e um botao para soltar tudo. Cada pedido aparece no Serial.

  COMO RODAR
  a) Primeiro com o ESP32 no USB (VIN desconectado) e 6 V so no borne V+ do PCA:
     acompanhe o Monitor Serial enquanto mexe os sliders.
  b) Depois com o ESP32 no VIN (sem USB): mesmos sliders. Se a rede cair ao mexer um servo
     grande (base/ombro), e a queda de tensao no pico de corrente do servo.

  Rede "BracoTeste" / senha "braco1234" / http://192.168.4.1
  Os sliders so mandam pulso depois que voce encosta neles (nada pula ao abrir a pagina).
*/
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SDA_PIN  21
#define SCL_PIN  22
#define OE_PIN    4
#define PCA_ADDR 0x40
#define US_MIN  500
#define US_MAX 2500
#define TX_POTENCIA WIFI_POWER_8_5dBm
const char* SSID = "BracoTeste";
const char* SENHA = "braco1234";

Adafruit_PWMServoDriver pwm(PCA_ADDR, Wire);
WebServer server(80);
bool pcaOk = false;
uint32_t nPedidos = 0;

const char PAGINA[] PROGMEM = R"HTML(<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Teste servos</title>
<style>body{font:16px system-ui,sans-serif;background:#0D2648;color:#eef;margin:0;padding:16px}
h1{font-size:20px;margin:0 0 4px}.r{margin:14px 0}label{display:flex;justify-content:space-between}
input{width:100%}button{font:inherit;padding:12px;width:100%;margin-top:10px;border:0;border-radius:8px;background:#038FFD;color:#fff}
#st{color:#88A5D6;font-size:14px;min-height:1.2em}</style></head><body>
<h1>Teste de servos</h1><div id="st">conectando...</div><div id="s"></div>
<button onclick="go('off')">Soltar todos</button>
<script>
const N=["0 Base","1 Ombro","2 Cotovelo","3 Punho","4 Giro punho","5 Garra"];
const st=document.getElementById('st');
let busy=false,pend=null;
async function go(p){try{const r=await fetch('/'+p,{cache:'no-store'});st.textContent=await r.text()}catch(e){st.textContent='sem resposta do ESP32: '+e}}
async function envia(c,a){pend=[c,a];if(busy)return;busy=true;while(pend){const[x,y]=pend;pend=null;await go('set?c='+x+'&a='+y)}busy=false}
document.getElementById('s').innerHTML=N.map((n,i)=>`<div class="r"><label><span>${n}</span><b id="v${i}">--</b></label>
<input type="range" min="0" max="180" value="90" oninput="v${i}.textContent=this.value;envia(${i},this.value)"></div>`).join('');
go('ping');
</script></body></html>)HTML";

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n================ TESTE 4: pagina minima ================");
  if (OE_PIN >= 0) { pinMode(OE_PIN, OUTPUT); digitalWrite(OE_PIN, LOW); }

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(PCA_ADDR);
  pcaOk = (Wire.endTransmission() == 0);
  if (pcaOk) {
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);
    for (int i = 0; i < 16; i++) pwm.setPWM(i, 0, 4096);
  }
  Serial.printf("PCA9685: %s\n", pcaOk ? "ok" : "NAO ENCONTRADO");

  delay(1000);                               // deixa a fonte assentar antes do radio
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(TX_POTENCIA);
  WiFi.softAP(SSID, SENHA);
  WiFi.setTxPower(TX_POTENCIA);
  Serial.printf("Rede %s  IP %s\n", SSID, WiFi.softAPIP().toString().c_str());

  server.on("/", []() { server.send_P(200, "text/html; charset=utf-8", PAGINA); });
  server.on("/ping", []() {
    server.send(200, "text/plain", pcaOk ? "ESP32 ok, PCA9685 ok" : "ESP32 ok, mas PCA9685 NAO encontrado");
  });
  server.on("/set", []() {
    int c = server.arg("c").toInt();
    float a = constrain(server.arg("a").toFloat(), 0, 180);
    int us = (int)(US_MIN + a / 180.0f * (US_MAX - US_MIN) + 0.5f);
    nPedidos++;
    if (pcaOk && c >= 0 && c < 16) pwm.writeMicroseconds(c, us);
    Serial.printf("[%lu] canal %d -> %.0f graus (%d us)%s\n", (unsigned long)nPedidos, c, a, us, pcaOk ? "" : "  PCA ausente!");
    server.send(200, "text/plain", String("canal ") + c + " = " + String(a, 0) + " graus (" + us + " us)");
  });
  server.on("/off", []() {
    if (pcaOk) for (int i = 0; i < 16; i++) pwm.setPWM(i, 0, 4096);
    Serial.println("todos soltos");
    server.send(200, "text/plain", "todos soltos");
  });
  server.begin();
}

void loop() {
  server.handleClient();
  delay(2);
}
