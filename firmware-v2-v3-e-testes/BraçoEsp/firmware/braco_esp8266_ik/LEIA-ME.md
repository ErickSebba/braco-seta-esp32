# Braço – versão 2 para ESP8266 (NodeMCU)

Mesma página, cinemática, calibração e "pegar e levar" da pasta `braco_esp32_ik`, adaptado para ESP8266.
Use até o ESP32 chegar. **Compilado com o pacote oficial esp8266 3.1.2 sem erros nem avisos**
(usa 40% da RAM e 30% da flash). Ainda não foi testado com a placa e o braço reais.

## Passo a passo no Arduino IDE
1. **Arquivo → Preferências → URLs adicionais do Gerenciador de Placas**, cole (se já houver outra URL, separe com vírgula):
   `https://arduino.esp8266.com/stable/package_esp8266com_index.json`
2. **Ferramentas → Placa → Gerenciador de Placas**: procure **esp8266** e instale **"esp8266 by ESP8266 Community"**.
3. **Ferramentas → Gerenciar Bibliotecas**: procure **Adafruit PWM Servo Driver** e instale (se perguntar, aceite instalar também a *Adafruit BusIO*).
4. **Arquivo → Abrir** → `BraçoEsp\firmware\braco_esp8266_ik\braco_esp8266_ik.ino`
   (as outras abas — `kin.h`, `tasks.h`, `motion.h`, `pagina.h` — abrem sozinhas; não renomeie a pasta).
5. **Ferramentas → Placa → ESP8266 Boards → NodeMCU 1.0 (ESP-12E Module)**.
6. Plugue a placa no USB (chave da caixa **desligada**) e escolha **Ferramentas → Porta** (ex.: `COM4`).
   - Sem porta? Instale o driver do chip USB da placa: **CP2102** (placa "Ver 0.1") ou **CH340** (HW-306).
7. Clique em **Carregar (→)**. Se não começar a gravar, tente **Ferramentas → Upload Speed → 115200** ou segure o botão **FLASH** ao clicar em Carregar.
8. **Ferramentas → Monitor Serial**, 115200 baud, aperte **RST** na placa: deve aparecer `Wi-Fi: BracoESP8266  IP: 192.168.4.1`.
9. No celular: rede **BracoESP8266**, senha **braco1234**, abra **http://192.168.4.1**.

## Ligação (NodeMCU)
| NodeMCU | PCA9685 / outros |
|---|---|
| D2 (GPIO4) | SDA |
| D1 (GPIO5) | SCL |
| 3V3 | VCC (lógica) |
| D5 (GPIO14) | OE (opcional) |
| GND | GND do PCA9685 e do conversor (todos juntos) |
| VIN | saída do conversor (6,0 V) — a mesma que vai no V+ do PCA9685 |

Não use D3, D4 e D8 para outras coisas: eles decidem como a placa liga.
Servos nos canais 0 Base, 1 Ombro, 2 Cotovelo, 3 Punho, 4 Giro do punho, 5 Garra.

## Diferenças em relação ao ESP32
- Resposta da página um pouco mais lenta e um celular por vez de preferência.
- Calibração e pontos ficam na EEPROM desta placa: ao trocar para o ESP32, refaça a calibração.
