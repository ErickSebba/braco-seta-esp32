# Testes por partes – Braço ESP32 (SETA)

Rode na ordem e só avance quando o teste atual passar. Todos usam a mesma ligação do firmware
completo (SDA 21, SCL 22, OE 4, PCA em 0x40, canais 0 Base … 5 Garra).

| # | Sketch | Alimentação | Pergunta que responde |
|---|---|---|---|
| 0 | (multímetro) | conversor ligado | Tem 6,0 V no borne V+ do PCA e no VIN do ESP? A tensão cai quando o ESP liga o Wi‑Fi? |
| 1 | `T1_i2c_pca` | só USB | O ESP32 conversa com o PCA9685? |
| 2 | `T2_servo_serial` | USB no ESP + 6 V no borne V+ | Os servos mexem sem Wi‑Fi? Qual canal falha? |
| 3 | `T3_boot_wifi` | 6 V no VIN (sem USB) | O ESP reinicia (brownout) ou trava na partida? Baixar a potência do rádio resolve? |
| 4 | `T4_web_servo` | primeiro USB, depois VIN | O caminho celular → página → servo funciona? A rede cai quando um servo grande mexe? |
| 5 | firmware completo | – | Volta para `braco_esp32_ik` com as correções que os testes apontarem. |

Nunca ligue USB e VIN ao mesmo tempo.

## O que anotar
- T1: a linha `SDA=… SCL=…`, os endereços encontrados e o PRESCALE lido.
- T2: quais canais mexeram com `t`; se algum só zumbe ou treme.
- T3: o diário (motivo do reset + até que fase chegou), com 8,5 dBm e com 19,5 dBm, e se o botão EN fez a rede aparecer.
- T4: se aparece `[n] canal … ` no Serial ao mexer o slider; se a rede cai ao mexer base/ombro.
