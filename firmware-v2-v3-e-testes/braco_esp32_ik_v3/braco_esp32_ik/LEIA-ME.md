# Braço ESP32 – versão 2 (cinemática, calibração e pegar-e-levar)

Abra `braco_esp32_ik.ino` no Arduino IDE (placa **ESP32 Dev Module**, biblioteca **Adafruit PWM Servo Driver**).
Ligação igual à versão 1. Rede Wi‑Fi `BracoESP32` / senha `braco1234`, página em http://192.168.4.1

## O que tem
| Arquivo | O que faz |
|---|---|
| `braco_esp32_ik.ino` | Wi‑Fi, rotas HTTP, servos, calibração, pontos ensinados, loop de 50 Hz |
| `kin.h` | Cinemática direta e inversa (garra sempre para baixo) e mapeamento servo ↔ junta |
| `tasks.h` | Movimento em linha reta e a sequência "pegar e levar" |
| `motion.h` | Rampa de velocidade das juntas (igual à v1) |
| `pagina.html` / `pagina.h` | Página com as 3 vistas (3/4, lado, cima) e as abas Juntas, Mover XYZ, Tarefa, Calibração |

Medidas usadas (tiradas do Fusion): ombro 13,8 mm à frente do eixo da base e 97,5 mm acima da mesa;
braço 120,0 mm; antebraço 119,7 mm; eixo do punho até o centro dos dedos 119,2 mm.
Sistema de coordenadas: origem no eixo da base na altura da mesa, **x para a frente** (face UFMT),
**y para a esquerda**, **z para cima**, tudo em mm.

## Roteiro de teste no laboratório
1. **Sem servos ligados aos fios** (ou com o braço solto): grave, conecte no Wi‑Fi e confira se a página abre e se o desenho aparece.
2. **Deixe o braço perto da pose em pé** e toque em *Ligar servos*. Ele vai para 90° em tudo — observe se nada bate.
3. **Calibração (aba Calibração), em ordem 1 → 8.** Em cada passo use os sliders que aparecem, confira com esquadro/nível e toque em *Registrar*.
   - Se algum passo não der para alcançar (servo no fim do curso), anote qual: pode ser a montagem do horn no estriado.
4. **Confira a calibração:** na aba Juntas, mexa cada slider e compare o desenho com o braço real. Se uma junta andar ao contrário no desenho, refaça os passos dela.
5. **Mover XYZ com passo de 5 mm:** teste Subir/Descer/Frente/Trás/Esquerda/Direita. A garra deve continuar apontando para baixo. Meça com uma régua o quanto ela andou de verdade.
6. **Ensinar pontos:** leve a garra até um objeto (ex.: tampinha) e salve "pegar"; leve até outro lugar e salve "soltar".
7. **Tarefa:** comece com velocidade 20–30 % e altura de aproximação 40 mm. Mantenha a mão perto do botão **PARAR**.
8. Anote o erro de posição (quanto a garra erra o alvo) — isso decide se vale ajustar as medidas ou a calibração.

## Limitações conhecidas
- Servos de hobby não informam a posição: o desenho mostra onde o braço **deveria** estar.
- O giro do punho vem antes da inclinação: com a garra para baixo, os dedos sempre abrem perpendiculares à direção do braço.
- Ao ligar, o firmware assume a pose home (90°). Ligue com o braço perto dela.
- Precisão esperada com MG996R: alguns mm até ~1 cm.

## O que foi testado (sem o braço físico)
- `kin.h`/`tasks.h` no PC: a pose do CAD dá a mesma posição da ponta que o Fusion (203,1 / 151,1 mm); ida e volta da cinemática inversa com erro < 0,001 mm em 2082 pontos; simulação completa do pegar‑e‑levar com juntas em rampa e calibração com sinais trocados.
- Compilado com o pacote oficial esp32 3.0.7 (placa ESP32 Dev Module): sem erros, 77% da flash e 14% da RAM.
- Página no navegador contra um simulador da API: calibração em 8 passos, desenho igual ao cálculo do firmware, jog, ensinar pontos e a tarefa completa.
