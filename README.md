# Braço SETA (ESP32 / ESP8266)

Braço robótico com servos controlado por ESP32 ou ESP8266, com cinemática direta e inversa no próprio microcontrolador (`kin.h`), tarefa "pegar e levar" (`tasks.h`) e página web de controle (`pagina.h`). Inclui peças mecânicas em STEP, as versões v2/v3 do firmware, testes de servo e a documentação completa do firmware (`docs/`).

## Contexto
Projeto separado do BIMO. Antes estava dentro de `bimo-firmware-hardware` com o nome BraçoEsp; a leitura do código e da documentação mostrou que é o "Braço SETA". A mecânica parte do braço "DIY Arduino Robot Arm" (FABRI creator), cuja referência está em `braco-robotico-referencias`.

## Estrutura
- `docs/` (1 arquivos)
- `firmware-v2-v3-e-testes/` (52 arquivos)
- `logo/` (1 arquivos)
- `mecanica-e-firmware-v1/` (19 arquivos)

## Conteúdo
- 74 arquivos (33.9 MB no total).
- Tipos mais comuns: .h ×28, .ino ×12, .md ×10, .html ×8, .svg ×6, .zip ×6, .step ×4.
- Datas dos arquivos: 2026-10-15 a 2026-10-18.

## Como executar
- Arduino/ESP32: abrir os `.ino` na Arduino IDE ou PlatformIO; `wokwi-*` roda no simulador Wokwi.

## Estado e observações
`mecanica-e-firmware-v1` (veio da Área de Trabalho) e `firmware-v2-v3-e-testes` (veio de Projetos-faculdade-codigo) são complementares, não duplicados: só 11 arquivos em comum.

## Origem dos arquivos
Reorganização feita entre 24 e 25/11/2026. De onde veio cada parte:
- `firmware-v2-v3-e-testes` ← `Área de Trabalho\Projetos-faculdade-codigo\Faculdade-Robotica\BraçoEsp`
- `mecanica-e-firmware-v1` ← `Área de Trabalho\BraçoEsp`
- `docs\2026-10-18_braco-seta-firmware.md` ← `Downloads\Braco_SETA_firmware.md`
- `firmware_completo.zip` ← `Downloads\firmware_completo.zip`
- `logo\SETA Logo Design File.zip` ← `Downloads\SETA Logo Design File.zip`
- `mecanica-e-firmware-v1` ← `C:\Users\erick\Projetos\bimo-firmware-hardware\braco-esp`
- `firmware-v2-v3-e-testes` ← `C:\Users\erick\Projetos\bimo-firmware-hardware\braco-esp-faculdade-robotica`
- `docs\2026-10-18_braco-seta-firmware.md` ← `C:\Users\erick\Projetos\bimo-firmware-hardware\material-de-downloads\2026-10-18_braco-seta-firmware.md`
- `logo\SETA Logo Design File.zip` ← `C:\Users\erick\Projetos\bimo-firmware-hardware\material-de-downloads\SETA Logo Design File.zip`
- `firmware_completo.zip` ← `C:\Users\erick\Projetos\bimo-firmware-hardware\material-de-downloads\firmware_completo.zip`

_Detalhes completos em `log_movimentos_fase1.csv` a `log_movimentos_fase5.csv` (pasta `testesClaudeCode`)._