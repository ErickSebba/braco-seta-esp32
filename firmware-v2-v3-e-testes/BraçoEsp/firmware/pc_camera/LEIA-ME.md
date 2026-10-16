# Braço ESP32 – versão 3 (computador + câmera) — FUTURO

Pasta reservada para a próxima etapa: um programa no computador (provavelmente Python + OpenCV)
que encontra o objeto pela câmera e manda o ESP32 pegar e levar.

Ideia combinada até agora:
- O ESP32 continua sendo o "músculo": cinemática, calibração e movimentos (firmware `braco_esp32_ik`).
- O computador só envia pontos pelos mesmos endereços que a página usa (ex.: `/api/movel?x=..&y=..&z=..`,
  `/api/pick`), por Wi‑Fi. Se precisar, dá para acrescentar USB serial depois.
- Vai precisar de uma calibração câmera → mesa (pixels para mm), por exemplo com marcadores nos cantos.
