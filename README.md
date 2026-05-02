# Įterptinių sistemų inžinerinis projektas

Matuojamas apšviestumas:
* 10 - 1000 lux
* 0 - 100 Hz
* 2 kanalai
* Skaičiuojamas kiekvieno kanalo vidurkis ir kanalų skirtumas

Matuojama nuolatinė įtampa:
* 0.1 - 25 V
* 0 - 0.1 Hz
* 2 kanalai
* Skaičiuojamas kiekvieno kanalo vidurkis

Indikatoriuje parodymai atnaujinami kas 2s, apšviestumo parametrai į PC perduodami kas 0.2 s, įtampos - kas 1 s.

Pagrindiniai komponentai:
* STM32F042K6T6
* TEPT5600 fototranzistoriai
* MCP6S22 PGA
* SSD1306 I2C OLED ekranas
