# Įterptinių sistemų inžinerinis projektas

<img width="903" height="774" alt="image" src="https://github.com/user-attachments/assets/f60f2acc-ccac-4346-9729-32f1e3194139" /> <br />
<img width="732" height="590" alt="image" src="https://github.com/user-attachments/assets/91ca460c-ab8e-49de-b5ac-fe785349629d" /> <br />
 
Matuojamas apšviestumas:
* 20 - 200 lx
* 0 - 100 Hz
* 2 kanalai
* Skaičiuojamas kiekvieno kanalo vidurkis ir kanalų skirtumas

Matuojama nuolatinė įtampa:
* 0.1 - 25 V
* 0 - 0.1 Hz
* 2 kanalai
* Skaičiuojamas kiekvieno kanalo vidurkis

Indikatoriuje parodymai atnaujinami kas 2s, apšviestumo parametrai į PC perduodami kas 0.2 s, įtampos - kas 1 s.

## Pagrindiniai komponentai:
* STM32L073RZ
* TEPT5600 fototranzistoriai
* MCP6S22 PGA
* SSD1306 I2C OLED ekranas
