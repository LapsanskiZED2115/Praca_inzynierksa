# Praca inżynierska
Najbardziej dumny jestem z implementacji kaskadowego układu regulacji stabilizacji drona, który napisałem w języku C na mikrokontrolerze STM32.
Cały system składa się z dwóch pętli: zewnętrznej (kątowej), która generuje zadane prędkości kątowe, oraz wewnętrznej (rate), która bezpośrednio stabilizuje układ.

Do implementacji algorytmu stabilizacji oraz całej komunikacji napisałem około 1000 linii kodu.
Projekt działa w czasie rzeczywistym (~1 kHz), z wykorzystaniem timerów i przerwań.

Kod zawiera fuzję danych z IMU (akcelerometr + żyroskop) przy użyciu filtra Madgwicka oraz dodatkowo filtrację dolnoprzepustową w celu redukcji szumów.
Na tej podstawie wyznaczana jest orientacja drona (roll, pitch, yaw), która trafia do regulatorów PID.

Dodatkowo zaimplementowałem:

własny mikser silników (układ quad X)
ograniczenia sygnałów oraz zabezpieczenia (np. tilt kill przy zbyt dużym przechyle)
system komunikacji z jednostką nadrzędną (np. Raspberry Pi)
telemetrykę wysyłaną przez UART (DMA)
mechanizmy bezpieczeństwa (timeout komunikacji, reset regulatorów)

Równolegle do części embedded stworzyłem warstwę wysokopoziomową w Pythonie, odpowiedzialną za przetwarzanie obrazu oraz komunikację z mikrokontrolerem.
W tej części wykorzystałem model YOLOv8, który został przeze mnie wytrenowany na zbiorze danych i służy do detekcji obiektów w czasie rzeczywistym.
Python odpowiada również za wysyłanie komend sterujących do STM32 (np. zadanych kątów/prędkości) przez interfejs SPI/UART.


# Technologie oraz mechanizmy jakie wykorzystałem:
-STM32 (STM32CubeIDE, HAL)
-język C (embedded)
-Python
-kaskadowa regulacja PID (angle + rate loop)
-filtr Madgwick (fuzja danych IMU)
-filtr dolnoprzepustowy (LPF)
-DMA (UART, SPI)
-przerwania (timery, SPI, UART)
-I2C (IMU – MPU9250)
-SPI (komunikacja z systemem nadrzędnym)
-UART (telemetria)
-PWM (sterowanie ESC i silnikami)
-ADC (monitoring napięcia – battery monitoring)
-system czasu rzeczywistego oparty na timerach sprzętowych

Dodatkowo w projekcie wykorzystałem model YOLOv8 do detekcji obiektów (trenowany na własnym zbiorze danych), uruchamiany na jednostce nadrzędnej, co umożliwia rozszerzenie systemu o funkcje autonomiczne.

<img width="919" height="257" alt="image" src="https://github.com/user-attachments/assets/9c165a4f-cb6f-494d-af0d-b8efb4a719c8" />

Powyżej przedstawiono schemat kaskadowego układu regulacji zaimplementowanego w systemie.

<img width="778" height="804" alt="image" src="https://github.com/user-attachments/assets/dcad7c04-7f00-4fa9-9549-46ed55e61c0b" />

Na wykresie przedstawiono wyniki działania obu pętli regulacji:

pętla kątowa (zadany vs aktualny kąt)
pętla prędkości kątowej (zadana vs zmierzona prędkość)

Układ osiąga stabilną pracę oraz poprawne śledzenie wartości zadanych.
