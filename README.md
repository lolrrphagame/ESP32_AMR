ESP32-S3 UART SEQUENCE TEST compatible with Raspberry Pi pi_uart_seq.py

This package contains only the main/ folder files for an EXISTING ESP-IDF project.
1) In your existing ESP32S3_TEST project, remove or rename the original hello_world_main.c.
2) Copy main/main.c and main/CMakeLists.txt into that project's main/ folder.
3) In the ESP-IDF Terminal opened in the project ROOT, run:
      idf.py set-target esp32s3    (only if project target is not yet esp32s3)
      idf.py build
      idf.py -p COM3 flash monitor
4) Leave the board plugged into the computer's COM-labelled USB-C port.
5) Cross-wire Raspberry Pi 4's GPIO UART (3.3V logic only):
      Pi physical PIN 8  GPIO14 TXD --> ESP GPIO18 RX (UART1)
      Pi physical PIN 10 GPIO15 RXD <-- ESP GPIO17 TX (UART1)
      Pi physical PIN 6  GND       --- ESP GND
   Do not connect the Pi 5V/3V3 power pins; keep grounds common.
6) From Raspberry Pi 4, stop ROS serial users and run:
      python3 pi_uart_seq.py --port /dev/serial0 --baud 57600 --count 100
   Expected only when correctly wired and working:
      RESULT 100/100 matched, 0 failed, 0 unexpected lines

IMPORTANT: This firmware listens on ESP UART1 GPIO17/18, NOT the COM/USB connector.
The COM USB port and COM3 remain for flashing/logging on your Windows PC.
If your Pi is connected to ESP via a USB cable instead of GPIO wires,
this firmware must be adapted to that serial interface.
No motors are activated by this firmware.
