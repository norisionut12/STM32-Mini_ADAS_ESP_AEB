# Mini-ADAS: Sistem Auto de Siguranță pe STM32

Sistem Auto de Frânare de Urgență (AEB) și Control al Stabilității (ESP)

Acest proiect reprezintă un prototip pentru un sistem avansat de asistență a șoferului (ADAS). Dispozitivul simulează tracțiunea unei mașini și intervine automat pentru a preveni accidentele (frânează la obstacole) și pentru a menține controlul (reduce viteza pe pante periculoase). Proiectul este dezvoltat în **C++**.

### Video Proiect
[![Proiect Videoclip](https://img.youtube.com/vi/6T_DxWhUW28/hqdefault.jpg)](https://youtube.com/shorts/6T_DxWhUW28)

**Cum funcționează?**

* **Accelerarea (Pedala):** Un potențiometru joacă rolul pedalei de gaz. În condiții normale, motorul accelerează proporțional cu pedala.
* **Frânarea de Urgență (AEB):** Radarul ultrasonic măsoară constant distanța în față. Dacă un obstacol apare la mai puțin de 15 cm, sistemul ignoră pedala de accelerație, oprește brusc motorul și pornește o alarmă sonoră.
* **Controlul Stabilității (ESP):** Senzorul MPU6050 măsoară înclinația mașinii. Dacă detectează o pantă periculoasă (peste 20 de grade), sistemul limitează automat puterea motorului la maxim 30% pentru a preveni pierderea controlului (răsturnarea).
* **Feedback vizual:** Un ecran OLED afișează telemetria completă (viteză, distanță, status AEB/ESP), iar un shift register comandă 8 LED-uri care funcționează ca un turometru.

**Componente folosite**

* **Nucleo-L476RG:** Microcontroller-ul care funcționează ca unitate centrala.
* **HC-SR04 (Senzor Ultrasonic):** Radar frontal pentru detecția obstacolelor.
* **MPU6050 (Accelerometru/Giroscop):** Măsoară unghiul de pitch (înclinare).
* **L293D (Punte H) + Motor DC:** Simulează tracțiunea mașinii.
* **Potențiometru (50kΩ):** Simulează pedala de accelerație.
* **OLED SSD1306:** Afișează în timp real starea sistemului.
* **74HC595 (Shift Register) + 8 LED-uri:** Afișează turația motorului.
* **Buzzer:** Alarma sonoră pentru sistemul AEB.

**Configurare Hardware & STM32CubeMX**

* **ADC1 (PA0):** Citirea valorii analogice de la potențiometru.
* **I2C1:** Comunicare cu senzorul MPU6050 și ecranul OLED.
* **TIM2_CH2 (PWM):** Controlul turației motorului.
* **TIM4_CH1 (PWM):** Controlul buzzer-ului (frecvență audio).
* **GPIO Output/Input:** * Direcția motorului pe puntea H (PB4, PB5).
  * Controlul shift register-ului `74HC595` : Data (PA8), Clock (PA9), Latch (PC7).
  * Interfațarea cu radarul HC-SR04: Trig (PA10), Echo (PB10).
* **Frecvență ceas:** 80 MHz pentru un timp de răspuns critic foarte mic.
* **DWT (Data Watchpoint and Trace):** Activat pentru a genera delay-uri precise la nivel de microsecundă, necesare protocolului HC-SR04.

**Logica Codului**

Codul a fost importat din C în C++. Clasa centrală `masinaGeneral` integrează toate perifericele și rulează o buclă de decizie :
1. **Citire Senzori:** Extrage datele de la clasele `Pedala`, `Radar` și `Dinamica`.
2. **Logica de Siguranță:** Aplică prioritățile. AEB are prioritate absolută (oprește mașina). ESP are prioritate secundară (limitează viteza).
3. **Control:** Trimite comenzile către clasa `ControlMotor`.
4. **Watchdog I2C:** Clasa `EcranAfisare` monitorizează activ starea magistralei I2C.

**Structura Fișierelor**

* `Core/Src/main.cpp`: Conține toată logica aplicației (toate clasele C++ și bucla principală).
* `Core/Src/ssd1306.c` & `.h`: Driverul extern în C pentru afișajul OLED.
* `Core/Src/ssd1306_fonts.c` & `.h`: Fonturile pentru afișaj.
* `mini_radar.ioc`: Fișierul de configurare grafică generat de STM32CubeMX.
