#include "main.h"
#include <cmath>
#include <stdio.h>
// Includem librăria externa C pentru OLED
extern "C" {
    #include "ssd1306.h"
    #include "ssd1306_fonts.h"
}

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);


// Pedala ( Potentiometrul )
#define ADC_VALOARE_MAX 4095
#define ADC_TIMEOUT_MS 10

// Motorul
#define MOTOR_CCR_MAX 999

// Radar HC-SR04
#define RADAR_SYSCLK_MHZ 80
#define RADAR_TIMEOUT_US 30000
#define RADAR_FACTOR_CM 58.0f
#define RADAR_LIMITA_CM 15.0f

// MPU6050 (giroscop/accelerometru)
#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_REG_PWR 0x6B
#define MPU6050_REG_ACCEL 0x3B
#define MPU6050_LSB_PER_G 16384.0f
#define I2C_TIMEOUT_MS 10

// Buzzer
#define BUZZER_DUTY_50PCT 32767

// ESP ( Electronic Stability Program ) / AEB ( Auto Emergency Brake )
#define AEB_DISTANTA_CM 15.0f
#define ESP_UNGHI_GRADE 20.0f
#define ESP_LIMITA_PROCENT 30.0f

// Oled
#define OLED_INTERVAL_MS 100

// Citim pozitia pedalei de acceleratie printr-un potentiometru
// Returnam un procentaj intre 0 si 100

class Pedala {
private:
    ADC_HandleTypeDef* _hadc;
public:
    // Constructorul primeste o referinta la handle-ul ADC configurat de CubeMX
    Pedala(ADC_HandleTypeDef* hadc) {
        _hadc = hadc;
    }

    // Facem o conversie ADC si o transformam in procente (0-100%)
    float citesteProcent() {
        HAL_ADC_Start(_hadc);

        // Asteptam sa se termine conversia
        if (HAL_ADC_PollForConversion(_hadc, ADC_TIMEOUT_MS) != HAL_OK) {
            HAL_ADC_Stop(_hadc);
            return 0.0f; // Daca n-a reusit, returnam 0 ca sa nu se miste masina
        }

        uint16_t valBruta = HAL_ADC_GetValue(_hadc);
        HAL_ADC_Stop(_hadc);

        // Transformam valoarea bruta (0-4095) in procent (0-100%)
        return ((float)valBruta / ADC_VALOARE_MAX) * 100.0f;
    }


};

// Controlam motorul DC prin L293D:
//  - PWM-ul controleaza viteza
//  - PB4 si PB5 controleaza directia

class ControlMotor {
private:
    TIM_HandleTypeDef* _htim;
    uint32_t _canal;
    GPIO_TypeDef* _portDir1;
    uint16_t  _pinDir1;
    GPIO_TypeDef*  _portDir2;
    uint16_t _pinDir2;
    float _comandaCurenta;
public:
    ControlMotor(TIM_HandleTypeDef* htim, uint32_t canal,GPIO_TypeDef* portDir1, uint16_t pinDir1,GPIO_TypeDef* portDir2, uint16_t pinDir2) {
        _htim = htim;
        _canal = canal;
        _portDir1  = portDir1;
        _pinDir1 = pinDir1;
        _portDir2 = portDir2;
        _pinDir2 = pinDir2;
        _comandaCurenta = 0.0f;
    }

    // Pornim PWM-ul si oprim motorul initial
    void init() {
        HAL_TIM_PWM_Start(_htim, _canal);
        opreste();
    }

    // Setam viteza in procente (0-100%)
    // Transformam procentul in valoarea registrului CCR (0-999)
    void seteazaViteza(float procent) {
        // Stabilizam valoarea ca sa nu iasa din interval
        if (procent < 0.0f)   procent = 0.0f;
        if (procent > 100.0f) procent = 100.0f;

        _comandaCurenta = procent;

        // Calculam valoarea CCR proportional: Exemplu :100% → 999, 50% → 499
        uint32_t ccr = (uint32_t)((procent / 100.0f) * MOTOR_CCR_MAX);

        // Setam directia: inainte
        HAL_GPIO_WritePin(_portDir1, _pinDir1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(_portDir2, _pinDir2, GPIO_PIN_RESET);

        // Aplicam duty cycle-ul pe PWM
        __HAL_TIM_SET_COMPARE(_htim, _canal, ccr);
    }

    // Oprim motorul: PWM la 0 si ambii pini de directie pe LOW (frana)
    void opreste() {
        _comandaCurenta = 0.0f;
        __HAL_TIM_SET_COMPARE(_htim, _canal, 0);
        HAL_GPIO_WritePin(_portDir1, _pinDir1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(_portDir2, _pinDir2, GPIO_PIN_RESET);
    }

    // Returnam ultima comanda trimisa, pentru afisaj
    float getComandaCurenta() {
        return _comandaCurenta;
    }


};

// Masuram distanta prin durata pulsului Echo

class Radar {
private:
    GPIO_TypeDef* _portTrig;
    uint16_t _pinTrig;
    GPIO_TypeDef* _portEcho;
    uint16_t _pinEcho;

public:
    Radar(GPIO_TypeDef* portTrig, uint16_t pinTrig,GPIO_TypeDef* portEcho, uint16_t pinEcho) {
        _portTrig = portTrig;
        _pinTrig = pinTrig;
        _portEcho = portEcho;
        _pinEcho = pinEcho;
    }

    // Initializam DWT-ul (contorul de cicluri al procesorului) pentru delay-uri precise
    void init() {
    	// Senzorul HC-SR04 are nevoie de un impuls de fix 10 microsecunde.
    	// Functia HAL_Delay stie sa astepte doar milisecunde , iar eu am nevoie de microsecunde
    	// Pornesc DWT ca sa numere ciclurile de ceas ca sa pot face un delay extrem de mic si precis
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
        // Ne asiguram ca pinul Trig este pe LOW la start
        HAL_GPIO_WritePin(_portTrig, _pinTrig, GPIO_PIN_RESET);
    }

    // Masuram distanta in centimetri
    // Formula din datasheet: distanta (cm) = durata_echo_us / 58
    float masoareDistantaCm() {
        // Trimitem un puls de 10us pe Trig ca sa pornim masurarea
        HAL_GPIO_WritePin(_portTrig, _pinTrig, GPIO_PIN_RESET);
        delayMicrosecunde(2);
        HAL_GPIO_WritePin(_portTrig, _pinTrig, GPIO_PIN_SET);
        delayMicrosecunde(10);
        HAL_GPIO_WritePin(_portTrig, _pinTrig, GPIO_PIN_RESET);

        // Asteptam sa inceapa pulsul Echo (sa treaca pe HIGH)
        uint32_t startAsteptare = DWT->CYCCNT;
        while (HAL_GPIO_ReadPin(_portEcho, _pinEcho) == GPIO_PIN_RESET) {
            if ((DWT->CYCCNT - startAsteptare) / RADAR_SYSCLK_MHZ > RADAR_TIMEOUT_US)
                return -1.0f; // Timeout, nu e niciun obiect
        }

        // Masuram cat timp tine Echo pe HIGH
        uint32_t timpStart = DWT->CYCCNT;
        while (HAL_GPIO_ReadPin(_portEcho, _pinEcho) == GPIO_PIN_SET) {
            if ((DWT->CYCCNT - timpStart) / RADAR_SYSCLK_MHZ > RADAR_TIMEOUT_US)
                return -1.0f; // Obiect prea departe sau eroare
        }
        uint32_t timpStop = DWT->CYCCNT;

        // Calculam durata pulsului in microsecunde
        float durataMicrosecunde = (float)(timpStop - timpStart) / (float)RADAR_SYSCLK_MHZ;

        // Impartim la 58 ca sa obtinem centimetrii
        return durataMicrosecunde / RADAR_FACTOR_CM;
    }

    // Returnam true daca e un obstacol mai aproape decat limita data
    bool esteObstacolAproape(float limitaCm) {
        float d = masoareDistantaCm();
        return (d > 0.0f) && (d < limitaCm);
    }
private:
    // Delay precis in microsecunde, folosind DWT
    void delayMicrosecunde(uint32_t us) {
            uint32_t start = DWT->CYCCNT;
            uint32_t cicluriNecesare = us * RADAR_SYSCLK_MHZ;
            while ((DWT->CYCCNT - start) < cicluriNecesare) {
            }
        }

};

// Afisajul format din:
//   - Un shift register 74HC595 cu 10 LED-uri (folosesc doar 8 dintre acestea ) pentru bara de progres a motorului
//      ( PA8=Data, PA9=CLK, PC7=Latch )
//   - Un ecran OLED SSD1306 pe I2C1 (PB8=SCL, PB9=SDA)

class EcranAfisare {
private:
    GPIO_TypeDef* _portData;
    uint16_t _pinData;
    GPIO_TypeDef* _portClk;
    uint16_t _pinClk;
    GPIO_TypeDef* _portLatch;
    uint16_t _pinLatch;
    I2C_HandleTypeDef* _hi2c;
public:
    EcranAfisare(GPIO_TypeDef* portData, uint16_t pinData,GPIO_TypeDef* portClk,  uint16_t pinClk,GPIO_TypeDef* portLatch, uint16_t pinLatch,I2C_HandleTypeDef* hi2c) {
        _portData  = portData;
        _pinData = pinData;
        _portClk = portClk;
        _pinClk  = pinClk;
        _portLatch = portLatch;
        _pinLatch  = pinLatch;
        _hi2c = hi2c;
    }

    void init() {
        // Initializam ecranul OLED
        ssd1306_Init();
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
        // Stingem toate LED-urile la start
        actualizeazaLED(0.0f);
    }

    // Actualizam bara de LED-uri in functie de cat de apasata e pedala
    void actualizeazaLED(float procentMotor) {
        uint8_t bitmask = procentLaBitmask(procentMotor);

        // Dam latch pe LOW ca sa blocam iesirea in timp ce scriem
        // Trimitem cei 8 biti serial pe pin-ul Data, cu puls de CLK dupa fiecare
        // Dam latch pe HIGH ca sa "publicam" datele noi pe iesiri
        HAL_GPIO_WritePin(_portLatch, _pinLatch, GPIO_PIN_RESET);
        trimiteOctet(bitmask);
        HAL_GPIO_WritePin(_portLatch, _pinLatch, GPIO_PIN_SET);
        HAL_GPIO_WritePin(_portLatch, _pinLatch, GPIO_PIN_RESET); // il lasam jos dupa
    }

    // Afisam informatiile principale pe OLED
    // Am adaugat o verificare pentru I2C deoarece ecranul isi lua freeze la anumite momente
    // Daca detectam ca I2C-ul nu e gata, il reinitializam fortat 
    void afiseazaOLED(float viteza, float distanta, bool aebActiv, bool espActiv) {
        // Verificam starea I2C inainte sa scriem pe ecran
        if (HAL_I2C_GetState(_hi2c) != HAL_I2C_STATE_READY) {
            // Daca I2C-ul pare blocat ,fortam o repornire rapida a perifericului si a ecranului
            HAL_I2C_DeInit(_hi2c);
            HAL_I2C_Init(_hi2c);
            ssd1306_Init(); // Reinitializam ecranul
            return; 
        }

        char linie[24];

        // Curatam ecranul si incepem sa scriem
        ssd1306_Fill(Black);

        ssd1306_SetCursor(0, 0);
        // Linia 1 : Nume proiect 
        ssd1306_WriteString("Mini_ADAS", Font_7x10, White);

        // Linia 2: Procentul motorului
        ssd1306_SetCursor(0, 14);
        snprintf(linie, sizeof(linie), "Motor: %5.1f%%", viteza);
        ssd1306_WriteString(linie, Font_7x10, White);

        // Linia 3: distanta de la radar (---- inseamna ca n-a citit nimic valid)
        ssd1306_SetCursor(0, 28);
        if (distanta < 0.0f)
            snprintf(linie, sizeof(linie), "Radar: ----  cm");
        else
            snprintf(linie, sizeof(linie), "Radar: %5.1f cm", distanta);
        ssd1306_WriteString(linie, Font_7x10, White);

        // Linia 4: starea AEB si ESP
        ssd1306_SetCursor(0, 42);
        snprintf(linie, sizeof(linie), "AEB:%s  ESP:%s",
                 aebActiv ? " ON" : "OFF",
                 espActiv ? " ON" : "OFF");
        ssd1306_WriteString(linie, Font_7x10, White);

        // Daca AEB-ul a intrat, afisam si un avertisment vizual mare
        if (aebActiv) {
            ssd1306_SetCursor(20, 54);
            ssd1306_WriteString("!! BRAKE !!", Font_7x10, White);
        }

        ssd1306_UpdateScreen();
    }

private:
    // Trimitem un octet bit cu bit pe shift register (MSB ul primul)
    void trimiteOctet(uint8_t data) {
        for (int i = 7; i >= 0; i--) {
            // Extragem bitul curent si il punem pe pin-ul de date
            GPIO_PinState bit = ((data >> i) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            HAL_GPIO_WritePin(_portData, _pinData, bit);

            // Dam un puls de clock ca shift register-ul sa preia bitul
            HAL_GPIO_WritePin(_portClk, _pinClk, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(_portClk, _pinClk, GPIO_PIN_SET);
            HAL_GPIO_WritePin(_portClk, _pinClk, GPIO_PIN_RESET);
        }
    }

    // Calculam cate LED-uri trebuie aprinse
    // Exemplu: 50% - 4 LED-uri - 0b00001111 = 0x0F
    uint8_t procentLaBitmask(float procent) {
        if (procent < 0.0f)   procent = 0.0f;
        if (procent > 100.0f) procent = 100.0f;

        // Impartim intervalul 0-100% in 8 trepte de cate 12.5%
        uint8_t nrLED = (uint8_t)(procent / 12.5f);
        if (nrLED > 8) nrLED = 8;

        if (nrLED == 0) return 0x00; // Niciunul aprins
        if (nrLED == 8) return 0xFF; // Toate aprinse

        // (1 << n) - 1 produce exact n biti de 1 la coada, de ex: (1<<4)-1 = 0b00001111
        return (uint8_t)((1 << nrLED) - 1);
    }
};

// Citim acceleratia de pe MPU6050 prin I2C1 si calculam unghiul de inclinare
// Folosim atan2 ca sa obtinem unghiul in grade

class Dinamica {
private:
    I2C_HandleTypeDef* _hi2c;
    int16_t _accelX, _accelY, _accelZ;
public:
    Dinamica(I2C_HandleTypeDef* hi2c) {
        _hi2c = hi2c;
        _accelX = 0;
        _accelY = 0;
        _accelZ = 0;
    }

    // Scoatem MPU6050-ul din sleep mode (registrul 0x6B cu valoarea 0 = trezit)
    bool init() {
        uint8_t dateInit[2] = { MPU6050_REG_PWR, 0x00 };
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
            _hi2c, MPU6050_ADDR, dateInit, 2, I2C_TIMEOUT_MS
        );
        HAL_Delay(10); // Asteptam putin sa se trezeasca
        return (status == HAL_OK);
    }

    // Calculam unghiul de inclinare fata de verticala, in grade
    // Folosim atan2 cu axa X si Z (panta fata-spate)
    float citestePantaGrade() {
        if (!citesteAccelBrut()) return 0.0f;

        // Convertim valorile brute in unitati "g" (forta gravitationala)
        float ax = (float)_accelX / MPU6050_LSB_PER_G;
        float az = (float)_accelZ / MPU6050_LSB_PER_G;

        // atan2 ne da unghiul in radiani, il transformam in grade
        float unghiRad = atan2f(ax, az);
        return unghiRad * (180.0f / 3.14159265f);
    }

    // Verificam daca masina e inclinata mai mult decat limita admisa
    bool esteInclinatPericulos(float limitaGrade) {
        return (fabsf(citestePantaGrade()) > limitaGrade);
    }

private:
    // Citim cei 6 octeti bruti de accelerometru din MPU6050 (cate 2 octeti per axa)
    bool citesteAccelBrut() {
        uint8_t regAddr = MPU6050_REG_ACCEL;

        // Mai intai trimitem adresa registrului de start
        if (HAL_I2C_Master_Transmit(_hi2c, MPU6050_ADDR, &regAddr, 1, I2C_TIMEOUT_MS) != HAL_OK)
            return false;

        // Citim 6 octeti
        uint8_t buffer[6] = {0};
        if (HAL_I2C_Master_Receive(_hi2c, MPU6050_ADDR, buffer, 6, I2C_TIMEOUT_MS) != HAL_OK)
            return false;

        // Asamblam cei doi octeti in int16 pentru fiecare axa 
        _accelX = (int16_t)((buffer[0] << 8) | buffer[1]);
        _accelY = (int16_t)((buffer[2] << 8) | buffer[3]);
        _accelZ = (int16_t)((buffer[4] << 8) | buffer[5]);
        return true;
    }
};

// Buzzer-ul produce un sunet prin PWM
// Duty cycle 50% → sunet maxim, 0% → mut

class Buzzer {
private:
    TIM_HandleTypeDef* _htim;
    uint32_t _canal;
    bool _activ;
    
public:
    Buzzer(TIM_HandleTypeDef* htim, uint32_t canal) {
        _htim  = htim;
        _canal = canal;
        _activ = false;
    }

    void init() {
        HAL_TIM_PWM_Start(_htim, _canal);
        __HAL_TIM_SET_COMPARE(_htim, _canal, 0); // Pornit fara sa bazaie
    }

    void porneste() {
        if (!_activ) {
            // Setam duty cycle la 50% ca sa produca sunet
            __HAL_TIM_SET_COMPARE(_htim, _canal, BUZZER_DUTY_50PCT);
            _activ = true;
        }
    }

    void opreste() {
        if (_activ) {
            __HAL_TIM_SET_COMPARE(_htim, _canal, 0); // Oprit / Mut
            _activ = false;
        }
    }

    // Un beep de durata specificata (in ms), blocant
    void beep(uint32_t duratMs) {
        porneste();
        HAL_Delay(duratMs);
        opreste();
    }

    bool esteActiv() {
        return _activ;
    }

};

// Integram toate functionalitatile intr o singura clasa principala
//   - AEB (Auto Emergency Brake): frana automat la obstacol apropiat
//   - ESP (Electronic Stability Program): limitam viteza la inclinare periculoasa
// ==============================================================================

class masinaGeneral {
private:
	    Pedala& _pedala;
	    ControlMotor& _motor;
	    Radar& _radar;
	    EcranAfisare& _dashboard;
	    Dinamica& _dinamica;
	    Buzzer& _buzzer;

	    // Starea interna
	    bool _aebActiv;
	    bool _espActiv;
	    float _comandaFinalaMotor;
	    float _distantaCurenta;
	    float _unghiCurent;
	    uint32_t _ultimaActualizareOLED;
public:
    masinaGeneral(Pedala& pedala, ControlMotor& motor, Radar& radar,EcranAfisare& dashboard, Dinamica& dinamica, Buzzer& buzzer)
    : _pedala(pedala), _motor(motor), _radar(radar),_dashboard(dashboard), _dinamica(dinamica), _buzzer(buzzer)
    {
        _aebActiv = false;
        _espActiv = false;
        _comandaFinalaMotor = 0.0f;
        _distantaCurenta = -1.0f;
        _unghiCurent = 0.0f;
        _ultimaActualizareOLED = 0;
    }

    // Initializam toate componentele in ordine logica
    void init() {
        _radar.init();
        _dinamica.init();
        _motor.init();
        _buzzer.init();
        _dashboard.init();
    }

    // Citim toti senzorii, aplicam logica de siguranta si comandam motorul
    void actualizeaza() {
        // Citim toti senzorii
        float pedalaVal = _pedala.citesteProcent();
        _distantaCurenta = _radar.masoareDistantaCm();
        _unghiCurent = _dinamica.citestePantaGrade();

        // Plecam de la comanda bruta a soferului
        float comanda = pedalaVal;

        // ESP — daca suntem pe o panta prea mare, limitam viteza
        // (AEB are prioritate, asa ca ESP nu mai conteaza daca AEB-ul e activ)
        logicaESP(_unghiCurent, comanda);

        // AEB — daca e obstacol aproape, taiem tot si activam buzerul
        logicaAEB(_distantaCurenta, comanda);

        _comandaFinalaMotor = comanda;

        // Trimitem comanda la motor
        if (_aebActiv)
            _motor.opreste();
        else
            _motor.seteazaViteza(_comandaFinalaMotor);

        // Actualizam bara de LED-uri imediat (merge rapid)
        _dashboard.actualizeazaLED(_comandaFinalaMotor);

        // Actualizam OLED-ul mai rar (la fiecare 100ms) ca sa nu il suprasolicite
        if ((HAL_GetTick() - _ultimaActualizareOLED) >= OLED_INTERVAL_MS) {
            _dashboard.afiseazaOLED(_comandaFinalaMotor, _distantaCurenta, _aebActiv, _espActiv);
            _ultimaActualizareOLED = HAL_GetTick();
        }
    }

private:
    // Logica AEB: daca distanta e prea mica, taiem acceleratia si pornim buzerul
    void logicaAEB(float distanta, float& comanda) {
        bool obstacolDetectat = (distanta > 0.0f) && (distanta < AEB_DISTANTA_CM);

        if (obstacolDetectat) {
            _aebActiv = true;
            comanda   = 0.0f; // Fortat la zero, indiferent de ce zice pedala
            if (!_buzzer.esteActiv())
                _buzzer.porneste(); // Pornim alarma sonora
        } else {
            _aebActiv = false;
            if (_buzzer.esteActiv())
                _buzzer.opreste(); // Drum liber, oprim buzerul
        }
    }

    // Logica ESP: daca inclinarea e prea mare, limitam viteza la 30%
    // Nu intervenim daca AEB-ul e deja activ (el are prioritate)
    void logicaESP(float unghi, float& comanda) {
        if (_aebActiv) {
            _espActiv = false;
            return;
        }

        if (fabsf(unghi) > ESP_UNGHI_GRADE) {
            _espActiv = true;
            // Nu lasam motorul sa mearga mai mult de 30% pe panta periculoasa
            if (comanda > ESP_LIMITA_PROCENT)
                comanda = ESP_LIMITA_PROCENT;
        } else {
            _espActiv = false;
        }
    }
};


// Pedala de acceleratie: potentiometru pe ADC1 (PA0 / ADC_CHANNEL_5)
Pedala gaz(&hadc1);

// Motorul: PWM pe TIM2_CH2, directie pe PB4 si PB5
ControlMotor motor(&htim2, TIM_CHANNEL_2, GPIOB, GPIO_PIN_4, GPIOB, GPIO_PIN_5);

// Radarul HC-SR04: Trig pe PA10, Echo pe PB10
Radar radar(GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_10);

// Dashboard: LED-uri pe shift register (PA8=Data, PA9=CLK, PC7=Latch) + OLED pe I2C1
EcranAfisare afisaj(GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_9, GPIOC, GPIO_PIN_7, &hi2c1);

// Dinamica vehiculului: MPU6050 pe I2C1
Dinamica dinamica(&hi2c1);

// Buzer: PWM pe TIM4_CH1 (PB6)
Buzzer buzzer(&htim4, TIM_CHANNEL_1);

// ECU-ul primeste referinte la toate componentele de mai sus
masinaGeneral masinaMini(gaz, motor, radar, afisaj, dinamica, buzzer);

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  masinaMini.init(); // Pornește toți senzorii și afișajele

  while (1)
  {
	  masinaMini.actualizeaza();
	      HAL_Delay(60);

  }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F12981;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

static void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }

}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
