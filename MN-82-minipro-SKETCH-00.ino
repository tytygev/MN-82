// ============================================================
//  КАРКАС СКЕТЧА ДЛЯ УПРАВЛЕНИЯ НАГРУЗКОЙ
//  Платформа: Arduino Mini Pro (китайский аналог, Atmega328P)
//  Принципы SOLID: разделение ответственности, абстракции,
//  открытость для расширения, закрытость для модификации.
// ============================================================

// ================== НАСТРОЙКИ (ПИНЫ И КОНСТАНТЫ) ==================
// Внимание: номера пинов могут измениться после разводки платы.
// Измените их при необходимости.

// --- Пины ---
// Аналоговые входы (датчики мотора, через делитель и стабилитрон)
const uint8_t PIN_ANALOG_MOTOR_1 = A0;
const uint8_t PIN_ANALOG_MOTOR_2 = A1;

// Цифровые входы (приёмник RX480-E-WQ)
const uint8_t PIN_RC_CH1 = 2;
const uint8_t PIN_RC_CH2 = 3;
const uint8_t PIN_RC_CH3 = 4;
const uint8_t PIN_RC_CH4 = 5;
const uint8_t PIN_RC_CH5 = 6;

// Выходы управления транзисторами BC327-40 (PNP)
const uint8_t PIN_OUT_1 = 7;
const uint8_t PIN_OUT_2 = 8;
const uint8_t PIN_OUT_3 = 9;
const uint8_t PIN_OUT_4 = 10;
const uint8_t PIN_OUT_5 = 11;

// Уровни для PNP-транзистора:
#define OUTPUT_ACTIVE   LOW   // включить нагрузку
#define OUTPUT_INACTIVE HIGH  // выключить нагрузку

// --- Параметры аналоговых датчиков ---
const float DIVIDER_RATIO = 0.3197;   // Коэффициент делителя R2/(R1+R2)
// Расчёт: R2/(R1+R2), где R1 = 10кОм (от источника к пину),
// R2 = 4.7кОм (от пина к земле). Если делителя нет, поставьте 1.0
const float VREF = 5.0;               // Опорное напряжение АЦП (обычно 5.0 или 3.3)

// Порог "сухого" сигнала (используется, когда гистерезис выключен)
const float THRESHOLD = 0.5;          // Вольты

// Пороги гистерезиса (используются, когда гистерезис включён)
const float LOWER_THRESHOLD = 0.4;    // Вольты
const float UPPER_THRESHOLD = 0.6;    // Вольты
const bool  HYSTERESIS_ENABLED = true; // true = использовать гистерезис, false = простой порог

// --- Параметры RC-каналов (ШИМ) ---
const unsigned long MIN_PULSE = 1000; // Минимальная длительность импульса (мкс)
const unsigned long MAX_PULSE = 2000; // Максимальная длительность импульса (мкс)

// ================== КЛАССЫ (SOLID) ==================

// 1. Класс для чтения аналогового датчика (Single Responsibility)
class AnalogSensor {
  private:
    uint8_t _pin;
    float   _dividerRatio;
    float   _vref;
    float   _threshold;         // простой порог
    float   _lowerThreshold;    // гистерезис: нижний
    float   _upperThreshold;    // гистерезис: верхний
    bool    _hysteresisEnabled; // включён ли гистерезис
    bool    _state;             // текущее логическое состояние

  public:
    // Конструктор
    AnalogSensor(uint8_t pin, float dividerRatio, float vref,
                 float threshold,
                 bool hysteresisEnabled,
                 float lowerThreshold,
                 float upperThreshold)
      : _pin(pin), _dividerRatio(dividerRatio), _vref(vref),
        _threshold(threshold),
        _lowerThreshold(lowerThreshold),
        _upperThreshold(upperThreshold),
        _hysteresisEnabled(hysteresisEnabled),
        _state(false) {}

    void begin() {
      pinMode(_pin, INPUT);
    }

    int readRaw() {
      return analogRead(_pin);
    }

    float readVoltage() {
      int raw = readRaw();
      float voltageAtPin = (raw * _vref) / 1023.0;
      return voltageAtPin / _dividerRatio;
    }

    // Метод с гистерезисом или простым порогом:
    // возвращает измеренное напряжение, если сигнал активен, иначе 0.
    float readVoltageFiltered() {
      float voltage = readVoltage();

      if (_hysteresisEnabled) {
        // Гистерезис
        if (!_state) {
          if (voltage >= _upperThreshold) {
            _state = true;
          }
        } else {
          if (voltage <= _lowerThreshold) {
            _state = false;
          }
        }
      } else {
        // Простой порог
        _state = (voltage >= _threshold);
      }

      return _state ? voltage : 0.0;
    }

    // Получить текущее логическое состояние
    bool isActive() const {
      return _state;
    }

    // Изменить пороги на лету (при необходимости)
    void setThresholds(float threshold, float lower, float upper, bool hyst) {
      _threshold = threshold;
      _lowerThreshold = lower;
      _upperThreshold = upper;
      _hysteresisEnabled = hyst;
    }
};

// 2. Класс для чтения одного канала приёмника (Single Responsibility)
class RCChannel {
  private:
    uint8_t  _pin;
    bool     _usePWM;        // true = измерять длительность импульса
    unsigned long _minPulse;
    unsigned long _maxPulse;

  public:
    RCChannel(uint8_t pin, bool usePWM=false,
              unsigned long minPulse, unsigned long maxPulse)
      : _pin(pin), _usePWM(usePWM), _minPulse(minPulse), _maxPulse(maxPulse) {}

    void begin() {
      pinMode(_pin, INPUT);
    }

    // Чтение нормализованного значения:
    //   - для ШИМ: 0..1000
    //   - для цифрового: 0 или 1
    int read() {
      if (_usePWM) {
        unsigned long duration = pulseIn(_pin, HIGH, 25000);
        if (duration == 0) return 0;
        if (duration <= _minPulse) return 0;
        if (duration >= _maxPulse) return 1000;
        return map(duration, _minPulse, _maxPulse, 0, 1000);
      } else {
        return digitalRead(_pin);
      }
    }

    // Сырая длительность импульса (только для ШИМ)
    unsigned long readPulseRaw() {
      if (_usePWM) {
        return pulseIn(_pin, HIGH, 25000);
      }
      return 0;
    }
};

// 3. Класс управления выходом (Single Responsibility)
class OutputController {
  private:
    uint8_t _pin;
    bool    _inverted; // true = активный уровень LOW (для PNP)

  public:
    // По умолчанию inverted = true (для BC327-40)
    OutputController(uint8_t pin, bool inverted = true)
      : _pin(pin), _inverted(inverted) {}

    void begin() {
      pinMode(_pin, OUTPUT);
      off(); // безопасное состояние при старте
    }

    void on() {
      digitalWrite(_pin, _inverted ? LOW : HIGH);
    }

    void off() {
      digitalWrite(_pin, _inverted ? HIGH : LOW);
    }

    void set(bool state) {
      state ? on() : off();
    }

    // ШИМ-управление (если понадобится)
    void setPWM(int value) {
      if (_inverted) value = 255 - value;
      analogWrite(_pin, value);
    }
};

// 4. Главный контроллер (объединяет все компоненты, содержит логику)
class MainController {
  private:
    AnalogSensor _motorSensor1;
    AnalogSensor _motorSensor2;
    RCChannel    _rcChannels[5];
    OutputController _outputs[5];

  public:
    // Конструктор инициализирует все объекты с нужными параметрами
    MainController()
      : _motorSensor1(PIN_ANALOG_MOTOR_1, DIVIDER_RATIO, VREF,
                      THRESHOLD, HYSTERESIS_ENABLED,
                      LOWER_THRESHOLD, UPPER_THRESHOLD),
        _motorSensor2(PIN_ANALOG_MOTOR_2, DIVIDER_RATIO, VREF,
                      THRESHOLD, HYSTERESIS_ENABLED,
                      LOWER_THRESHOLD, UPPER_THRESHOLD),
        _rcChannels{
          RCChannel(PIN_RC_CH1),
          RCChannel(PIN_RC_CH2),
          RCChannel(PIN_RC_CH3),
          RCChannel(PIN_RC_CH4),
          RCChannel(PIN_RC_CH5) 
        },
        _outputs{
          OutputController(PIN_OUT_1),  // inverted по умолчанию true
          OutputController(PIN_OUT_2),
          OutputController(PIN_OUT_3),
          OutputController(PIN_OUT_4),
          OutputController(PIN_OUT_5)
        }
    {}

    // Инициализация всех компонентов
    void begin() {
      _motorSensor1.begin();
      _motorSensor2.begin();
      for (uint8_t i = 0; i < 5; i++) {
        _rcChannels[i].begin();
        _outputs[i].begin();
      }
      Serial.begin(9600);
      Serial.println(F("System initialized"));
    }

    // Основной рабочий цикл: чтение входов -> логика -> управление выходами
    void update() {
      // --- Чтение аналоговых датчиков (с гистерезисом/порогом) ---
      float voltage1 = _motorSensor1.readVoltageFiltered();
      float voltage2 = _motorSensor2.readVoltageFiltered();

      // --- Чтение каналов приёмника ---
      int rc1 = _rcChannels[0].read();
      int rc2 = _rcChannels[1].read();
      int rc3 = _rcChannels[2].read();
      int rc4 = _rcChannels[3].read();
      int rc5 = _rcChannels[4].read();

      // ============================================
      // ВСТАВЬТЕ СЮДА СВОЮ ЛОГИКУ УПРАВЛЕНИЯ
      // ============================================
      // Пример (закомментировано):
      /*
      if (voltage1 > 0) {
        _outputs[0].on();
      } else {
        _outputs[0].off();
      }

      if (rc2 > 500) {
        _outputs[1].on();
      } else {
        _outputs[1].off();
      }
      // ... и так далее
      */

      // Пока для безопасности выключаем все выходы
      for (uint8_t i = 0; i < 5; i++) {
        _outputs[i].off();
      }

      // Небольшая пауза для стабильности
      delay(10);
    }
};

// --------------------- ГЛОБАЛЬНЫЙ ОБЪЕКТ ---------------------
MainController controller;

// --------------------- СТАНДАРТНЫЕ ФУНКЦИИ ARDUINO ---------------------
void setup() {
  controller.begin();
}

void loop() {
  controller.update();
}
