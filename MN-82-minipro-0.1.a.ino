// ============================================================
//  СКЕТЧ УПРАВЛЕНИЯ НАГРУЗКОЙ ПО СПАДУ НАПРЯЖЕНИЯ НА ПОЛЮСАХ МОТОРА
//  Один мотор, два аналоговых входа (полюса A и B),
//  одна тормозная нагрузка (при спаде на любом полюсе),
//  вторая нагрузка – по направлению вращения.
// ============================================================

// ==================== НАСТРОЙКИ ====================
// --- Пины ---
const uint8_t PIN_ANALOG_POLE_A = A0;   // полюс A мотора
const uint8_t PIN_ANALOG_POLE_B = A1;   // полюс B мотора

const uint8_t PIN_RC_CH1 = 2;
const uint8_t PIN_RC_CH2 = 3;
const uint8_t PIN_RC_CH3 = 4;
const uint8_t PIN_RC_CH4 = 5;
const uint8_t PIN_RC_CH5 = 6;

const uint8_t PIN_OUT_BRAKE = 7;    // тормозная нагрузка (при спаде)
const uint8_t PIN_OUT_AUX   = 8;    // вторая нагрузка (по направлению)
const uint8_t PIN_OUT_3 = 9;
const uint8_t PIN_OUT_4 = 10;
const uint8_t PIN_OUT_5 = 11;

// --- Аналоговый делитель ---
const float DIVIDER_RATIO = 0.3197;   // R2/(R1+R2)
const float VREF = 5.0;

// --- Параметры детектора быстрого спада ---
const float MIN_DROP_RATE = 1.0;               // минимальная скорость падения, В/с
const unsigned long MIN_DROP_DURATION_MS = 500; // мин. длительность быстрого падения
const unsigned long MAX_LOAD_TIME_MS = 10000;   // макс. время удержания тормоза
const float CUTOFF_VOLTAGE = 0.5;               // порог отсечки (В)
const unsigned long CUTOFF_HOLD_MS = 10000;     // задержка после падения ниже порога
const unsigned long UPDATE_INTERVAL_MS = 20;    // период обновления детектора

// --- Порог для определения направления вращения ---
const float DIRECTION_THRESHOLD = 0.5;  // минимальная разность напряжений (В)

// --- Параметры RC-каналов (ШИМ) ---
const unsigned long MIN_PULSE = 1000;
const unsigned long MAX_PULSE = 2000;
const bool USE_PWM = false;

// Уровни для PNP-транзистора (BC327-40)
#define OUTPUT_ACTIVE   LOW
#define OUTPUT_INACTIVE HIGH

// ==================== КЛАССЫ ====================

// 1. Класс детектора быстрого спада напряжения
class VoltageDropDetector {
  private:
    uint8_t _pin;
    float   _dividerRatio;
    float   _vref;
    float   _minDropRate;
    unsigned long _minDropDuration;
    unsigned long _maxLoadTime;
    float   _cutoffVoltage;
    unsigned long _cutoffHoldTime;
    unsigned long _updateInterval;

    float _lastVoltage;
    unsigned long _lastTime;
    unsigned long _lastUpdate;
    unsigned long _dropDuration;
    bool _loadActive;
    unsigned long _loadStartTime;
    bool _cutoffEntered;
    unsigned long _cutoffEnterTime;

  public:
    VoltageDropDetector(uint8_t pin, float dividerRatio, float vref,
                        float minDropRate, unsigned long minDropDuration,
                        unsigned long maxLoadTime, float cutoffVoltage,
                        unsigned long cutoffHoldTime, unsigned long updateInterval)
      : _pin(pin), _dividerRatio(dividerRatio), _vref(vref),
        _minDropRate(minDropRate), _minDropDuration(minDropDuration),
        _maxLoadTime(maxLoadTime), _cutoffVoltage(cutoffVoltage),
        _cutoffHoldTime(cutoffHoldTime), _updateInterval(updateInterval),
        _lastVoltage(0), _lastTime(0), _lastUpdate(0),
        _dropDuration(0), _loadActive(false), _loadStartTime(0),
        _cutoffEntered(false), _cutoffEnterTime(0) {}

    void begin() {
      pinMode(_pin, INPUT);
      _lastVoltage = readVoltage();
      _lastTime = millis();
      _lastUpdate = _lastTime;
    }

    float readVoltage() {
      int raw = analogRead(_pin);
      float voltageAtPin = (raw * _vref) / 1023.0;
      return voltageAtPin / _dividerRatio;
    }

    void update() {
      unsigned long now = millis();
      if (now - _lastUpdate < _updateInterval) return;
      _lastUpdate = now;

      float voltage = readVoltage();
      float dt = (now - _lastTime) / 1000.0;
      float dv = voltage - _lastVoltage;
      float rate = dv / dt;

      // Проверка быстрого спада
      if (rate < -_minDropRate) {
        _dropDuration += (now - _lastTime);
      } else {
        _dropDuration = 0;
      }

      // Включение нагрузки при выполнении условий
      if (!_loadActive && _dropDuration >= _minDropDuration && voltage > _cutoffVoltage) {
        _loadActive = true;
        _loadStartTime = now;
        _cutoffEntered = false;
      }

      // Обработка выключения
      if (_loadActive) {
        if (rate >= 0) {
          _loadActive = false;
        } else if (now - _loadStartTime >= _maxLoadTime) {
          _loadActive = false;
        } else if (voltage < _cutoffVoltage) {
          if (!_cutoffEntered) {
            _cutoffEntered = true;
            _cutoffEnterTime = now;
          } else if (now - _cutoffEnterTime >= _cutoffHoldTime) {
            _loadActive = false;
          }
        }
        if (voltage >= _cutoffVoltage && _cutoffEntered) {
          _cutoffEntered = false;
        }
      }

      _lastVoltage = voltage;
      _lastTime = now;
    }

    bool isLoadActive() const { return _loadActive; }
    float getLastVoltage() const { return _lastVoltage; }
};

// 2. Класс для чтения канала приёмника (RC)
class RCChannel {
  private:
    uint8_t _pin;
    bool _usePWM;
    unsigned long _minPulse;
    unsigned long _maxPulse;

  public:
    RCChannel(uint8_t pin, bool usePWM=USE_PWM, unsigned long minPulse=MIN_PULSE, unsigned long maxPulse=MAX_PULSE)
      : _pin(pin), _usePWM(usePWM), _minPulse(minPulse), _maxPulse(maxPulse) {}

    void begin() { pinMode(_pin, INPUT); }

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
};

// 3. Класс управления выходом
class OutputController {
  private:
    uint8_t _pin;
    bool _inverted;

  public:
    OutputController(uint8_t pin, bool inverted = true)
      : _pin(pin), _inverted(inverted) {}

    void begin() {
      pinMode(_pin, OUTPUT);
      off();
    }

    void on()  { digitalWrite(_pin, _inverted ? LOW : HIGH); }
    void off() { digitalWrite(_pin, _inverted ? HIGH : LOW); }
    void set(bool state) { state ? on() : off(); }
};

// 4. Главный контроллер
class MainController {
  private:
    VoltageDropDetector _poleADetector;
    VoltageDropDetector _poleBDetector;
    RCChannel _rcChannels[5];
    OutputController _outputs[5];

  public:
    MainController()
      : _poleADetector(PIN_ANALOG_POLE_A, DIVIDER_RATIO, VREF,
                       MIN_DROP_RATE, MIN_DROP_DURATION_MS,
                       MAX_LOAD_TIME_MS, CUTOFF_VOLTAGE,
                       CUTOFF_HOLD_MS, UPDATE_INTERVAL_MS),
        _poleBDetector(PIN_ANALOG_POLE_B, DIVIDER_RATIO, VREF,
                       MIN_DROP_RATE, MIN_DROP_DURATION_MS,
                       MAX_LOAD_TIME_MS, CUTOFF_VOLTAGE,
                       CUTOFF_HOLD_MS, UPDATE_INTERVAL_MS),
        _rcChannels{
          RCChannel(PIN_RC_CH1),
          RCChannel(PIN_RC_CH2),
          RCChannel(PIN_RC_CH3),
          RCChannel(PIN_RC_CH4),
          RCChannel(PIN_RC_CH5)
        },
        _outputs{
          OutputController(PIN_OUT_BRAKE),  // 0 – тормозная нагрузка
          OutputController(PIN_OUT_AUX),    // 1 – вторая нагрузка (по направлению)
          OutputController(PIN_OUT_3),      // 2 – свободен
          OutputController(PIN_OUT_4),      // 3 – свободен
          OutputController(PIN_OUT_5)       // 4 – свободен
        }
    {}

    void begin() {
      _poleADetector.begin();
      _poleBDetector.begin();
      for (uint8_t i = 0; i < 5; i++) {
        _rcChannels[i].begin();
        _outputs[i].begin();
      }
      Serial.begin(9600);
      Serial.println(F("System initialized"));
    }

    void update() {
      // Обновляем детекторы спада
      _poleADetector.update();
      _poleBDetector.update();

      // --- Управление тормозной нагрузкой ---
      // Если спад обнаружен на любом полюсе – включаем тормоз
      if (_poleADetector.isLoadActive() || _poleBDetector.isLoadActive()) {
        _outputs[0].on();
      } else {
        _outputs[0].off();
      }

      // --- Управление второй нагрузкой (по направлению вращения) ---
      // Простая логика: сравниваем напряжения на полюсах.
      // Если разность превышает порог – мотор крутится в одну сторону,
      // иначе – в другую (или стоит).
      float voltageA = _poleADetector.getLastVoltage();
      float voltageB = _poleBDetector.getLastVoltage();
      float diff = voltageA - voltageB;

      // Здесь вы можете настроить, какое направление должно включать нагрузку.
      // В примере включаем, если напряжение на A значительно выше, чем на B.
      if (diff > DIRECTION_THRESHOLD) {
        _outputs[1].on();
      } else if (diff < -DIRECTION_THRESHOLD) {
        // Обратное направление – можно выключить или оставить выключенным
        _outputs[1].off();
      } else {
        // Мотор остановлен или разность мала – выключаем
        _outputs[1].off();
      }

      // Чтение RC-каналов (при необходимости)
      // ...

      // Небольшая пауза
      delay(1);
    }
};

// ==================== ГЛОБАЛЬНЫЙ ОБЪЕКТ ====================
MainController controller;

// ==================== СТАНДАРТНЫЕ ФУНКЦИИ ====================
void setup() {
  controller.begin();
}

void loop() {
  controller.update();
}