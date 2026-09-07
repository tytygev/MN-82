// ============================================================
//  СКЕТЧ УПРАВЛЕНИЯ НАГРУЗКОЙ ПО СПАДУ НАПРЯЖЕНИЯ НА ПОЛЮСАХ МОТОРА
//  Один мотор, два аналоговых входа (полюса A и B),
//  одна тормозная нагрузка (при спаде на любом полюсе),
//  вторая нагрузка – по направлению вращения.
// ============================================================

// ==================== НАСТРОЙКИ ====================
// --- Пины ---
const uint8_t PIN_ANALOG_POLE_A = A1;   // полюс A мотора
const uint8_t PIN_ANALOG_POLE_B = A2;   // полюс B мотора

const uint8_t PIN_RC_VT = 12;
const uint8_t PIN_RC_D0 = 8;
const uint8_t PIN_RC_D1 = 9;
const uint8_t PIN_RC_D2 = 11;
const uint8_t PIN_RC_D3 = 10;

const uint8_t PIN_OUT_BRAKE = 7;    // тормоз
const uint8_t PIN_OUT_AUX   = 5;    // задний Ход
const uint8_t PIN_OUT_FL = 4;
const uint8_t PIN_OUT_TL = 3;
const uint8_t PIN_OUT_TR = 2;

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
//const unsigned long MIN_PULSE = 1000;
//const unsigned long MAX_PULSE = 2000;
//const bool USE_PWM = false;

// Уровни для PNP-транзистора (BC327-40)
#define OUTPUT_ACTIVE   LOW
#define OUTPUT_INACTIVE HIGH

// --- Параметры мигания ---
const unsigned long BLINK1_ON_MS  = 350;  // время включения (мс)
const unsigned long BLINK1_OFF_MS = 350;  // время выключения (мс)


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
    _cutoffEntered = false;
  }

  // Обработка выключения
  if (_loadActive) {
    // Условие 1: напряжение перестало падать (rate >= 0)
    if (rate >= 0) {
      _loadActive = false;
    }
    // Условие 2: напряжение ниже порога и прошло время удержания
    else if (voltage < _cutoffVoltage) {
      if (!_cutoffEntered) {
        _cutoffEntered = true;
        _cutoffEnterTime = now;
      } else if (now - _cutoffEnterTime >= _cutoffHoldTime) {
        _loadActive = false;
      }
    }

    // Сброс таймера отсечки, если напряжение снова поднялось выше порога
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

// 2. Класс для чтения цифрового канала приёмника (RC)
//    Теперь только digitalRead, возвращает bool.
class RCChannel {
  private:
    uint8_t _pin;

  public:
    RCChannel(uint8_t pin) : _pin(pin) {}
    void begin() {pinMode(_pin, INPUT); }
    bool read() {return digitalRead(_pin); }
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

// Генератор импульсов (мигание)
class PulseGenerator {
  private:
    OutputController* _output;    // указатель на управляемый выход
    unsigned long _onTime;
    unsigned long _offTime;
    unsigned long _lastToggle;
    bool _currentState;
    bool _enabled;

  public:
    // конструктор
    PulseGenerator(OutputController* output, unsigned long onMs, unsigned long offMs)
      : _output(output), _onTime(onMs), _offTime(offMs),
        _lastToggle(0), _currentState(false), _enabled(false) {}

    void begin() {
      _lastToggle = millis();
      _output->off();
      _enabled = false; // или по умолчанию включено
    }

    void enable()  { _enabled = true;  _lastToggle = millis(); _output->off(); }
    void disable() { _enabled = false; _output->off(); }

    void update() {
      if (!_enabled) return;

      unsigned long now = millis();
      if (_currentState) {
        if (now - _lastToggle >= _onTime) {
          _currentState = false;
          _output->off();
          _lastToggle = now;
        }
      } else {
        if (now - _lastToggle >= _offTime) {
          _currentState = true;
          _output->on();
          _lastToggle = now;
        }
      }
    }
};
// 4. Главный контроллер
class MainController {
  private:
    VoltageDropDetector _poleADetector;
    VoltageDropDetector _poleBDetector;
    RCChannel _rcChannels[5];
    OutputController _outputs[5];
    PulseGenerator _blinkerL;
    PulseGenerator _blinkerR;
    bool _tfFlag = false;
    bool _tlFlag = false;
    bool _trFlag = false;
    bool _tlrFlag = false;
    bool _prevChannelState[5] = {false, false, false, false, false};

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
          RCChannel(PIN_RC_VT),
          RCChannel(PIN_RC_D0),
          RCChannel(PIN_RC_D1),
          RCChannel(PIN_RC_D2),
          RCChannel(PIN_RC_D3)
        },
        _outputs{
          OutputController(PIN_OUT_BRAKE),  // 0 – тормозная нагрузка
          OutputController(PIN_OUT_AUX),    // 1 – вторая нагрузка (по направлению)
          OutputController(PIN_OUT_FL),      // 2 – птф
          OutputController(PIN_OUT_TL),      // 3 – ЛПП
          OutputController(PIN_OUT_TR)       // 4 – ППП
        },
        _blinkerL(&_outputs[3], BLINK1_ON_MS, BLINK1_OFF_MS),
        _blinkerR(&_outputs[4], BLINK1_ON_MS, BLINK1_OFF_MS)
    {}

    void begin() {
      _poleADetector.begin();
      _poleBDetector.begin();
      for (uint8_t i = 0; i < 5; i++) {
        _rcChannels[i].begin();
        _outputs[i].begin();
      }
      _blinkerL.begin();
      _blinkerR.begin();
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
        _outputs[1].off();// Обратное направление – можно выключить или оставить выключенным
      } else {
        _outputs[1].off();// Мотор остановлен или разность мала – выключаем
      }

      // Обновляем генераторы мигания (независимо)
      _blinkerL.update();
      _blinkerR.update();

      // Чтение RC-каналов (при необходимости)

      for (uint8_t i = 0; i < 5; i++) {
        bool current = _rcChannels[i].read();
        bool pressed = current && !_prevChannelState[i]; // переход LOW -> HIGH
         _prevChannelState[i] = current;

        if (!pressed) continue; // реагируем только на нажатие кнопки

        switch (i) {
            case 1: // D0 – противотуманки (тумблер)
            _tfFlag = !_tfFlag;
            _outputs[2].set(_tfFlag);
            break;

        case 2: // D1 – левый поворотник (положение рычага)
            // Переключаем левый, при этом правый выключаем всегда
            _tlFlag = !_tlFlag;
            if (_tlFlag) {
                _trFlag = false;          // правый не может быть активен одновременно
                if (!_tlrFlag) _blinkerR.disable();// при включённой аварийке игнорируем рычаг
                if (!_tlrFlag) _blinkerL.enable();// при включённой аварийке игнорируем рычаг
            } else {
                if (!_tlrFlag) _blinkerL.disable();// при включённой аварийке игнорируем рычаг
            }
            break;

        case 3: // D2 – правый поворотник
            _trFlag = !_trFlag;
            if (_trFlag) {
                _tlFlag = false;
                if (!_tlrFlag) _blinkerL.disable();// при включённой аварийке игнорируем рычаг
                if (!_tlrFlag) _blinkerR.enable();// при включённой аварийке игнорируем рычаг
            } else {
                if (!_tlrFlag) _blinkerR.disable();// при включённой аварийке игнорируем рычаг
            }
            break;

        case 4: // D3 – аварийная сигнализация (кнопка-переключатель)
            _tlrFlag = !_tlrFlag;
            if (_tlrFlag) {
                // Включаем оба поворотника, запоминаем, что аварийка активна
                _blinkerL.enable();
                _blinkerR.enable();
                // Флаги рычага не сбрасываем – они сохранят положение рычага
            } else {
                // Выключаем аварийку и восстанавливаем состояние по флагам рычага
                if (_tlFlag) {
                    _blinkerL.enable();
                    _blinkerR.disable();
                } else if (_trFlag) {
                    _blinkerL.disable();
                    _blinkerR.enable();
                } else {
                    _blinkerL.disable();
                    _blinkerR.disable();
                }
            }
            break;
       


        

       }
      }
      
       // Небольшая пауза
       delay(10);
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