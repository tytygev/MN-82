// ============================================================
//  СКЕТЧ УПРАВЛЕНИЯ НАГРУЗКОЙ
//  Архитектура: SOLID, Dependency Inversion (IOutput)
// ============================================================

// ==================== НАСТРОЙКИ ====================
// --- Пины ---
// входы от RC
const uint8_t PIN_RC_VT = 12;
const uint8_t PIN_RC_D0 = 10;//8
const uint8_t PIN_RC_D1 = 11;//9
const uint8_t PIN_RC_D2 = 9;//11
const uint8_t PIN_RC_D3 = 8;//10

// выходы на нагрузку
const uint8_t PIN_OUT_BRAKE = 7;    // тормоз
const uint8_t PIN_OUT_AUX   = 5;    // задний Ход
const uint8_t PIN_OUT_FL = 4;
const uint8_t PIN_OUT_TL = 3;
const uint8_t PIN_OUT_TR = 2;

// === Тахометр на двух датчиках Холла ===
const uint8_t PIN_HALL_A = A1; // Датчик A 
const uint8_t PIN_HALL_B = A2; // Датчик B 



// === Тахометр: общие ===
const unsigned long TACHO_STOP_THRESHOLD_MS = 300;   // нет импульсов дольше → стоим

const unsigned long TACHO_MIN_INTERVAL_MS = 5;
const unsigned long TACHO_MAX_INTERVAL_MS = 1000;
const unsigned long TACHO_WARMUP_PULSES = 3;

// === Тахометр: сглаживание ===
const float TACHO_INTERVAL_EMA = 0.3;     // вес нового значения интервала (0..1)
const float TACHO_BASELINE_EMA = 0.02;    // скорость адаптации эталона (0..1)

// === Тахометр: детекция торможения ===
const float BRAKE_DECEL_RATIO = 1.05;     // во сколько раз должен вырасти интервал
const unsigned long BRAKE_STABILIZE_MS = 2000;   // через сколько «торможения» сбросить эталон
const float BRAKE_BASELINE_DEADBAND = 0.10;      // ±10% мёртвая зона обновления эталона

// === Стоп-сигналы ===
const unsigned long BRAKE_OFF_DELAY_MS = 0;          // задержка OFF стоп-сигналов 
const unsigned long STOP_HOLD_MS = 120000;           // удержание стоп-сигнала после остановки (120 с)

// === Огни заднего хода ===
const unsigned long REVERSE_HOLD_MS = 60000;         // удержание ЗХ после остановки (60 с)
const unsigned long REVERSE_DEBOUNCE_MS = 300;       // минимальная длительность DIR=-1 для включения ЗХ




// --- Параметры мигания ---
const unsigned long BLINK1_ON_MS  = 350;
const unsigned long BLINK1_OFF_MS = 350;

// === интервал вывода логов в монитор порта ===
const unsigned long LOG_INTERVAL_MS = 500;


// ==================== КЛАССЫ ====================

// ============================================================================
// 1. Интерфейс вывода (Dependency Inversion Principle)
// Любой выход (PNP, MOSFET, PCA9685, ESP32) должен реализовать эти методы.
// ============================================================================
class IOutput {
  public:
    virtual void on() = 0;
    virtual void off() = 0;
    virtual void set(bool state) = 0;
    virtual ~IOutput() {}
};

// ============================================================================
// 2. Реализация для PNP-транзистора (BC327-40)
// ============================================================================
class PnpOutput : public IOutput {
  private:
    uint8_t _pin;

  public:
    PnpOutput(uint8_t pin) : _pin(pin) {}

    void begin() {
      pinMode(_pin, OUTPUT);
      off();
    }

    void on() override {
      // Активный LOW для PNP
      digitalWrite(_pin, LOW);
    }

    void off() override {
      // Неактивный HIGH для PNP
      digitalWrite(_pin, HIGH);
    }

    void set(bool state) override { state ? on() : off(); }
};

// ============================================================================
// 3. Класс для чтения цифрового канала приёмника (RC)
// ============================================================================
class RCChannel {
  private:
    uint8_t _pin;

  public:
    RCChannel(uint8_t pin) : _pin(pin) {}
    void begin() {pinMode(_pin, INPUT); }
    bool read() {return digitalRead(_pin); }
};

// ============================================================================
// 4. Тахометр на двух датчиках Холла
// Теперь независим от глобальных констант — пороги передаются в конструктор.
// ============================================================================
class Tachometer {
  private:
    uint8_t _pinA, _pinB;
    bool _lastStateA, _lastStateB;
    unsigned long _timeA, _timeB;
    unsigned long _lastPulseTime;
    unsigned long _intervalAB, _intervalBA;
    unsigned long _lastInterval;
    
    // Сглаженные средние для детекции торможения
    float _avgIntervalAB;
    float _avgIntervalBA;
    // Эталон для детекции торможения
    float _baselineSum;
    uint8_t _baselineCounter;
    unsigned long _brakingStartTime;
    bool _wasStopped;
    
    int8_t _direction;   // 1 = Вперёд, -1 = Назад, 0 = Стоим
    bool _braking;

    // Параметры, перенесённые из глобальной области
    unsigned long _stopThresholdMs;
    float _decelRatio;
    unsigned long _minIntervalMs;
    unsigned long _maxIntervalMs;
    unsigned long _warmupPulses;
    float _intervalEma;
    float _baselineEma;
    unsigned long _stabilizeMs;
    float _deadband;

  public:
    Tachometer(uint8_t pinA, uint8_t pinB,
               unsigned long stopThresholdMs, float decelRatio,
               unsigned long minIntervalMs, unsigned long maxIntervalMs,
               unsigned long warmupPulses, float intervalEma,
               float baselineEma, unsigned long stabilizeMs, float deadband)
      : _pinA(pinA), _pinB(pinB),
        _lastStateA(true), _lastStateB(true),
        _timeA(0), _timeB(0), _lastPulseTime(0),
        _intervalAB(0), _intervalBA(0), _lastInterval(0),
        _avgIntervalAB(0), _avgIntervalBA(0),
        _baselineSum(0), _baselineCounter(0), _brakingStartTime(0),
        _wasStopped(true),
        _direction(0), _braking(false),
        _stopThresholdMs(stopThresholdMs), _decelRatio(decelRatio),
        _minIntervalMs(minIntervalMs), _maxIntervalMs(maxIntervalMs),
        _warmupPulses(warmupPulses), _intervalEma(intervalEma),
        _baselineEma(baselineEma), _stabilizeMs(stabilizeMs),
        _deadband(deadband) {}

    void begin() {
      pinMode(_pinA, INPUT_PULLUP);
      pinMode(_pinB, INPUT_PULLUP);
      _lastStateA = digitalRead(_pinA);
      _lastStateB = digitalRead(_pinB);
      _lastPulseTime = millis();
    }

    void update() {
      bool a = digitalRead(_pinA);
      bool b = digitalRead(_pinB);
      unsigned long now = millis();

      bool edgeA = (a == LOW && _lastStateA == HIGH);
      bool edgeB = (b == LOW && _lastStateB == HIGH);

      if (edgeA) {
        if (_timeB > 0) {
          _intervalBA = now - _timeB;
          if (_intervalBA > _minIntervalMs && _intervalBA < _maxIntervalMs) {
            if (_intervalBA < _intervalAB) _direction = -1;
            else _direction = 1;
          }
        }
        _timeA = now;
      }

      if (edgeB) {
        if (_timeA > 0) {
          _intervalAB = now - _timeA;
          if (_intervalAB > _minIntervalMs && _intervalAB < _maxIntervalMs) {
            if (_intervalAB < _intervalBA) _direction = 1;
            else _direction = -1;
          }
        }
        _timeB = now;
      }
      
      // Обработка торможения
      if (edgeA || edgeB) {
        _lastPulseTime = now;

        if (_intervalAB > 0) {
          if (_avgIntervalAB == 0) _avgIntervalAB = _intervalAB;
          else _avgIntervalAB = _avgIntervalAB * (1.0 - _intervalEma) + _intervalAB * _intervalEma;
        }
        if (_intervalBA > 0) {
          if (_avgIntervalBA == 0) _avgIntervalBA = _intervalBA;
          else _avgIntervalBA = _avgIntervalBA * (1.0 - _intervalEma) + _intervalBA * _intervalEma;
        }

        float avgSum = _avgIntervalAB + _avgIntervalBA;
        if (avgSum <= 0) return;

        if (_baselineSum == 0) {
          _baselineSum = avgSum;
          _baselineCounter = 0;
          _braking = false;
          return;
        }

        if (_baselineCounter < _warmupPulses) {
          _baselineCounter++;
          _braking = false;
          return;
        }

        float ratio = avgSum / _baselineSum;

        if (ratio > _decelRatio) { // Используем _decelRatio из конструктора
          if (!_braking) {
            _brakingStartTime = now;
          }
          _braking = true;

          if (now - _brakingStartTime > _stabilizeMs) {
            _braking = false;
            _baselineSum = avgSum;
            _brakingStartTime = 0;
          }
        } else {
          _braking = false;
          _brakingStartTime = 0;

          if (ratio > (1.0 - _deadband) && ratio < (1.0 + _deadband)) {
            _baselineSum = _baselineSum * (1.0 - _baselineEma) + avgSum * _baselineEma;
          }
        }
      }

      _lastStateA = a;
      _lastStateB = b;

      if (now - _lastPulseTime > _stopThresholdMs) { // Используем _stopThresholdMs
        _direction = 0;
        _braking = false;
        _lastInterval = 0;
        _intervalAB = 0;
        _intervalBA = 0;
        _wasStopped = true;
      } else {
        if (_wasStopped) {
          _baselineSum = 0;
          _baselineCounter = 0;
          _avgIntervalAB = 0;
          _avgIntervalBA = 0;
          _braking = false;
          _brakingStartTime = 0;
          _wasStopped = false;
        }
      }
    }

    int8_t getDirection() const { return _direction; }
    bool isStopped() const { return _direction == 0; }
    bool isBraking() const { return _braking; }
    unsigned long getLastPulseAge() const { return millis() - _lastPulseTime; }
    unsigned long getIntervalAB() const { return _intervalAB; }
    unsigned long getIntervalBA() const { return _intervalBA; }
};

// ============================================================================
// 5. Синхронный контроллер двух поворотников
// Работает через интерфейс IOutput
// ============================================================================
class DualBlinker {
  private:
    IOutput* _left;
    IOutput* _right;
    unsigned long _onTime;
    unsigned long _offTime;

    bool _leftEnabled;
    bool _rightEnabled;
    bool _hazardEnabled;

    unsigned long _lastToggle;
    bool _currentState;
    bool _timerRunning;

  public:
    DualBlinker(IOutput* left, IOutput* right, unsigned long onMs, unsigned long offMs)
      : _left(left), _right(right),
        _onTime(onMs), _offTime(offMs),
        _leftEnabled(false), _rightEnabled(false), _hazardEnabled(false),
        _lastToggle(0), _currentState(false), _timerRunning(false) {}

    void begin() {
      _left->off();
      _right->off();
    }

    void enableLeft()   { _leftEnabled = true;   }
    void disableLeft()  { _leftEnabled = false;  }
    void enableRight()  { _rightEnabled = true;  }
    void disableRight() { _rightEnabled = false; }
    void enableHazard() { _hazardEnabled = true;  }
    void disableHazard(){ _hazardEnabled = false; }

    void update() {
      unsigned long now = millis();
      bool anythingEnabled = _leftEnabled || _rightEnabled || _hazardEnabled;

      if (!anythingEnabled) {
        if (_timerRunning || _currentState) {
          _currentState = false;
          _timerRunning = false;
          _updateOutputs();
        }
        return;
      }

      if (!_timerRunning) {
        _timerRunning = true;
        _currentState = true;
        _lastToggle = now;
        _updateOutputs();
        return;
      }

      unsigned long interval = _currentState ? _onTime : _offTime;
      if (now - _lastToggle >= interval) {
        _currentState = !_currentState;
        _lastToggle = now;
        _updateOutputs();
      }
    }

  private:
    void _updateOutputs() {
      bool leftOn  = _currentState && (_leftEnabled  || _hazardEnabled);
      bool rightOn = _currentState && (_rightEnabled || _hazardEnabled);
      if (leftOn)  _left->on();  else _left->off();
      if (rightOn) _right->on(); else _right->off();
    }
};

// ============================================================================
// 6. Контроллер огней заднего хода (на тахометре)
// Работает через интерфейс IOutput, получает debounceMs в конструктор
// ============================================================================
class ReverseLightController {
  private:
    IOutput* _output;
    Tachometer* _tacho;
    unsigned long _holdMs;
    unsigned long _debounceMs;
    unsigned long _reverseStart;
    unsigned long _lastReverseTime;
    bool _on;

  public:
    ReverseLightController(IOutput* output, Tachometer* tacho, unsigned long holdMs, unsigned long debounceMs)
      : _output(output), _tacho(tacho), _holdMs(holdMs), _debounceMs(debounceMs),
        _reverseStart(0), _lastReverseTime(0), _on(false) {}

    void begin() {
      _output->off();
      _reverseStart = 0;
      _lastReverseTime = 0;
      _on = false;
    }

    void update() {
      unsigned long now = millis();
      int8_t dir = _tacho->getDirection();

      if (dir == -1) {
        if (_reverseStart == 0) {
          _reverseStart = now;
        }
        _lastReverseTime = now;

        if (!_on && (now - _reverseStart > _debounceMs)) {
          _on = true;
          _output->on();
        }
      } else if (dir == 1) {
        _reverseStart = 0;
        if (_on) {
          _on = false;
          _output->off();
        }
      } else {
        _reverseStart = 0;
        if (_on && (now - _lastReverseTime > _holdMs)) {
          _on = false;
          _output->off();
        }
      }
    }

    bool isOn() const { return _on; }
};

// ============================================================================
// 7. Контроллер стоп-сигналов (на тахометре)
// Работает через интерфейс IOutput
// ============================================================================
class StopLightController {
  private:
    IOutput* _output;
    Tachometer* _tacho;
    unsigned long _offDelayMs;
    unsigned long _holdMs;

    unsigned long _stopStartTime;
    bool _wasStopped;
    bool _timeoutExpired;
    bool _on;

  public:
    StopLightController(IOutput* output, Tachometer* tacho, unsigned long offDelayMs, unsigned long holdMs)
      : _output(output), _tacho(tacho),
        _offDelayMs(offDelayMs), _holdMs(holdMs),
        _stopStartTime(0), _wasStopped(false),
        _timeoutExpired(false), _on(false) {}

    void begin() {
      _output->off();
      _stopStartTime = 0;
      _wasStopped = false;
      _timeoutExpired = false;
      _on = false;
    }

    void update() {
      unsigned long now = millis();
      bool stopped = _tacho->isStopped();
      bool braking = _tacho->isBraking();

      if (braking) {
        if (!_on) { _on = true; _output->on(); }
        _wasStopped = false;
        _timeoutExpired = false;
        return;
      }

      int8_t dir = _tacho->getDirection();
      if (dir == 1) {
        if (_on) { _on = false; _output->off(); }
        _wasStopped = false;
        _timeoutExpired = false;
        return;
      }

      if (stopped) {
        if (!_wasStopped) {
          _stopStartTime = now;
          _wasStopped = true;
        }

        if (_timeoutExpired) {
          if (_on) { _on = false; _output->off(); }
          return;
        }

        if (!_on) { _on = true; _output->on(); }

        if (now - _stopStartTime > _holdMs) {
          _on = false;
          _output->off();
          _timeoutExpired = true;
        }
        return;
      }

      _wasStopped = false;
      _timeoutExpired = false;

      if (_on && (now - _stopStartTime > _offDelayMs)) {
        _on = false;
        _output->off();
      }
    }

    bool isOn() const { return _on; }
};

// ============================================================================
// 8. Главный контроллер
// Хранит физические объекты PnpOutput, а логике передаёт указатели IOutput*
// ============================================================================
class MainController {
  private:
    // 1. Сначала создаём физические объекты (они должны быть объявлены до указателей)
    PnpOutput _pnpOutputs[5];
    RCChannel _rcChannels[5];
    Tachometer _tacho;
    
    // 2. Массив указателей на интерфейс для передачи в бизнес-логику
    IOutput* _outputs[5];

    // 3. Бизнес-логика
    DualBlinker _blinkers;
    ReverseLightController _reverseLight;
    StopLightController _stopLight;
    
    // 4. Флаги состояний RC
    bool _tfFlag = false;
    bool _tlFlag = false;
    bool _trFlag = false;
    bool _tlrFlag = false;
    bool _prevChannelState[5] = {false, false, false, false, false};
    unsigned long _lastLogTime = 0;

  public:
    MainController()
      // Инициализация физического вывода
      : _pnpOutputs{
          PnpOutput(PIN_OUT_BRAKE),
          PnpOutput(PIN_OUT_AUX),
          PnpOutput(PIN_OUT_FL),
          PnpOutput(PIN_OUT_TL),
          PnpOutput(PIN_OUT_TR)
        },
        _rcChannels{
          RCChannel(PIN_RC_VT),
          RCChannel(PIN_RC_D0),
          RCChannel(PIN_RC_D1),
          RCChannel(PIN_RC_D2),
          RCChannel(PIN_RC_D3)
        },
        // Тахометр получает свои параметры
        _tacho(PIN_HALL_A, PIN_HALL_B,
               TACHO_STOP_THRESHOLD_MS, BRAKE_DECEL_RATIO,
               TACHO_MIN_INTERVAL_MS, TACHO_MAX_INTERVAL_MS,
               TACHO_WARMUP_PULSES, TACHO_INTERVAL_EMA,
               TACHO_BASELINE_EMA, BRAKE_STABILIZE_MS,
               BRAKE_BASELINE_DEADBAND),
			   
        // Привязка интерфейсов к физическим объектам
        _outputs{
          &_pnpOutputs[0],
          &_pnpOutputs[1],
          &_pnpOutputs[2],
          &_pnpOutputs[3],
          &_pnpOutputs[4]
        },
        
        // Логика получает только абстрактные указатели IOutput*
        _blinkers(_outputs[3], _outputs[4], BLINK1_ON_MS, BLINK1_OFF_MS),
        _reverseLight(_outputs[1], &_tacho, REVERSE_HOLD_MS, REVERSE_DEBOUNCE_MS),
        _stopLight(_outputs[0], &_tacho, BRAKE_OFF_DELAY_MS, STOP_HOLD_MS)
    {}

    void begin() {
      _tacho.begin();
      for (uint8_t i = 0; i < 5; i++) {
        _rcChannels[i].begin();
        _pnpOutputs[i].begin(); // Инициализируем физические пины
      }
      _blinkers.begin();
      _reverseLight.begin();
      _stopLight.begin();
      Serial.begin(115200);
      Serial.println(F("System initialized"));
    }

    void update() {
      _tacho.update();
      _stopLight.update();
      _reverseLight.update();
      _blinkers.update();

      // === Логирование ===
      unsigned long nowForLog = millis();
      if (nowForLog - _lastLogTime >= LOG_INTERVAL_MS) {
        _lastLogTime = nowForLog;
        Serial.print(F("A="));   Serial.print(digitalRead(PIN_HALL_A));
        Serial.print(F(" B="));   Serial.print(digitalRead(PIN_HALL_B));
        Serial.print(F(" DIR=")); Serial.print((int)_tacho.getDirection());
        Serial.print(F(" BRK=")); Serial.print(_tacho.isBraking() ? 1 : 0);
        Serial.print(F(" STOP="));Serial.print(_stopLight.isOn() ? 1 : 0);
        Serial.print(F(" REV=")); Serial.print(_reverseLight.isOn() ? 1 : 0);
        Serial.print(F(" AB="));  Serial.print(_tacho.getIntervalAB());
        Serial.print(F(" BA="));  Serial.print(_tacho.getIntervalBA());
        Serial.print(F(" P7="));  Serial.print(digitalRead(PIN_OUT_BRAKE));
        Serial.print(F(" P5="));  Serial.println(digitalRead(PIN_OUT_AUX));
      }

      // === Чтение RC ===
      for (uint8_t i = 0; i < 5; i++) {
        bool current = _rcChannels[i].read();
        bool pressed = current && !_prevChannelState[i];
        _prevChannelState[i] = current;

        if (!pressed) continue;

        switch (i) {
          case 1: // D0 – противотуманки
            _tfFlag = !_tfFlag;
            _outputs[2]->set(_tfFlag); // Работаем через интерфейс
            break;

          case 2: // D1 – левый поворотник
            _tlFlag = !_tlFlag;
            if (_tlFlag) {
                _trFlag = false;
                _blinkers.disableRight();
                _blinkers.enableLeft();
            } else {
                _blinkers.disableLeft();
            }
            break;

          case 3: // D2 – правый поворотник
            _trFlag = !_trFlag;
            if (_trFlag) {
                _tlFlag = false;
                _blinkers.disableLeft();
                _blinkers.enableRight();
            } else {
                _blinkers.disableRight();
            }
            break;

          case 4: // D3 – аварийка
            _tlrFlag = !_tlrFlag;
            if (_tlrFlag) {
                _blinkers.enableHazard();
            } else {
                _blinkers.disableHazard();
                if (_tlFlag) {
                    _blinkers.enableLeft();
                    _blinkers.disableRight();
                } else if (_trFlag) {
                    _blinkers.enableRight();
                    _blinkers.disableLeft();
                } else {
                    _blinkers.disableLeft();
                    _blinkers.disableRight();
                }
            }
            break;
        }
      }
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