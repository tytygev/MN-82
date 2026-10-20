  // ============================================================
//  СКЕТЧ УПРАВЛЕНИЯ НАГРУЗКОЙ ПО СПАДУ НАПРЯЖЕНИЯ НА ПОЛЮСАХ МОТОРА
//  Один мотор, два аналоговых входа (полюса A и B),
//  одна тормозная нагрузка (при спаде на любом полюсе),
//  вторая нагрузка – по направлению вращения.
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

const unsigned long TACHO_STOP_THRESHOLD_MS = 300;   // нет импульсов дольше → стоим
const unsigned long BRAKE_OFF_DELAY_MS = 0;        // задержка OFF стоп-сигналов 
const unsigned long STOP_HOLD_MS = 120000;           // удержание стоп-сигнала после остановки (120 с)
const unsigned long REVERSE_HOLD_MS = 60000;         // удержание ЗХ после остановки (60 с)
const float BRAKE_DECEL_RATIO = 1.1;                 // замедление: интервал вырос в 1.5 раза → торможение

// Уровни для PNP-транзистора (BC327-40)
#define OUTPUT_ACTIVE   LOW
#define OUTPUT_INACTIVE HIGH

// --- Параметры мигания ---
const unsigned long BLINK1_ON_MS  = 350;  // время включения (мс)
const unsigned long BLINK1_OFF_MS = 350;  // время выключения (мс)

// === интервал вывода логов в монитор порта ===
const unsigned long LOG_INTERVAL_MS = 500;


// ==================== КЛАССЫ ====================

// ============================================================================
// Класс для чтения цифрового канала приёмника (RC)
class RCChannel {
  private:
    uint8_t _pin;

  public:
    RCChannel(uint8_t pin) : _pin(pin) {}
    void begin() {pinMode(_pin, INPUT); }
    bool read() {return digitalRead(_pin); }
};

// ============================================================================
// Тахометр на двух датчиках Холла
// Определяет направление по разнице интервалов A->B и B->A.
// Определяет торможение по резкому росту интервала между любыми импульсами.
class Tachometer {
  private:
    uint8_t _pinA, _pinB;
    bool _lastStateA, _lastStateB;
    unsigned long _timeA, _timeB;
    unsigned long _lastPulseTime;
    unsigned long _intervalAB, _intervalBA;
    unsigned long _lastInterval;
    int8_t _direction;   // 1 = Вперёд, -1 = Назад, 0 = Стоим
    bool _braking;

  public:
    Tachometer(uint8_t pinA, uint8_t pinB)
      : _pinA(pinA), _pinB(pinB),
        _lastStateA(true), _lastStateB(true),
        _timeA(0), _timeB(0), _lastPulseTime(0),
        _intervalAB(0), _intervalBA(0), _lastInterval(0),
        _direction(0), _braking(false) {}

    void begin() {
      // A3144 имеет открытый коллектор — нужен подтягивающий резистор.
      // Внутренний INPUT_PULLUP подойдёт, если провода не длинные.
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

      // Датчик A3144 тянет линию вниз при обнаружении магнита (активный LOW).
      // Ловим фронт HIGH -> LOW.
      bool edgeA = (a == LOW && _lastStateA == HIGH);
      bool edgeB = (b == LOW && _lastStateB == HIGH);

      if (edgeA) {
        if (_timeB > 0) {
          _intervalBA = now - _timeB;
          if (_intervalBA > 5 && _intervalBA < 1000) {
            // Если B->A короче, чем A->B, значит едем назад (датчики ближе друг к другу)
            if (_intervalBA < _intervalAB) _direction = -1;
            else _direction = 1;
          }
        }
        _timeA = now;
      }

      if (edgeB) {
        if (_timeA > 0) {
          _intervalAB = now - _timeA;
          if (_intervalAB > 5 && _intervalAB < 1000) {
            // Если A->B короче, значит едем вперёд
            if (_intervalAB < _intervalBA) _direction = 1;
            else _direction = -1;
          }
        }
        _timeB = now;
      }

      // Обработка торможения и остановки
      if (edgeA || edgeB) {
        unsigned long currentInterval = now - _lastPulseTime;
        if (_lastInterval > 0 && currentInterval < 1000) {
          // Если интервал резко вырос — значит мотор тормозит
          _braking = (currentInterval > _lastInterval * BRAKE_DECEL_RATIO);
        } else {
          _braking = false;
        }
        _lastInterval = currentInterval;
        _lastPulseTime = now;
      }

      _lastStateA = a;
      _lastStateB = b;

      // Если импульсов нет дольше порога — стоим
      if (now - _lastPulseTime > TACHO_STOP_THRESHOLD_MS) {
        _direction = 0;
        _braking = false;
        _lastInterval = 0;
        _intervalAB = 0;
        _intervalBA = 0;
      }
    }

    int8_t getDirection() const { return _direction; }
    bool isStopped() const { return _direction == 0; }
    bool isBraking() const { return _braking; }
    unsigned long getLastPulseAge() const { return millis() - _lastPulseTime; }
};

// ============================================================================
// Класс управления выходом
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

// ============================================================================
// Синхронный контроллер двух поворотников
// Один общий таймер — противофаза физически невозможна.
// Каждый поворотник включается независимо (рычагом) + есть общий режим "аварийка".
class DualBlinker {
  private:
    OutputController* _left;      // левый поворотник
    OutputController* _right;     // правый поворотник
    unsigned long _onTime;        // длительность свечения (мс)
    unsigned long _offTime;       // длительность паузы (мс)

    bool _leftEnabled;            // левый включён рычагом
    bool _rightEnabled;           // правый включён рычагом
    bool _hazardEnabled;          // включена аварийка

    unsigned long _lastToggle;    // момент последнего переключения
    bool _currentState;           // фаза: true=горит, false=пауза
    bool _timerRunning;           // работает ли таймер

  public:
    DualBlinker(OutputController* left, OutputController* right,
                unsigned long onMs, unsigned long offMs)
      : _left(left), _right(right),
        _onTime(onMs), _offTime(offMs),
        _leftEnabled(false), _rightEnabled(false), _hazardEnabled(false),
        _lastToggle(0), _currentState(false), _timerRunning(false) {}

    void begin() {
      _left->off();
      _right->off();
    }

    // Управление режимами. Только выставляют флаг — таймер подхватит в update().
    void enableLeft()   { _leftEnabled = true;   }
    void disableLeft()  { _leftEnabled = false;  }
    void enableRight()  { _rightEnabled = true;  }
    void disableRight() { _rightEnabled = false; }
    void enableHazard() { _hazardEnabled = true;  }
    void disableHazard(){ _hazardEnabled = false; }

    void update() {
      unsigned long now = millis();

      bool anythingEnabled = _leftEnabled || _rightEnabled || _hazardEnabled;

      // Ничего не включено — гасим оба выхода и сбрасываем таймер
      if (!anythingEnabled) {
        if (_timerRunning || _currentState) {
          _currentState = false;
          _timerRunning = false;
          _updateOutputs();
        }
        return;
      }

      // Первый запуск — стартуем с включённого состояния
      if (!_timerRunning) {
        _timerRunning = true;
        _currentState = true;
        _lastToggle = now;
        _updateOutputs();
        return;
      }

      // Продолжаем мигать
      unsigned long interval = _currentState ? _onTime : _offTime;
      if (now - _lastToggle >= interval) {
        _currentState = !_currentState;
        _lastToggle = now;
        _updateOutputs();
      }
    }

  private:
    void _updateOutputs() {
      // Левый горит, если (левый включён рычагом ИЛИ аварийка) И фаза = "горит"
      bool leftOn  = _currentState && (_leftEnabled  || _hazardEnabled);
      bool rightOn = _currentState && (_rightEnabled || _hazardEnabled);
      if (leftOn)  _left->on();  else _left->off();
      if (rightOn) _right->on(); else _right->off();
    }
};

// ============================================================================
// Контроллер огней заднего хода (на тахометре)
// Включает ЗХ, если тахометр зафиксировал вращение назад.
// Гасит сразу при переключении вперёд или через 60 с после остановки.
class ReverseLightController {
  private:
    OutputController* _output;
    Tachometer* _tacho;
    unsigned long _holdMs;
    unsigned long _lastReverseTime;
    bool _on;

  public:
    ReverseLightController(OutputController* output, Tachometer* tacho, unsigned long holdMs)
      : _output(output), _tacho(tacho), _holdMs(holdMs),
        _lastReverseTime(0), _on(false) {}

    void begin() {
      _output->off();
      _lastReverseTime = 0;
      _on = false;
    }

    void update() {
      unsigned long now = millis();
      int8_t dir = _tacho->getDirection();

      if (dir == -1) { // Едем назад
        _lastReverseTime = now;
        if (!_on) {
          _on = true;
          _output->on();
        }
      } else if (dir == 1) { // Едем вперёд
        if (_on) {
          _on = false;
          _output->off();
        }
      } else { // Стоим
        if (_on && (now - _lastReverseTime > _holdMs)) {
          _on = false;
          _output->off();
        }
      }
    }

    bool isOn() const { return _on; }
};

// ============================================================================
// Контроллер стоп-сигналов (на тахометре) ===
// Включает стоп при торможении или остановке.
// Держит включённым STOP_HOLD_MS после остановки (120 с).
// Выключает: при возобновлении движения (через offDelay) ИЛИ по истечении STOP_HOLD_MS.
class StopLightController {
  private:
    OutputController* _output;
    Tachometer* _tacho;
    unsigned long _offDelayMs;   // задержка OFF при возобновлении движения
    unsigned long _holdMs;        // максимальное удержание после остановки (120 с)

    unsigned long _stopStartTime; // момент, когда машина остановилась (один раз)
    bool _wasStopped;             // для отслеживания перехода moving->stopped
    bool _on;

  public:
    StopLightController(OutputController* output, Tachometer* tacho,
                        unsigned long offDelayMs, unsigned long holdMs)
      : _output(output), _tacho(tacho),
        _offDelayMs(offDelayMs), _holdMs(holdMs),
        _stopStartTime(0), _wasStopped(false), _on(false) {}

    void begin() {
      _output->off();
      _stopStartTime = 0;
      _wasStopped = false;
      _on = false;
    }
    void update() {
      unsigned long now = millis();
      bool stopped = _tacho->isStopped();
      bool braking = _tacho->isBraking();

      // === 1. Торможение — активное событие ===
      if (braking) {
        if (!_on) { _on = true; _output->on(); }
        // Начало остановки запомним в момент перехода в stopped
        _wasStopped = false;
        return;
      }

      // === 2. Стоим ===
      if (stopped) {
        // Переход moving->stopped: фиксируем момент остановки ОДИН РАЗ
        if (!_wasStopped) {
          _stopStartTime = now;
          _wasStopped = true;
        }
        if (!_on) { _on = true; _output->on(); }

        // Таймаут 120 с
        if (_on && (now - _stopStartTime > _holdMs)) {
          _on = false;
          _output->off();
        }
        return;
      }

      // === 3. Едем ===
      _wasStopped = false;
      if (_on && (now - _stopStartTime > _offDelayMs)) {
        _on = false;
        _output->off();
      }
    }

    bool isOn() const { return _on; }
};

// ============================================================================
// Главный контроллер
class MainController {
  private:
    RCChannel _rcChannels[5];
    OutputController _outputs[5];
    DualBlinker _blinkers;   // единый контроллер обоих поворотников
    Tachometer _tacho;                    // тахометр
    ReverseLightController _reverseLight;
    StopLightController _stopLight;
    bool _tfFlag = false;
    bool _tlFlag = false;
    bool _trFlag = false;
    bool _tlrFlag = false;
    bool _prevChannelState[5] = {false, false, false, false, false};
    unsigned long _lastLogTime = 0;   // момент последнего вывода логов

  public:
    MainController()
      : _tacho(PIN_HALL_A, PIN_HALL_B),
        _rcChannels{
          RCChannel(PIN_RC_VT),
          RCChannel(PIN_RC_D0),
          RCChannel(PIN_RC_D1),
          RCChannel(PIN_RC_D2),
          RCChannel(PIN_RC_D3)
        },
        _outputs{
          OutputController(PIN_OUT_BRAKE),  // 0 – Стоп-сигналы
          OutputController(PIN_OUT_AUX),    // 1 – огни заднего хода
          OutputController(PIN_OUT_FL),      // 2 – птф
          OutputController(PIN_OUT_TL),      // 3 – ЛПП
          OutputController(PIN_OUT_TR)       // 4 – ППП
        },
        _blinkers(&_outputs[3], &_outputs[4], BLINK1_ON_MS, BLINK1_OFF_MS),
        _reverseLight(&_outputs[1], &_tacho, REVERSE_HOLD_MS),
        _stopLight(&_outputs[0], &_tacho, BRAKE_OFF_DELAY_MS, STOP_HOLD_MS)
    {}

    void begin() {
      _tacho.begin();
      for (uint8_t i = 0; i < 5; i++) {
        _rcChannels[i].begin();
        _outputs[i].begin();
      }
      _blinkers.begin();   // ЗАДАЧА В
      _reverseLight.begin();   // ЗАДАЧА Б
      _stopLight.begin();      // ЗАДАЧА А
      Serial.begin(115200);  // ЗАДАЧА А: увеличено, чтобы вывод логов не блокировал цикл
      Serial.println(F("System initialized"));
    }

    void update() {
            // Обновляем тахометр
      _tacho.update();

      // Управление стоп-фарами (по тахометру)
      _stopLight.update();

      // Управление огнями заднего хода (по тахометру)
      _reverseLight.update();

      // Обновляем генератор мигания (один на оба поворотника)
      _blinkers.update();

      // === логирование тахометра (после всех update, показывает актуальное состояние) ===
      unsigned long nowForLog = millis();
      if (nowForLog - _lastLogTime >= LOG_INTERVAL_MS) {
        _lastLogTime = nowForLog;
        Serial.print(F("DIR="));
        Serial.print((int)_tacho.getDirection());  // (int) обязательно: int8_t иначе печатается как ASCII-символ
        Serial.print(F(" BRK="));
        Serial.print(_tacho.isBraking() ? 1 : 0);
        Serial.print(F(" STOP="));
        Serial.print(_stopLight.isOn() ? 1 : 0);
        Serial.print(F(" REV="));
        Serial.println(_reverseLight.isOn() ? 1 : 0);
      }

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
            // Переключаем левый рычаг. Правый всегда отключаем — на авто
            // левый и правый не могут быть активны одновременно.
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

        case 4: // D3 – аварийная сигнализация (кнопка-переключатель)
            _tlrFlag = !_tlrFlag;
            if (_tlrFlag) {
                // Включаем аварийку. Флаги рычага не трогаем —
                // они восстановятся при выключении аварийки.
                _blinkers.enableHazard();
            } else {
                // Выключаем аварийку и восстанавливаем состояние по флагам рычага
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