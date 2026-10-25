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
const unsigned long REVERSE_DEBOUNCE_MS = 300;  // минимальная длительность DIR=-1 для включения ЗХ
const float BRAKE_DECEL_RATIO = 1.05;                 // замедление: интервал вырос в 1.5 раза → торможение

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
    // NEW: сглаженные средние для детекции торможения
    float _avgIntervalAB;
    float _avgIntervalBA;
        // NEW: эталон для детекции торможения
    float _baselineSum;       // эталонная сумма средних (при стабильном движении)
    uint8_t _baselineCounter; // счётчик импульсов для обновления эталона
    unsigned long _brakingStartTime;   // NEW: когда началось торможение
    bool _wasStopped;   // NEW: флаг, что машина была остановлена
    int8_t _direction;   // 1 = Вперёд, -1 = Назад, 0 = Стоим
    bool _braking;

  public:
    Tachometer(uint8_t pinA, uint8_t pinB)
      : _pinA(pinA), _pinB(pinB),
        _lastStateA(true), _lastStateB(true),
        _timeA(0), _timeB(0), _lastPulseTime(0),
        _intervalAB(0), _intervalBA(0), _lastInterval(0),
        _avgIntervalAB(0), _avgIntervalBA(0),
        _baselineSum(0), _baselineCounter(0), _brakingStartTime(0),
        _wasStopped(true),
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
      // Обработка торможения: эталон + тренд.
      // _baselineSum — «нормальная» сумма средних при стабильном движении.
      // Если текущая сумма превышает эталон в BRAKE_DECEL_RATIO раз — торможение.
      if (edgeA || edgeB) {
        _lastPulseTime = now;

        // Обновляем сглаженные средние
        if (_intervalAB > 0) {
          if (_avgIntervalAB == 0) _avgIntervalAB = _intervalAB;
          else _avgIntervalAB = _avgIntervalAB * 0.7 + _intervalAB * 0.3;
        }
        if (_intervalBA > 0) {
          if (_avgIntervalBA == 0) _avgIntervalBA = _intervalBA;
          else _avgIntervalBA = _avgIntervalBA * 0.7 + _intervalBA * 0.3;
        }

        float avgSum = _avgIntervalAB + _avgIntervalBA;
        if (avgSum <= 0) return;

        // Инициализация эталона при первом стабильном движении
        if (_baselineSum == 0) {
          _baselineSum = avgSum;
          _baselineCounter = 0;
          _braking = false;
          return;
        }

        // Не проверяем торможение в первые 3 импульса после старта.
        // ВАЖНО: НЕ обновляем _baselineSum здесь — иначе эталон «убегает»
        // вверх при замедлении, и торможение не детектируется.
        if (_baselineCounter < 3) {
          _baselineCounter++;
          _braking = false;
          // _baselineSum остаётся тем, что был установлен при первом импульсе
          return;
        }

               // Торможение: сумма средних превысила эталон в BRAKE_DECEL_RATIO раз
        // === Детекция торможения ===
        // Эталон обновляется ТОЛЬКО когда avgSum стабилен (±10%).
        // Если avgSum растёт (замедление) — эталон НЕ трогаем.
        float ratio = avgSum / _baselineSum;

        if (ratio > BRAKE_DECEL_RATIO) {
          // Торможение: avgSum вырос больше порога
          if (!_braking) {
            _brakingStartTime = now;
          }
          _braking = true;

          // Если «торможение» длится >2 с и avgSum стабилизировался —
          // это новая скорость, сбрасываем BRK и обновляем эталон
          if (now - _brakingStartTime > 2000) {
            _braking = false;
            _baselineSum = avgSum;
            _brakingStartTime = 0;
          }
        } else {
          _braking = false;
          _brakingStartTime = 0;

          // Эталон обновляем ТОЛЬКО если avgSum стабилен (±10%)
          if (ratio > 0.9 && ratio < 1.1) {
            _baselineSum = _baselineSum * 0.98 + avgSum * 0.02;
          }
          // Если ratio < 0.9 — машина ускорилась, эталон не трогаем
          // (иначе эталон «убежит» вниз, и следующее торможение не сработает)
        }
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

        // Запоминаем, что машина стояла. Эталон НЕ сбрасываем сразу —
        // он нужен для детекции торможения при следующем движении.
        // Сброс эталона произойдёт при ПЕРВОМ импульсе после остановки.
        _wasStopped = true;
      } else {
        // Импульсы идут. Если до этого стояли — это старт с места.
        if (_wasStopped) {
          // Сбрасываем эталон и счётчик — начинаем «с чистого листа».
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
// Класс управления выходом
// Для PNP-транзисторов (inverted=true) ВСЕГДА держим пин в OUTPUT.
// off() = HIGH (5В) — как в проверочном скетче, который доказал работоспособность.
// on()  = LOW (0В).
class OutputController {
  private:
    uint8_t _pin;
    bool _inverted;

  public:
    OutputController(uint8_t pin, bool inverted = true)
      : _pin(pin), _inverted(inverted) {}

    void begin() {
      // pinMode вызывается ОДИН РАЗ при старте — как в тестовом скетче.
      pinMode(_pin, OUTPUT);
      off();
    }

    void on() {
      // Только digitalWrite, БЕЗ pinMode — чтобы не дёргать DDR на каждом цикле.
      digitalWrite(_pin, _inverted ? LOW : HIGH);
    }

    void off() {
      // Только digitalWrite, БЕЗ pinMode.
      digitalWrite(_pin, _inverted ? HIGH : LOW);
    }

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

class ReverseLightController {
  private:
    OutputController* _output;
    Tachometer* _tacho;
    unsigned long _holdMs;          // удержание после остановки (60 с)
    unsigned long _reverseStart;    // когда DIR=-1 начался
    unsigned long _lastReverseTime; // последний момент, когда DIR был -1
    bool _on;

  public:
    ReverseLightController(OutputController* output, Tachometer* tacho, unsigned long holdMs)
      : _output(output), _tacho(tacho), _holdMs(holdMs),
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
        // DIR=-1. Проверяем, как долго он держится.
        if (_reverseStart == 0) {
          _reverseStart = now;   // запоминаем начало
        }
        _lastReverseTime = now;

        // Включаем ЗХ только если DIR=-1 держится > REVERSE_DEBOUNCE_MS мс
        if (!_on && (now - _reverseStart > REVERSE_DEBOUNCE_MS)) {
          _on = true;
          _output->on();
        }
      } else if (dir == 1) {
        // Едем вперёд — сразу гасим
        _reverseStart = 0;
        if (_on) {
          _on = false;
          _output->off();
        }
      } else {
        // DIR=0 (стоим). Сбрасываем таймер _reverseStart.
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
// Контроллер стоп-сигналов (на тахометре) ===
// Включает стоп при торможении или остановке.
// Держит включённым STOP_HOLD_MS после остановки (120 с).
// Выключает: при возобновлении движения (через offDelay) ИЛИ по истечении STOP_HOLD_MS.
class StopLightController {
  private:
    OutputController* _output;
    Tachometer* _tacho;
    unsigned long _offDelayMs;
    unsigned long _holdMs;

    unsigned long _stopStartTime;
    bool _wasStopped;
    bool _timeoutExpired;   // NEW: таймаут уже сработал, не включать снова
    bool _on;

  public:
    StopLightController(OutputController* output, Tachometer* tacho,
                        unsigned long offDelayMs, unsigned long holdMs)
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

      // === 1. Торможение — САМОЕ ВАЖНОЕ, проверяем ПЕРВЫМ ===
      // Если тахометр зафиксировал замедление — включаем стоп,
      // даже если DIR=1 (машина ещё едет вперёд, но уже тормозит).
      if (braking) {
        if (!_on) { _on = true; _output->on(); }
        _wasStopped = false;
        _timeoutExpired = false;
        return;
      }

      // === 2. Если едем ВПЕРЁД и НЕ тормозим — гасим стоп мгновенно ===
      int8_t dir = _tacho->getDirection();
      if (dir == 1) {
        if (_on) { _on = false; _output->off(); }
        _wasStopped = false;
        _timeoutExpired = false;
        return;
      }

      // === 2. Стоим ===
      if (stopped) {
        if (!_wasStopped) {
          // Переход moving->stopped: фиксируем момент остановки ОДИН РАЗ
          _stopStartTime = now;
          _wasStopped = true;
        }

        // Если таймаут уже истёк — не включаем стоп, пока машина не поедет
        if (_timeoutExpired) {
          // Убедимся, что пин точно выключен (один раз)
          if (_on) { _on = false; _output->off(); }
          return;
        }

        // Ещё не истёк — держим стоп включённым
        if (!_on) { _on = true; _output->on(); }

        // Проверяем таймаут
        if (now - _stopStartTime > _holdMs) {
          _on = false;
          _output->off();
          _timeoutExpired = true;   // запоминаем: больше не включать до движения
        }
        return;
      }

      // === 3. Едем (DIR=0 или нейтраль) ===
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
        Serial.print(F("A="));
        Serial.print(digitalRead(PIN_HALL_A));
        Serial.print(F(" B="));
        Serial.print(digitalRead(PIN_HALL_B));
        Serial.print(F(" DIR="));
        Serial.print((int)_tacho.getDirection());
        Serial.print(F(" BRK="));
        Serial.print(_tacho.isBraking() ? 1 : 0);
        Serial.print(F(" STOP="));
        Serial.print(_stopLight.isOn() ? 1 : 0);
        Serial.print(F(" REV="));
        Serial.print(_reverseLight.isOn() ? 1 : 0);
        Serial.print(F(" AB="));
        Serial.print(_tacho.getIntervalAB());
        Serial.print(F(" BA="));
        Serial.println(_tacho.getIntervalBA());
        Serial.print(F(" P7="));
        Serial.print(digitalRead(PIN_OUT_BRAKE));
        Serial.print(F(" P5="));
        Serial.println(digitalRead(PIN_OUT_AUX));
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
