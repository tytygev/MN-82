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
const uint8_t PIN_RC_D0 = 10;//8
const uint8_t PIN_RC_D1 = 11;//9
const uint8_t PIN_RC_D2 = 9;//11
const uint8_t PIN_RC_D3 = 8;//10

const uint8_t PIN_OUT_BRAKE = 7;    // тормоз
const uint8_t PIN_OUT_AUX   = 5;    // задний Ход
const uint8_t PIN_OUT_FL = 4;
const uint8_t PIN_OUT_TL = 3;
const uint8_t PIN_OUT_TR = 2;

// --- Аналоговый делитель ---
const float DIVIDER_RATIO = 0.3197;   // R2/(R1+R2)
const float VREF = 5.0;

// --- Параметры детектора быстрого спада ---
const float MIN_DROP_RATE = 0.50;               // минимальная скорость падения, В/с
const unsigned long MIN_DROP_DURATION_MS = 50; // мин. длительность быстрого падения
const unsigned long MAX_LOAD_TIME_MS = 10000;   // макс. время удержания тормоза
const float CUTOFF_VOLTAGE = 0.5;               // порог отсечки (В)
const unsigned long CUTOFF_HOLD_MS = 10000;     // задержка после падения ниже порога
const unsigned long UPDATE_INTERVAL_MS = 10;    // период обновления детектора

// --- Порог для определения направления вращения ---
const float DIRECTION_THRESHOLD = 0.1;          // минимальная разность напряжений (В)
const unsigned long LAMP_HOLD_MS = 60000;       // ЗАДАЧА Б: удержание огней ЗХ после пропадания реверса (60 с)
const unsigned long LAMP_HYSTERESIS_MS = 500;   // ЗАДАЧА Б: игнорировать пропадания реверса короче этого времени

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

// === ЗАДАЧА А: интервал вывода логов в монитор порта ===
const unsigned long LOG_INTERVAL_MS = 500;

// === ЗАДАЧА А: параметры контроллера стоп-сигналов ===
const float STOP_RATE_THRESHOLD = 1.0;         // порог спада среднего (В/с)
                                                // ниже — ловим даже плавный сброс газа;
                                                // шум АЦП после усреднения ~0.008 В/с, запас 100x
const unsigned long STOP_OFF_DELAY_MS = 100;   // задержка перед выключением после прекращения торможения
const unsigned long STOP_TIMEOUT_MS = 120000;   // максимум удержания стоп-сигнала (120 с)
const float STOP_MOVE_THRESHOLD = 0.3;          // порог «машина движется» по мгновенному vmax (В)
const unsigned long STOP_SAMPLE_INTERVAL_MS = 100; // окно усреднения (мс)
                                                   // должно быть больше периода ШИМ,
                                                   // чтобы среднее не «дёргалось» на его паузах
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
    // === ЗАДАЧА А: переменные для диагностики ===
    float _lastRate;   // последняя вычисленная скорость изменения (В/с)
    float _peakMin;    // минимум напряжения с момента последнего resetPeaks()
    float _peakMax;    // максимум напряжения с момента последнего resetPeaks()

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
        _cutoffEntered(false), _cutoffEnterTime(0),
        _lastRate(0), _peakMin(999.0), _peakMax(0.0) {}

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
		_lastRate = rate;                            // ЗАДАЧА А: сохраняем для логирования
		if (voltage < _peakMin) _peakMin = voltage;  // ЗАДАЧА А: трекинг минимума
		if (voltage > _peakMax) _peakMax = voltage;  // ЗАДАЧА А: трекинг максимума
		
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
    // === ЗАДАЧА А: геттеры для диагностики ===
    float getLastRate() const { return _lastRate; }
    float getPeakMin() const { return _peakMin; }
    float getPeakMax() const { return _peakMax; }
    unsigned long getDropDuration() const { return _dropDuration; }
    void resetPeaks() { _peakMin = 999.0; _peakMax = 0.0; }  // вызывать после логирования
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

// === ЗАДАЧА В: Синхронный контроллер двух поворотников ===
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

// === ЗАДАЧА Б: Контроллер огней заднего хода ===
// Логика:
//   - diff > +threshold  (мотор крутится назад)   -> огни ВКЛ, сбрасываем таймер
//   - diff < -threshold  (мотор крутится вперёд)  -> огни ВЫКЛ немедленно
//   - промежуточное состояние (стоп или ШИМ-пауза):
//       * если прошло < _hysteresisMs с последнего реверса — продолжаем гореть
//         (защита от дребезга ШИМ на низкой скорости)
//       * иначе запускается 60-секундный таймер удержания
//       * по истечении _holdMs огни гаснут
class ReverseLightController {
  private:
    OutputController* _output;      // управляемый выход
    float _threshold;               // порог разности напряжений (В)
    unsigned long _holdMs;          // удержание после остановки (мс)
    unsigned long _hysteresisMs;    // игнорирование коротких провалов ШИМ (мс)

    enum State { OFF, REVERSE, HOLD };
    State _state;
    unsigned long _lastReverseSeen; // последний момент, когда diff > threshold

  public:
    ReverseLightController(OutputController* output, float threshold,
                           unsigned long holdMs, unsigned long hysteresisMs)
      : _output(output), _threshold(threshold),
        _holdMs(holdMs), _hysteresisMs(hysteresisMs),
        _state(OFF), _lastReverseSeen(0) {}

    void begin() {
      _output->off();
      _state = OFF;
      _lastReverseSeen = 0;
    }

    void update(float diff) {
      unsigned long now = millis();

      bool clearReverse = (diff >  _threshold);  // явный реверс
      bool clearForward = (diff < -_threshold);  // явный ход вперёд

      if (clearReverse) {
        _lastReverseSeen = now;  // каждое подтверждение реверса обновляет таймер
      }

      switch (_state) {
        case OFF:
          if (clearReverse) {
            _state = REVERSE;
            _output->on();
          }
          break;

        case REVERSE:
          if (clearForward) {
            // Резкая смена направления — гасим немедленно
            _state = OFF;
            _output->off();
          } else if (!clearReverse && (now - _lastReverseSeen > _hysteresisMs)) {
            // Сигнал реверса пропал и не вернулся за время гистерезиса
            // Переходим в режим удержания — огни продолжают гореть
            _state = HOLD;
          }
          // иначе — всё ещё в реверсе, оставляем гореть
          break;

        case HOLD:
          if (clearReverse) {
            // Мотор снова крутится назад
            _state = REVERSE;
          } else if (clearForward) {
            // Мотор крутится вперёд — гасим
            _state = OFF;
            _output->off();
          } else if (now - _lastReverseSeen > _hysteresisMs + _holdMs) {
            // 60 с истекли — гасим
            _state = OFF;
            _output->off();
          }
          // иначе — огни продолжают гореть в фоне
          break;
      }
    }
};

// === ЗАДАЧА А: Контроллер стоп-сигналов ===
// Вход — ШИМ. Peak-hold был слеп к скважности: пока ШИМ импульсирует,
// пик не меняется, и плавные изменения газа не видны.
// Решение: считаем СРЕДНЕЕ vmax за окно — оно линейно пропорционально
// скважности ШИМ, то есть средней мощности на моторе.
// Моментальное ON на спаде среднего, 1 с задержки OFF, таймаут 120 с.
class StopLightController {
  private:
    OutputController* _output;
    float _rateThreshold;             // порог по скорости спада среднего (В/с)
    unsigned long _offDelayMs;        // задержка перед выключением
    unsigned long _timeoutMs;         // максимальное удержание
    float _moveThreshold;             // порог «машина движется» (по мгновенному vmax)
    unsigned long _sampleIntervalMs;  // окно усреднения

    float _sumVmax;                   // накопление vmax в текущем окне
    unsigned long _countVmax;         // количество сэмплов в текущем окне
    float _lastAvg;                   // среднее предыдущего окна
    float _currentAvg;                // среднее текущего окна (последнее посчитанное)
    unsigned long _lastSampleTime;    // момент начала текущего окна
    unsigned long _lastBrakeTime;     // момент последнего подтверждённого торможения
    bool _on;

  public:
    StopLightController(OutputController* output, float rateThreshold,
                        unsigned long offDelayMs, unsigned long timeoutMs,
                        float moveThreshold, unsigned long sampleIntervalMs)
      : _output(output), _rateThreshold(rateThreshold),
        _offDelayMs(offDelayMs), _timeoutMs(timeoutMs),
        _moveThreshold(moveThreshold), _sampleIntervalMs(sampleIntervalMs),
        _sumVmax(0), _countVmax(0),
        _lastAvg(0), _currentAvg(0), _lastSampleTime(0),
        _lastBrakeTime(0), _on(false) {}

    void begin() {
      _output->off();
      _sumVmax = 0;
      _countVmax = 0;
      _lastAvg = 0;
      _currentAvg = 0;
      _lastSampleTime = millis();
      _lastBrakeTime = 0;
      _on = false;
    }

    void update(float voltageA, float voltageB) {
      unsigned long now = millis();
      float vmax = max(fabs(voltageA), fabs(voltageB));

      // === 1. Накопление для среднего ===
      _sumVmax += vmax;
      _countVmax++;

      // === 2. Конец окна: считаем среднее и его rate ===
      if (now - _lastSampleTime >= _sampleIntervalMs) {
        float avg = _sumVmax / (float)_countVmax;
        _currentAvg = avg;

        unsigned long dt = now - _lastSampleTime;
        float rate = (avg - _lastAvg) * 1000.0 / dt;

        // Реальное торможение: среднее упало ниже порога.
        // Даже плавный сброс газа даёт rate порядка −1…−4 В/с, что выше порога 1.0.
        if (rate < -_rateThreshold) {
          _lastBrakeTime = now;
          if (!_on) {
            _on = true;
            _output->on();
          }
        }

        // Сброс окна
        _lastAvg = avg;
        _sumVmax = 0;
        _countVmax = 0;
        _lastSampleTime = now;
      }

      // === 3. OFF-условие — проверяем каждый цикл ===
      // «Движется» — по мгновенному vmax: плотный опрос ловит даже короткие
      // импульсы ШИМ, поэтому ложных «стою» при движении не будет.
      if (_on) {
        bool offDelayPassed = (now - _lastBrakeTime > _offDelayMs);
        bool timeoutPassed  = (now - _lastBrakeTime > _timeoutMs);
        bool moving         = (vmax > _moveThreshold);

        if (offDelayPassed && (moving || timeoutPassed)) {
          _on = false;
          _output->off();
        }
      }
    }

    bool isOn() const { return _on; }
    // ЗАДАЧА А: диагностика
    float getCurrentAvg() const { return _currentAvg; }
    float getLastAvg() const { return _lastAvg; }
};

// 4. Главный контроллер
class MainController {
  private:
    VoltageDropDetector _poleADetector;
    VoltageDropDetector _poleBDetector;
    RCChannel _rcChannels[5];
    OutputController _outputs[5];
    DualBlinker _blinkers;   // ЗАДАЧА В: единый контроллер обоих поворотников
    ReverseLightController _reverseLight;   // ЗАДАЧА Б: контроллер огней ЗХ
    StopLightController _stopLight;         // ЗАДАЧА А: контроллер стоп-сигналов
    bool _tfFlag = false;
    bool _tlFlag = false;
    bool _trFlag = false;
    bool _tlrFlag = false;
    bool _prevChannelState[5] = {false, false, false, false, false};
    unsigned long _lastLogTime = 0;   // ЗАДАЧА А: момент последнего вывода логов

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
          OutputController(PIN_OUT_BRAKE),  // 0 – Стоп-сигналы
          OutputController(PIN_OUT_AUX),    // 1 – огни заднего хода
          OutputController(PIN_OUT_FL),      // 2 – птф
          OutputController(PIN_OUT_TL),      // 3 – ЛПП
          OutputController(PIN_OUT_TR)       // 4 – ППП
        },
        _blinkers(&_outputs[3], &_outputs[4], BLINK1_ON_MS, BLINK1_OFF_MS),
        _reverseLight(&_outputs[1], DIRECTION_THRESHOLD, LAMP_HOLD_MS, LAMP_HYSTERESIS_MS),
        _stopLight(&_outputs[0], STOP_RATE_THRESHOLD, STOP_OFF_DELAY_MS,
                   STOP_TIMEOUT_MS, STOP_MOVE_THRESHOLD, STOP_SAMPLE_INTERVAL_MS)
    {}

    void begin() {
      _poleADetector.begin();
      _poleBDetector.begin();
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
      // Обновляем детекторы спада
      _poleADetector.update();
      _poleBDetector.update();

      // --- Управление стоп-фарами (ЗАДАЧА А: моментальное ON, 1 с задержка OFF) ---
      // Логика вынесена в StopLightController. Работает по уровням vmax и rate,
      // моментально включает при спаде и держит после остановки до 120 с.
      _stopLight.update(_poleADetector.getLastVoltage(),
                        _poleBDetector.getLastVoltage());

      // === ЗАДАЧА А: подробное логирование детекторов ===
      // Каждые LOG_INTERVAL_MS выводим реальные значения rate и пиков напряжения.
      // Это позволит подобрать MIN_DROP_RATE осмысленно, а не наугад.
      unsigned long nowForLog = millis();
      if (nowForLog - _lastLogTime >= LOG_INTERVAL_MS) {
        _lastLogTime = nowForLog;

        Serial.print(F("A="));      Serial.print(_poleADetector.getLastVoltage(), 2);
        Serial.print(F(" pk["));     Serial.print(_poleADetector.getPeakMin(), 2);
        Serial.print('-');           Serial.print(_poleADetector.getPeakMax(), 2);
        Serial.print(F("] rate="));  Serial.print(_poleADetector.getLastRate(), 2);
        Serial.print(F(" drop="));   Serial.print(_poleADetector.getDropDuration());
        Serial.print(F(" act="));    Serial.print(_poleADetector.isLoadActive() ? 1 : 0);

        Serial.print(F(" || B="));   Serial.print(_poleBDetector.getLastVoltage(), 2);
        Serial.print(F(" pk["));     Serial.print(_poleBDetector.getPeakMin(), 2);
        Serial.print('-');           Serial.print(_poleBDetector.getPeakMax(), 2);
        Serial.print(F("] rate="));  Serial.print(_poleBDetector.getLastRate(), 2);
        Serial.print(F(" drop="));   Serial.print(_poleBDetector.getDropDuration());
        Serial.print(F(" act="));    Serial.print(_poleBDetector.isLoadActive() ? 1 : 0);

        Serial.print(F(" || SL avg=")); Serial.print(_stopLight.getCurrentAvg(), 2);
        Serial.print(F(" prev="));      Serial.print(_stopLight.getLastAvg(), 2);
        Serial.print(F(" STOP="));      Serial.println(_stopLight.isOn() ? 1 : 0);

        // Сбрасываем пики для следующего окна логирования
        _poleADetector.resetPeaks();
        _poleBDetector.resetPeaks();
      }

      // --- Управление огнями заднего хода (по направлению вращения) ---
      // ЗАДАЧА Б: вся логика вынесена в класс ReverseLightController.
      // Он сам решает, когда зажигать и когда гасить огни, с учётом
      // гистерезиса (защита от ШИМ) и 60-секундного удержания.
      float voltageA = _poleADetector.getLastVoltage();
      float voltageB = _poleBDetector.getLastVoltage();
      float diff = voltageA - voltageB;
      _reverseLight.update(diff);

      // Обновляем генератор мигания (ЗАДАЧА В: один на оба поворотника)
      _blinkers.update();

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