// ============================================================
//  СКЕТЧ УПРАВЛЕНИЯ НАГРУЗКОЙ
//  Архитектура: SOLID + Dependency Inversion (IOutput)
//  Интеграция: PowerManager (управление питанием)
// ============================================================

#include "PowerManager.h"

// ============================================================
//  НАСТРОЙКИ ПИНОВ
// ============================================================

// --- Входы от RC-модуля (пульт) ---
// VT — переменный канал (обычно стик газа)
// D0..D3 — кнопки/тумблеры на пульте
const uint8_t PIN_RC = 12;              // физический пин RC-модуля (VT)
const uint8_t PIN_RC_VT = PIN_RC;       // алиас для читаемости (VT = тот же пин)
const uint8_t PIN_RC_D0 = 10;           // D0 — противотуманки (тумблер)
const uint8_t PIN_RC_D1 = 11;           // D1 — левый поворотник
const uint8_t PIN_RC_D2 = 9;            // D2 — правый поворотник
const uint8_t PIN_RC_D3 = 8;            // D3 — аварийка

// --- Выходы на нагрузку (через PNP-транзисторы BC327-40) ---
const uint8_t PIN_OUT_BRAKE = 7;        // стоп-сигналы
const uint8_t PIN_OUT_AUX   = 5;        // огни заднего хода
const uint8_t PIN_OUT_FL    = 4;        // противотуманки (FL = front lights)
const uint8_t PIN_OUT_TL    = 3;        // левый поворотник (TL = turn left)
const uint8_t PIN_OUT_TR    = 2;        // правый поворотник (TR = turn right)

// --- Датчики Холла на валу (тахометр) ---
const uint8_t PIN_HALL_A = A1;          // датчик A
const uint8_t PIN_HALL_B = A2;          // датчик B
                                        // ВАЖНО: угол между датчиками НЕ 90° и НЕ 180°,
                                        // иначе направление вращения не определяется

// --- Пин удержания питания (PowerManager) ---
// Схема: этот пин управляет реле/транзистором, который подаёт питание на Arduino.
// При старте (кнопка питания) Arduino поднимает этот пин, фиксирует питание на себе,
// затем через timeoutMs бездействия опускает и питание отключается.
const uint8_t PIN_HOLD = 13;

// ============================================================
//  ПАРАМЕТРЫ ТАХОМЕТРА
// ============================================================

// --- Общие параметры ---
// STOP_THRESHOLD_MS — сколько миллисекунд должно не быть импульсов с датчиков,
//                     чтобы считать, что машина стоит. 
//                     300 мс — компромисс между «быстро реагирует» и «не дёргается на медленном ходу».
//                     Если на очень медленном ходу стоп-сигнал ложно загорается — увеличь до 500.
//                     Если стоп при остановке включается с заметной задержкой — уменьши до 200.
const unsigned long TACHO_STOP_THRESHOLD_MS = 300;

// MIN_INTERVAL_MS — минимальный валидный интервал между импульсами (мс).
//                  Импульсы короче этого считаются дребезгом и игнорируются.
//                  5 мс = 200 Гц максимум. Для реального мотора с редуктором это с запасом.
//                  Если увидишь, что быстрые импульсы «теряются» — уменьши до 2–3.
const unsigned long TACHO_MIN_INTERVAL_MS = 5;

// MAX_INTERVAL_MS — максимальный валидный интервал (мс).
//                  Интервалы длиннее = «машина почти остановилась», они не участвуют в определении направления.
//                  1000 мс = 1 Гц = очень медленное вращение.
const unsigned long TACHO_MAX_INTERVAL_MS = 1000;

// WARMUP_PULSES — сколько первых импульсов после старта пропустить,
//                 прежде чем искать торможение. Нужно, чтобы эталон интервалов «устоялся».
//                 При старте с места интервалы хаотичны, без прогрева возможны ложные срабатывания.
const unsigned long TACHO_WARMUP_PULSES = 3;

// --- Сглаживание (EMA = exponential moving average) ---
// INTERVAL_EMA — коэффициент сглаживания интервалов между импульсами (0..1).
//                0.3 = 30% веса новому значению, 70% — старому. 
//                Меньше → плавнее, но медленнее реагирует. Больше → резче, но может дёргаться.
const float TACHO_INTERVAL_EMA = 0.3;

// BASELINE_EMA — коэффициент адаптации эталона (0..1).
//                0.02 = очень медленно подстраивается под текущую скорость.
//                Нужен, чтобы при длительной езде на одной скорости эталон «дрейфовал» 
//                вслед за изменениями (температура, износ и т.п.).
const float TACHO_BASELINE_EMA = 0.02;

// ============================================================
//  ПАРАМЕТРЫ ДЕТЕКЦИИ ТОРМОЖЕНИЯ
// ============================================================

// DECEL_RATIO — во сколько раз должен вырасти интервал между импульсами,
//               чтобы это считалось торможением.
//               1.05 = рост на 5%. Очень чувствительно.
//               Если стоп-сигнал ложно загорается на стабильной езде — увеличь до 1.2–1.5.
//               Если на реальном торможении не срабатывает — уменьши до 1.02–1.05.
const float BRAKE_DECEL_RATIO = 1.05;

// STABILIZE_MS — через сколько миллисекунд «непрерывного торможения» 
//                признать, что это не торможение, а новая стабильная скорость, и сбросить эталон.
//                2000 мс = 2 секунды. Если дольше 2 с интервал растёт — это не торможение, а переключение скорости.
const unsigned long BRAKE_STABILIZE_MS = 2000;

// BASELINE_DEADBAND — мёртвая зона вокруг эталона (±10%), 
//                     в которой эталон адаптируется к текущему значению.
//                     Вне этой зоны (при резком изменении скорости) эталон НЕ трогается.
const float BRAKE_BASELINE_DEADBAND = 0.10;

// ============================================================
//  ПАРАМЕТРЫ СТОП-СИГНАЛОВ
// ============================================================

// BRAKE_OFF_DELAY_MS — задержка перед выключением стоп-сигнала после возобновления движения.
//                      0 = гаснет мгновенно, как только машина поехала.
//                      При тахометре дребезга нет, поэтому задержка не нужна.
//                      Увеличь до 100–200, если визуально хочется «плавнее» гасло.
const unsigned long BRAKE_OFF_DELAY_MS = 0;

// STOP_HOLD_MS — сколько миллисекунд стоп-сигнал горит после остановки.
//                120000 мс = 120 секунд. По истечении стоп гаснет, даже если машина стоит.
//                Чтобы снова зажечь — нужно поехать и снова остановиться.
//                Это защита от «забыл выключить».
const unsigned long STOP_HOLD_MS = 120000;

// ============================================================
//  ПАРАМЕТРЫ ОГНЕЙ ЗАДНЕГО ХОДА
// ============================================================

// REVERSE_HOLD_MS — сколько огни ЗХ горят после последнего подтверждения реверса.
//                   60000 мс = 60 секунд. По истечении гаснут, даже если машина стоит.
const unsigned long REVERSE_HOLD_MS = 60000;

// REVERSE_DEBOUNCE_MS — минимальная длительность сигнала «назад», 
//                       после которой огни ЗХ включаются.
//                       300 мс = 300 миллисекунд. Защита от случайного дребезга при пуске/остановке.
//                       Если ЗХ включается слишком поздно — уменьши до 100–150.
const unsigned long REVERSE_DEBOUNCE_MS = 300;

// ============================================================
//  ПАРАМЕТРЫ МИГАНИЯ ПОВОРОТНИКОВ
// ============================================================

// BLINK1_ON_MS — сколько миллисекунд поворотник горит.
// BLINK1_OFF_MS — сколько миллисекунд между вспышками.
// 350/350 = классический ритм, как на реальных авто.
// Меняй для настройки «темпа» моргания.
const unsigned long BLINK1_ON_MS  = 350;
const unsigned long BLINK1_OFF_MS = 350;

// ============================================================
//  ПАРАМЕТРЫ ЛОГИРОВАНИЯ
// ============================================================

// LOG_INTERVAL_MS — как часто (в мс) выводить строку в Serial-монитор.
//                   500 = 2 раза в секунду. Если хочешь больше деталей — уменьши до 200.
const unsigned long LOG_INTERVAL_MS = 500;

// ============================================================
//  POWERMANAGER — УПРАВЛЕНИЕ ПИТАНИЕМ
// ============================================================
// Отслеживает активность на указанных пинах. Если активности не было 
// дольше timeoutMs (5 минут), снимает питание с PIN_HOLD → Arduino выключается.

// Список пинов, изменение которых считается «активностью»:
//   PIN_RC — стик газа (активный HIGH)
//   PIN_HALL_A, PIN_HALL_B — датчики Холла (активный LOW, потому что open collector)
//
// ВАЖНО: кнопки D0–D3 в этот список НЕ входят. Если машина стоит и ты щёлкаешь
//        только поворотниками, таймер продолжит идти и через 5 минут питание упадёт.
//        Если это нежелательно — добавь PIN_RC_D0..D3 в этот массив.
const InputPin inputs[] = {
  {PIN_RC,     HIGH, INPUT},          // VT: активный HIGH, обычный INPUT
  {PIN_HALL_A, LOW,  INPUT_PULLUP},   // датчик A: активный LOW, подтяжка к +5В
  {PIN_HALL_B, LOW,  INPUT_PULLUP}    // датчик B: активный LOW, подтяжка к +5В
};
const uint8_t inputCount = sizeof(inputs) / sizeof(inputs[0]);

// Экземпляр PowerManager: 
//   пин удержания = PIN_HOLD,
//   активный уровень = HIGH (то есть HIGH удерживает питание),
//   массив входов, 
//   таймаут бездействия = 300000 мс = 5 минут.
PowerManager power(PIN_HOLD, HIGH, inputs, inputCount, 300000);

// Forward declaration: функция определена ниже, но используется в setup()
void onPowerOff();

// ============================================================
//  КЛАССЫ
// ============================================================

// ============================================================================
// 1. IOutput — абстрактный интерфейс для любого выхода.
//    Принцип Dependency Inversion: вся бизнес-логика (мигание, стопы, ЗХ) работает
//    с IOutput* и не знает, какое физическое железо за ним стоит.
//    Позволяет легко добавить Pca9685Output, MosfetOutput, RelayOutput и т.д.
// ============================================================================
class IOutput {
  public:
    virtual void on() = 0;              // включить нагрузку
    virtual void off() = 0;             // выключить нагрузку
    virtual void set(bool state) = 0;   // установить состояние (true=on, false=off)
    virtual ~IOutput() {}               // виртуальный деструктор — обязательно для полиморфизма
};

// ============================================================================
// 2. PnpOutput — конкретная реализация IOutput для PNP-транзистора BC327-40.
//    Активный уровень — LOW (транзистор открывается, когда на базе 0В).
// ============================================================================
class PnpOutput : public IOutput {
  private:
    uint8_t _pin;

  public:
    PnpOutput(uint8_t pin) : _pin(pin) {}

    void begin() {
      pinMode(_pin, OUTPUT);
      off();  // по умолчанию нагрузка выключена
    }

    void on() override  { digitalWrite(_pin, LOW);  }  // LOW → PNP открыт → нагрузка включена
    void off() override { digitalWrite(_pin, HIGH); }  // HIGH → PNP закрыт → нагрузка выключена
    void set(bool state) override { state ? on() : off(); }
};

// ============================================================================
// 3. RCChannel — обёртка над цифровым пином RC-модуля.
//    Умеет только читать состояние (HIGH/LOW) и настраивать пин.
// ============================================================================
class RCChannel {
  private:
    uint8_t _pin;

  public:
    RCChannel(uint8_t pin) : _pin(pin) {}
    void begin() { pinMode(_pin, INPUT); }
    bool read()  { return digitalRead(_pin); }
};

// ============================================================================
// 4. Tachometer — тахометр на двух датчиках Холла.
//    Определяет:
//      - направление вращения (вперёд/назад/стоим) — по сравнению интервалов A→B и B→A
//      - торможение — по резкому росту среднего интервала относительно эталона
//      - остановку — по отсутствию импульсов дольше stopThresholdMs
//    Все параметры передаются через конструктор, не тянет глобальные константы.
// ============================================================================
class Tachometer {
  private:
    uint8_t _pinA, _pinB;
    bool _lastStateA, _lastStateB;
    unsigned long _timeA, _timeB;             // момент последнего импульса на каждом датчике
    unsigned long _lastPulseTime;             // момент последнего импульса вообще
    unsigned long _intervalAB, _intervalBA;   // последние интервалы между A→B и B→A
    unsigned long _lastInterval;

    float _avgIntervalAB, _avgIntervalBA;     // сглаженные (EMA) интервалы
    float _baselineSum;                       // эталон: сумма средних при стабильной езде
    uint8_t _baselineCounter;                 // счётчик импульсов после старта (для прогрева)
    unsigned long _brakingStartTime;          // когда началось текущее «торможение»
    bool _wasStopped;                         // была ли машина в состоянии «стоим»

    int8_t _direction;   // -1 = назад, 0 = стоим, 1 = вперёд
    bool _braking;       // флаг «сейчас тормозим»

    // Копии параметров из конструктора (см. глобальные настройки)
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
      // A3144 — открытый коллектор, нужен pull-up
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

      // Ловим фронт HIGH→LOW (магнит подошёл к датчику, A3144 притянул линию к земле)
      bool edgeA = (a == LOW && _lastStateA == HIGH);
      bool edgeB = (b == LOW && _lastStateB == HIGH);

      // === Определение направления ===
      // При вращении вперёд интервал A→B отличается от B→A (если датчики не на 90°/180°).
      // Сравнивая их, узнаём направление.
      if (edgeA) {
        if (_timeB > 0) {
          _intervalBA = now - _timeB;
          if (_intervalBA > _minIntervalMs && _intervalBA < _maxIntervalMs) {
            if (_intervalBA < _intervalAB) _direction = -1;  // B→A короче → едем назад
            else _direction = 1;                              // B→A длиннее → едем вперёд
          }
        }
        _timeA = now;
      }

      if (edgeB) {
        if (_timeA > 0) {
          _intervalAB = now - _timeA;
          if (_intervalAB > _minIntervalMs && _intervalAB < _maxIntervalMs) {
            if (_intervalAB < _intervalBA) _direction = 1;   // A→B короче → вперёд
            else _direction = -1;                              // A→B длиннее → назад
          }
        }
        _timeB = now;
      }

      // === Детекция торможения ===
      // Работает по «эталону»: сглаженная сумма средних интервалов при стабильной езде.
      // Когда сумма вырастает больше эталона в decelRatio раз — это торможение.
      if (edgeA || edgeB) {
        _lastPulseTime = now;

        // Сглаживание интервалов через EMA
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

        // Первый импульс после старта: устанавливаем эталон
        if (_baselineSum == 0) {
          _baselineSum = avgSum;
          _baselineCounter = 0;
          _braking = false;
          return;
        }

        // Прогрев: пропускаем первые N импульсов, эталон не трогаем
        if (_baselineCounter < _warmupPulses) {
          _baselineCounter++;
          _braking = false;
          return;
        }

        float ratio = avgSum / _baselineSum;

        if (ratio > _decelRatio) {
          // Интервал вырос больше порога → торможение
          if (!_braking) _brakingStartTime = now;
          _braking = true;

          // Если «торможение» длится дольше stabilizeMs — это не торможение,
          // а просто езда на новой (меньшей) скорости. Сбрасываем эталон.
          if (now - _brakingStartTime > _stabilizeMs) {
            _braking = false;
            _baselineSum = avgSum;
            _brakingStartTime = 0;
          }
        } else {
          // Интервал не вырос → не тормозим
          _braking = false;
          _brakingStartTime = 0;

          // Медленно подстраиваем эталон, но только в узкой зоне ±deadband
          // (чтобы не «убегал» при смене скорости)
          if (ratio > (1.0 - _deadband) && ratio < (1.0 + _deadband)) {
            _baselineSum = _baselineSum * (1.0 - _baselineEma) + avgSum * _baselineEma;
          }
        }
      }

      _lastStateA = a;
      _lastStateB = b;

      // === Остановка ===
      // Импульсов нет дольше stopThresholdMs → стоим, сбрасываем всё
      if (now - _lastPulseTime > _stopThresholdMs) {
        _direction = 0;
        _braking = false;
        _lastInterval = 0;
        _intervalAB = 0;
        _intervalBA = 0;
        _wasStopped = true;
      } else {
        // Импульсы снова пошли после стоянки → сбрасываем эталон для чистого старта
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
// 5. DualBlinker — синхронный контроллер двух поворотников.
//    Один общий таймер на оба → противофаза невозможна.
//    Режимы: левый, правый, аварийка (все четыре одновременно).
// ============================================================================
class DualBlinker {
  private:
    IOutput* _left;
    IOutput* _right;
    unsigned long _onTime;
    unsigned long _offTime;

    bool _leftEnabled;      // левый включён рычагом
    bool _rightEnabled;     // правый включён рычагом
    bool _hazardEnabled;    // аварийка

    unsigned long _lastToggle;  // когда в последний раз переключали фазу
    bool _currentState;         // текущая фаза: true=горит, false=пауза
    bool _timerRunning;         // работает ли мигание сейчас

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

      // Ничего не включено — гасим оба выхода и останавливаем таймер
      if (!anythingEnabled) {
        if (_timerRunning || _currentState) {
          _currentState = false;
          _timerRunning = false;
          _updateOutputs();
        }
        return;
      }

      // Первый запуск — стартуем с «горит»
      if (!_timerRunning) {
        _timerRunning = true;
        _currentState = true;
        _lastToggle = now;
        _updateOutputs();
        return;
      }

      // Продолжаем мигать: если прошло время текущей фазы — переключаем
      unsigned long interval = _currentState ? _onTime : _offTime;
      if (now - _lastToggle >= interval) {
        _currentState = !_currentState;
        _lastToggle = now;
        _updateOutputs();
      }
    }

  private:
    void _updateOutputs() {
      // Левый горит, если (левый рычаг ИЛИ аварийка) И текущая фаза = «горит»
      bool leftOn  = _currentState && (_leftEnabled  || _hazardEnabled);
      bool rightOn = _currentState && (_rightEnabled || _hazardEnabled);
      if (leftOn)  _left->on();  else _left->off();
      if (rightOn) _right->on(); else _right->off();
    }
};

// ============================================================================
// 6. ReverseLightController — огни заднего хода.
//    Логика:
//      - DIR=-1 держится дольше debounceMs → включаем ЗХ
//      - DIR=1 (едем вперёд) → сразу гасим
//      - DIR=0 (стоим) → гасим через holdMs после последнего реверса
// ============================================================================
class ReverseLightController {
  private:
    IOutput* _output;
    Tachometer* _tacho;
    unsigned long _holdMs;         // удержание после остановки
    unsigned long _debounceMs;     // минимальная длительность реверса для включения
    unsigned long _reverseStart;   // когда начался текущий реверс
    unsigned long _lastReverseTime;// последний момент, когда был реверс
    bool _on;

  public:
    ReverseLightController(IOutput* output, Tachometer* tacho,
                           unsigned long holdMs, unsigned long debounceMs)
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
        // Реверс: ждём debounceMs, потом включаем
        if (_reverseStart == 0) _reverseStart = now;
        _lastReverseTime = now;
        if (!_on && (now - _reverseStart > _debounceMs)) {
          _on = true;
          _output->on();
        }
      } else if (dir == 1) {
        // Вперёд — гасим сразу
        _reverseStart = 0;
        if (_on) {
          _on = false;
          _output->off();
        }
      } else {
        // Стоим — гасим через holdMs
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
// 7. StopLightController — стоп-сигналы.
//    Логика:
//      - Торможение ИЛИ остановка → стоп ВКЛ
//      - Движение вперёд → стоп ВЫКЛ (через offDelayMs)
//      - 120 секунд после остановки → стоп ВЫКЛ (защита от «забыл»)
//      - После этого стоп не включится, пока машина снова не поедет
// ============================================================================
class StopLightController {
  private:
    IOutput* _output;
    Tachometer* _tacho;
    unsigned long _offDelayMs;      // задержка перед выключением при возобновлении движения
    unsigned long _holdMs;          // максимум удержания после остановки

    unsigned long _stopStartTime;   // когда началась текущая остановка
    bool _wasStopped;               // флаг «уже в состоянии стояния»
    bool _timeoutExpired;           // таймаут уже сработал — не включать до движения
    bool _on;

  public:
    StopLightController(IOutput* output, Tachometer* tacho,
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

      // 1. Торможение — приоритет №1, включаем сразу
      if (braking) {
        if (!_on) { _on = true; _output->on(); }
        _wasStopped = false;
        _timeoutExpired = false;
        return;
      }

      // 2. Движение вперёд — гасим
      int8_t dir = _tacho->getDirection();
      if (dir == 1) {
        if (_on) { _on = false; _output->off(); }
        _wasStopped = false;
        _timeoutExpired = false;
        return;
      }

      // 3. Стоим
      if (stopped) {
        if (!_wasStopped) {
          // Только что остановились — фиксируем момент
          _stopStartTime = now;
          _wasStopped = true;
        }
        // Таймаут уже сработал — не включаем, пока не поедем
        if (_timeoutExpired) {
          if (_on) { _on = false; _output->off(); }
          return;
        }
        // Держим стоп включённым
        if (!_on) { _on = true; _output->on(); }
        // Проверяем таймаут 120 с
        if (now - _stopStartTime > _holdMs) {
          _on = false;
          _output->off();
          _timeoutExpired = true;
        }
        return;
      }

      // 4. Промежуточное состояние (выбег) — гасим через offDelayMs
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
// 8. MainController — главный контроллер.
//    Собирает всё вместе: физические выходы, тахометр, RC-каналы, бизнес-логику.
// ============================================================================
class MainController {
  private:
    // Порядок объявления ВАЖЕН: физические объекты объявлены до указателей на них.
    PnpOutput _pnpOutputs[5];           // 5 физических PNP-выходов
    RCChannel _rcChannels[5];           // 5 RC-каналов (VT, D0, D1, D2, D3)
    Tachometer _tacho;                  // тахометр

    IOutput* _outputs[5];               // указатели на PnpOutput через интерфейс IOutput

    // Бизнес-логика (работает через IOutput*, не знает про PNP)
    DualBlinker _blinkers;
    ReverseLightController _reverseLight;
    StopLightController _stopLight;

    // Флаги состояний RC-кнопок
    bool _tfFlag = false;      // противотуманки
    bool _tlFlag = false;      // левый поворотник
    bool _trFlag = false;      // правый поворотник
    bool _tlrFlag = false;     // аварийка
    bool _prevChannelState[5] = {false, false, false, false, false};  // предыдущие состояния для детекции фронта

    unsigned long _lastLogTime = 0;

  public:
    MainController()
      // 1. Физические выходы
      : _pnpOutputs{
          PnpOutput(PIN_OUT_BRAKE),   // индекс 0: стопы
          PnpOutput(PIN_OUT_AUX),     // индекс 1: ЗХ
          PnpOutput(PIN_OUT_FL),      // индекс 2: противотуманки
          PnpOutput(PIN_OUT_TL),      // индекс 3: левый поворотник
          PnpOutput(PIN_OUT_TR)       // индекс 4: правый поворотник
        },
        // 2. RC-каналы
        _rcChannels{
          RCChannel(PIN_RC_VT),       // индекс 0: VT (переменный)
          RCChannel(PIN_RC_D0),       // индекс 1: D0
          RCChannel(PIN_RC_D1),       // индекс 2: D1
          RCChannel(PIN_RC_D2),       // индекс 3: D2
          RCChannel(PIN_RC_D3)        // индекс 4: D3
        },
        // 3. Тахометр (все 11 параметров)
        _tacho(PIN_HALL_A, PIN_HALL_B,
               TACHO_STOP_THRESHOLD_MS, BRAKE_DECEL_RATIO,
               TACHO_MIN_INTERVAL_MS, TACHO_MAX_INTERVAL_MS,
               TACHO_WARMUP_PULSES, TACHO_INTERVAL_EMA,
               TACHO_BASELINE_EMA, BRAKE_STABILIZE_MS,
               BRAKE_BASELINE_DEADBAND),
        // 4. Указатели на физические выходы через интерфейс
        _outputs{
          &_pnpOutputs[0],
          &_pnpOutputs[1],
          &_pnpOutputs[2],
          &_pnpOutputs[3],
          &_pnpOutputs[4]
        },
        // 5. Бизнес-логика (получает только IOutput*)
        _blinkers(_outputs[3], _outputs[4], BLINK1_ON_MS, BLINK1_OFF_MS),
        _reverseLight(_outputs[1], &_tacho, REVERSE_HOLD_MS, REVERSE_DEBOUNCE_MS),
        _stopLight(_outputs[0], &_tacho, BRAKE_OFF_DELAY_MS, STOP_HOLD_MS)
    {}

    void begin() {
      _tacho.begin();
      for (uint8_t i = 0; i < 5; i++) {
        _rcChannels[i].begin();
        _pnpOutputs[i].begin();
      }
      _blinkers.begin();
      _reverseLight.begin();
      _stopLight.begin();
      Serial.begin(115200);
      Serial.println(F("System initialized"));
    }

    void update() {
      // 1. Обновить тахометр (читать импульсы, считать направление/торможение)
      _tacho.update();

      // 2. Обновить контроллеры по тахометру
      _stopLight.update();
      _reverseLight.update();

      // 3. Обновить мигание
      _blinkers.update();

      // 4. Логирование (раз в LOG_INTERVAL_MS)
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

      // 5. Чтение RC-кнопок с детекцией фронта (нажатия)
      for (uint8_t i = 0; i < 5; i++) {
        bool current = _rcChannels[i].read();
        bool pressed = current && !_prevChannelState[i];   // был LOW, стал HIGH = нажатие
        _prevChannelState[i] = current;

        if (!pressed) continue;

        switch (i) {
          case 1: // D0 — противотуманки (тумблер: переключает флаг)
            _tfFlag = !_tfFlag;
            _outputs[2]->set(_tfFlag);
            break;

          case 2: // D1 — левый поворотник
            _tlFlag = !_tlFlag;
            if (_tlFlag) {
                _trFlag = false;                  // правый и левый — взаимоисключающие
                _blinkers.disableRight();
                _blinkers.enableLeft();
            } else {
                _blinkers.disableLeft();
            }
            break;

          case 3: // D2 — правый поворотник
            _trFlag = !_trFlag;
            if (_trFlag) {
                _tlFlag = false;
                _blinkers.disableLeft();
                _blinkers.enableRight();
            } else {
                _blinkers.disableRight();
            }
            break;

          case 4: // D3 — аварийка (тумблер)
            _tlrFlag = !_tlrFlag;
            if (_tlrFlag) {
                _blinkers.enableHazard();         // включаем аварийку, флаги рычагов не трогаем
            } else {
                _blinkers.disableHazard();
                // Восстанавливаем состояние по флагам рычагов
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

// ==================== onPowerOff — коллбэк PowerManager ====================
// Вызывается ПЕРЕД снятием питания с PIN_HOLD.
// Здесь можно: погасить все выходы, сохранить состояние в EEPROM, 
//              пикнуть зуммером и т.д.
// Сейчас пустая — при желании добавь сюда действия.
void onPowerOff() {
  // TODO: если хочешь что-то сделать при выключении — пиши здесь.
}

// ==================== СТАНДАРТНЫЕ ФУНКЦИИ ====================
void setup() {
  // ВАЖНО: PowerManager первым — он должен зафиксировать питание
  // раньше, чем пользователь отпустит кнопку включения.
  power.setShutdownCallback(onPowerOff);
  power.begin();

  // Потом — остальная инициализация
  controller.begin();
}

void loop() {
  // PowerManager обновляется всегда — следит за активностью и таймаутом
  power.update();

  // Основная логика
  controller.update();
}