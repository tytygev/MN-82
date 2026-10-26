// ============================================================================
// PWMOutputController.cpp
// ============================================================================

#include "PWMOutputController.h"

// ----------------------------------------------------------------------------
// _isHardwarePwmPin
// Возвращает true, если пин имеет аппаратный ШИМ на ATmega328P
// (Pro Mini / Uno / Nano). Карта чипа, не наша прихоть:
//   Пины 3, 11 -> Timer2
//   Пины 5, 6  -> Timer0 (общий с millis(), но analogWrite() безопасен)
//   Пины 9, 10 -> Timer1
// ----------------------------------------------------------------------------
bool PWMOutputController::_isHardwarePwmPin(uint8_t pin) {
    switch (pin) {
        case 3:  return true;
        case 5:  return true;
        case 6:  return true;
        case 9:  return true;
        case 10: return true;
        case 11: return true;
        default: return false;
    }
}

// ----------------------------------------------------------------------------
// Конструктор — только запоминает параметры.
// Пин НЕ настраивается, это делает begin() в setup().
// ----------------------------------------------------------------------------
PWMOutputController::PWMOutputController(uint8_t pin,
                                         bool    inverted,
                                         bool    hasPwm,
                                         bool    verbose)
    : _pin(pin)
    , _inverted(inverted)
    , _hasPwm(hasPwm)
    , _verbose(verbose)
{
    // Все поля с инициализаторами по умолчанию уже заданы в заголовке.
}

// ----------------------------------------------------------------------------
// begin — инициализация пина, проверка PWM, установка «выключено»
// ----------------------------------------------------------------------------
void PWMOutputController::begin() {

    // Пин всегда выход — независимо от режима PWM.
    pinMode(_pin, OUTPUT);

    // Если пользователь запросил ШИМ, но пин его не поддерживает —
    // понижаем режим до бинарного и ГРОМКО предупреждаем.
    if (_hasPwm && !_isHardwarePwmPin(_pin)) {
        _hasPwm = false;          // фактический режим — бинарный

        if (_verbose) {
            Serial.println();
            Serial.println(F("========================================================"));
            Serial.println(F("!!!!!!   PWMOutputController WARNING   !!!!!!"));
            Serial.print  (F("!!!!!!   Pin "));
            Serial.print  (_pin);
            Serial.println(F(" has NO hardware PWM on this MCU."));
            Serial.println(F("!!!!!!   Falling back to BINARY ON/OFF mode."));
            Serial.println(F("!!!!!!   Change the pin, or pass hasPwm=false"));
            Serial.println(F("!!!!!!   explicitly to silence this message."));
            Serial.println(F("========================================================"));
            Serial.println();
        }
    }

    // Сброс состояния в «выключено».
    _target             = 0;
    _current            = 0;
    _startBrightness    = 0;
    _transitionActive   = false;
    _transitionDuration = 0;
    _lastWritten        = -1;     // заставит _write() реально записать пин

    _write(0);                    // физически выключаем выход

    _begun = true;
}

// ----------------------------------------------------------------------------
// Простые сеттеры — только сохраняют параметры для следующих переходов.
// ----------------------------------------------------------------------------
void PWMOutputController::setRiseTime(uint16_t ms) { _riseTimeMs = ms; }
void PWMOutputController::setFallTime(uint16_t ms) { _fallTimeMs = ms; }
void PWMOutputController::setCurve(Curve c)        { _curve     = c;  }

// ----------------------------------------------------------------------------
// setMaxBrightness — ограничивает «максимум» для on().
// Если текущая цель выше — плавно переходим к новому максимуму.
// ----------------------------------------------------------------------------
void PWMOutputController::setMaxBrightness(uint8_t v) {
    _maxBrightness = v;
    if (_target > _maxBrightness) {
        _startTransition(_maxBrightness);
    }
}

// ----------------------------------------------------------------------------
// Пользовательские команды — все идут через _startTransition()
// ----------------------------------------------------------------------------
void PWMOutputController::on()  { _startTransition(_maxBrightness); }
void PWMOutputController::off() { _startTransition(0); }

void PWMOutputController::set(bool state) {
    state ? on() : off();
}

void PWMOutputController::setBrightness(uint8_t v) {
    if (v > _maxBrightness) v = _maxBrightness;
    _startTransition(v);
}

// ----------------------------------------------------------------------------
// _startTransition — сердце библиотеки
//
//  - совпадает цель          -> ничего
//  - бинарный режим          -> мгновенная запись
//  - активный переход        -> фиксируем ТЕКУЩЕЕ значение как старт нового
//  - duration = delta × time / 255
// ----------------------------------------------------------------------------
void PWMOutputController::_startTransition(uint8_t newTarget) {

    // Уже едем туда — не перезапускаем.
    if (newTarget == _target) return;

    // Бинарный режим: плавность невозможна.
    if (!_hasPwm) {
        _target           = newTarget;
        _current          = newTarget;
        _transitionActive = false;
        _write(_current);
        return;
    }

    // Старт нового перехода — ТЕКУЩАЯ фактическая яркость.
    // Именно это даёт «догоняющее» поведение при прерывании:
    // если в момент t=10мс пришла новая команда, стартуем от того,
    // что успело накопиться, а не от нуля и не от старой цели.
    uint8_t from = _current;

    // Уже в этой точке — переходить некуда.
    if (newTarget == from) {
        _target           = newTarget;
        _transitionActive = false;
        return;
    }

    // Параметры нового перехода.
    _startBrightness = from;
    _target          = newTarget;
    _transitionStart = millis();

    // База: riseTime для нарастания, fallTime для затухания.
    uint16_t baseTime = (newTarget > from) ? _riseTimeMs : _fallTimeMs;

    // Длительность пропорциональна дельте.
    uint8_t  delta    = (newTarget > from) ? (newTarget - from) : (from - newTarget);
    uint32_t duration = (uint32_t)delta * baseTime / 255UL;

    if (duration > 65535UL) duration = 65535UL;
    _transitionDuration = (uint16_t)duration;

    // Нулевая длительность — мгновенная запись.
    if (_transitionDuration == 0) {
        _current          = newTarget;
        _transitionActive = false;
        _write(_current);
        return;
    }

    _transitionActive = true;
}

// ----------------------------------------------------------------------------
// _write — физическая запись в пин с учётом инверсии и режима.
// Подавляет повторные записи того же значения (экономия тактов).
// ----------------------------------------------------------------------------
void PWMOutputController::_write(uint8_t brightness) {

    // Подавление повторов.
    if (_lastWritten == (int16_t)brightness) return;
    _lastWritten = (int16_t)brightness;

    if (_hasPwm) {
        // ШИМ: brightness — «насколько ярко», OCR — «какая скважность».
        // Для PNP инверсия: brightness=255 → OCR=0 → LOW → транзистор открыт.
        uint8_t ocrValue = _inverted ? (uint8_t)(255 - brightness) : brightness;
        analogWrite(_pin, ocrValue);
    } else {
        // Бинарный режим: любой ненулевой уровень — «ВКЛ».
        bool logicalOn = (brightness > 0);
        bool pinHigh   = _inverted ? !logicalOn : logicalOn;
        digitalWrite(_pin, pinHigh ? HIGH : LOW);
    }
}

// ----------------------------------------------------------------------------
// _computeBrightness — промежуточная яркость в момент elapsed (мс)
//
//  LINEAR:      B = start + delta·elapsed/duration
//  EXPONENTIAL: f(τ) = 1 - (1-τ)^3, τ = elapsed/duration ∈ [0,1]
//               Полиномиальная аппроксимация экспоненты: без float,
//               быстрый старт + плавное «дотягивание» к цели.
// ----------------------------------------------------------------------------
uint8_t PWMOutputController::_computeBrightness(uint32_t elapsed) const {

    // Защита от выхода за границу (на случай рассинхронизации).
    if (elapsed >= _transitionDuration) return _target;

    // Знаковое приращение — единый код для нарастания и затухания.
    int16_t delta = (int16_t)_target - (int16_t)_startBrightness;

    int32_t result;

    if (_curve == Curve::LINEAR) {
        // Прямая пропорция.
        int32_t adj = (int32_t)delta * (int32_t)elapsed / (int32_t)_transitionDuration;
        result = (int32_t)_startBrightness + adj;
    } else {
        // EXPONENTIAL без float:
        // τ нормируем в [0..65535], считаем (1-τ)^3, f = 1 - (1-τ)^3.
        uint32_t tau  = elapsed * 65535UL / _transitionDuration;
        uint32_t inv  = 65535UL - tau;
        uint32_t inv2 = (inv  * inv) / 65535UL;
        uint32_t inv3 = (inv2 * inv) / 65535UL;
        uint32_t f    = 65535UL - inv3;

        int32_t adj = (int32_t)delta * (int32_t)f / 65535L;
        result = (int32_t)_startBrightness + adj;
    }

    // Ограничение [0..255].
    if (result < 0)   result = 0;
    if (result > 255) result = 255;

    return (uint8_t)result;
}

// ----------------------------------------------------------------------------
// update — неблокирующий «тик». Вызывать в loop().
// ----------------------------------------------------------------------------
void PWMOutputController::update() {

    // До begin() ничего не делаем.
    if (!_begun)            return;

    // Нет активного перехода — выходим сразу.
    if (!_transitionActive) return;

    uint32_t elapsed = millis() - _transitionStart;

    if (elapsed >= _transitionDuration) {
        // Финал: фиксируем целевую яркость.
        _current          = _target;
        _transitionActive = false;
    } else {
        // Промежуточное значение по кривой.
        _current = _computeBrightness(elapsed);
    }

    _write(_current);
}