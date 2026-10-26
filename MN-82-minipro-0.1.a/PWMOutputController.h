// ============================================================================
// PWMOutputController.h
//
// Универсальный контроллер одного выхода с поддержкой ШИМ и имитацией
// тепловой инерции лампы накаливания.
//
// Заменяет простой OutputController: при riseTime=fallTime=0 или hasPwm=false
// ведёт себя как обычный on/off-контроллер.
//
// Требования к использованию:
//   1. Создать объект с параметрами (pin, inverted, hasPwm).
//   2. В setup() вызвать begin()  (и Serial.begin() до него, если verbose=true).
//   3. В loop() как можно чаще вызывать update().
//   4. Управлять через on()/off()/setBrightness()/set().
// ============================================================================

#ifndef PWM_OUTPUT_CONTROLLER_H
#define PWM_OUTPUT_CONTROLLER_H

#include <Arduino.h>

class PWMOutputController {
public:

    // ----------------------------------------------------------------------
    // Кривая интерполяции яркости во время перехода
    // ----------------------------------------------------------------------
    enum class Curve : uint8_t {
        LINEAR      = 0,   // B = start + (target-start)·t
        EXPONENTIAL = 1    // ease-out: быстрый старт, плавное завершение
    };

    // ----------------------------------------------------------------------
    // Конструктор
    //   pin      : номер пина Arduino
    //   inverted : true  — PNP-транзистор (LOW=ON, HIGH=OFF)
    //              false — прямая логика (HIGH=ON, LOW=OFF)
    //   hasPwm   : true  — использовать analogWrite()
    //              false — только digitalWrite() (бинарный режим)
    //   verbose  : true  — выводить предупреждение при несоответствии пина
    // ----------------------------------------------------------------------
    PWMOutputController(uint8_t pin,
                        bool    inverted = false,
                        bool    hasPwm   = true,
                        bool    verbose  = true);

    // ----------------------------------------------------------------------
    // Настройка пина и сброс в состояние ВЫКЛ. Обязательно в setup().
    // ----------------------------------------------------------------------
    void begin();

    // ----------------------------------------------------------------------
    // Настройки профиля (можно менять на лету)
    //   setRiseTime(ms) — длительность полного перехода 0→255
    //   setFallTime(ms) — длительность полного перехода 255→0
    //   Меньшие дельты занимают пропорционально меньшее время:
    //   delta=10 при riseTime=100 → ~4 мс (10·100/255).
    // ----------------------------------------------------------------------
    void setRiseTime(uint16_t ms);
    void setFallTime(uint16_t ms);
    void setCurve(Curve c);
    void setMaxBrightness(uint8_t v);   // 0..255, ограничивает on()

    // ----------------------------------------------------------------------
    // Управление
    // ----------------------------------------------------------------------
    void on();                       // плавно к maxBrightness
    void off();                      // плавно к 0
    void set(bool state);            // обёртка on()/off()
    void setBrightness(uint8_t v);   // плавно к указанному уровню

    // ----------------------------------------------------------------------
    // Обслуживание — вызывать из loop() каждую итерацию.
    // ----------------------------------------------------------------------
    void update();

    // ----------------------------------------------------------------------
    // Запросы
    // ----------------------------------------------------------------------
    uint8_t currentBrightness() const { return _current; }
    uint8_t targetBrightness()  const { return _target;  }
    bool    isSettled()         const { return _current == _target; }
    bool    isOn()              const { return _target > 0; }
    bool    pwmActive()         const { return _hasPwm; }

private:
    static bool _isHardwarePwmPin(uint8_t pin);

    void    _startTransition(uint8_t newTarget);
    void    _write(uint8_t brightness);
    uint8_t _computeBrightness(uint32_t elapsed) const;

    // ---- Конфигурация ----
    uint8_t  _pin;
    bool     _inverted;
    bool     _hasPwm;
    bool     _verbose;
    bool     _begun = false;

    uint16_t _riseTimeMs    = 0;
    uint16_t _fallTimeMs    = 0;
    Curve    _curve         = Curve::LINEAR;
    uint8_t  _maxBrightness = 255;

    // ---- Состояние ----
    uint8_t  _target             = 0;
    uint8_t  _current            = 0;
    uint8_t  _startBrightness    = 0;
    uint32_t _transitionStart    = 0;
    uint16_t _transitionDuration = 0;
    bool     _transitionActive   = false;
    int16_t  _lastWritten        = -1;
};

#endif