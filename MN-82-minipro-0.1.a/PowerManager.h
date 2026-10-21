#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>

// Структура для описания входного пина (датчик, RC-модуль и т.д.)
struct InputPin {
  uint8_t pin;
  uint8_t activeLevel; // HIGH или LOW
  uint8_t mode;        // INPUT, INPUT_PULLUP и т.д.
};

class PowerManager {
  public:
    // Конструктор: 
    // holdPin - пин удержания, holdActiveLevel - активный уровень для удержания (HIGH/LOW)
    // inputs - массив входов, inputCount - количество входов
    // timeoutMs - время бездействия в миллисекундах до отключения
    PowerManager(uint8_t holdPin, uint8_t holdActiveLevel, 
                 const InputPin* inputs, uint8_t inputCount, 
                 uint32_t timeoutMs);

    // Инициализация пинов и подача сигнала удержания
    void begin();

    // Вызывать в loop() для опроса датчиков и проверки таймаута
    void update();

    // Принудительный сброс таймера (если активность произошла вне update)
    void resetTimer();

    // Изменение таймаута "на лету"
    void setTimeout(uint32_t timeoutMs);

    // Регистрация коллбэка, выполняемого перед отключением питания
    void setShutdownCallback(void (*callback)());

  private:
    uint8_t _holdPin;
    uint8_t _holdActiveLevel;
    const InputPin* _inputs;
    uint8_t _inputCount;
    uint32_t _timeoutMs;
    uint32_t _lastActivityTime;
    bool _isActive;
    
    void (*_shutdownCallback)();

    void shutdown();
};

#endif