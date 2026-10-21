#include "PowerManager.h"

PowerManager::PowerManager(uint8_t holdPin, uint8_t holdActiveLevel, 
                           const InputPin* inputs, uint8_t inputCount, 
                           uint32_t timeoutMs) {
  _holdPin = holdPin;
  _holdActiveLevel = holdActiveLevel;
  _inputs = inputs;
  _inputCount = inputCount;
  _timeoutMs = timeoutMs;
  _shutdownCallback = nullptr;
  _isActive = false;
}

void PowerManager::begin() {
  // Настройка пина удержания и немедленная фиксация питания
  pinMode(_holdPin, OUTPUT);
  digitalWrite(_holdPin, _holdActiveLevel);
  
  // Настройка входных пинов
  for (uint8_t i = 0; i < _inputCount; i++) {
    pinMode(_inputs[i].pin, _inputs[i].mode);
  }

  _lastActivityTime = millis();
  _isActive = true;
}

void PowerManager::update() {
  if (!_isActive) return;

  bool activityDetected = false;

  // Опрос всех входов
  for (uint8_t i = 0; i < _inputCount; i++) {
    if (digitalRead(_inputs[i].pin) == _inputs[i].activeLevel) {
      activityDetected = true;
      break; // Достаточно одного активного сигнала
    }
  }

  if (activityDetected) {
    resetTimer();
  }

  // Проверка таймаута (безопасно для переполнения millis())
  if (millis() - _lastActivityTime >= _timeoutMs) {
    shutdown();
  }
}

void PowerManager::resetTimer() {
  _lastActivityTime = millis();
}

void PowerManager::setTimeout(uint32_t timeoutMs) {
  _timeoutMs = timeoutMs;
}

void PowerManager::setShutdownCallback(void (*callback)()) {
  _shutdownCallback = callback;
}

void PowerManager::shutdown() {
  if (!_isActive) return;
  _isActive = false;

  // Вызов пользовательского коллбэка, если он задан
  if (_shutdownCallback != nullptr) {
    _shutdownCallback();
  }

  // Отключение удержания питания
  // Устанавливаем уровень, противоположный активному
  digitalWrite(_holdPin, (_holdActiveLevel == HIGH) ? LOW : HIGH);
}