#ifndef PUSH_BUTTON_H
#define PUSH_BUTTON_H

#include <Arduino.h>
#include <functional>

#define DIGITAL_READ_COUNT          20
#define DIGITAL_READ_THRESHOLD      10
#define DIGITAL_READ_DELAY_MS       2
#define READ_THRESHOLD_MS           200

enum class ButtonState : uint8_t { 
  Off = 0,
  On
};

class PushButton
{
  public:
    typedef std::function<void(PushButton*)> PushButtonEventCallback;

    PushButton(uint8_t pin, uint8_t inputMode = INPUT, uint8_t onValue = HIGH, unsigned int threshold_ms = 60) : pin(pin), onValue(onValue), threshold_ms(threshold_ms)
    {
      pinMode(pin, inputMode);
      _setState(_digitalRead(pin));
    }

    ButtonState getState()
    {
      return state;
    }

    ButtonState getStateRaw()
    {
      return _digitalRead(pin);
    }

    void onChanged(PushButtonEventCallback cb)
    {
      _onStateChangedCb = cb;
    }

    void loop(unsigned long now) {
      if (now - lastRead_ms < READ_THRESHOLD_MS) return;
      lastRead_ms = now;

      auto s = _digitalRead(pin);
      lastRead_ms = millis();

      if (s != state) {
        if (s != newState) {
          newState = s;
          newState_ms = now;
        }
        else if (lastRead_ms - newState_ms > threshold_ms) {
          _setState(s);

          if (_onStateChangedCb != NULL)
            _onStateChangedCb(this);
        }
      }
    }

  private:
    uint8_t pin;
    uint8_t onValue;
    unsigned int threshold_ms;
    unsigned long newState_ms = 0;
    unsigned long lastRead_ms = 0;
    ButtonState state = ButtonState::Off, lastState = ButtonState::Off, newState = ButtonState::Off;
    PushButtonEventCallback _onStateChangedCb = NULL;

    void _setState(ButtonState value) {
      log_d("PushButton @%d: value=%d", pin, value);

      lastState = state;
      state = value;
    }

    ButtonState _digitalRead(uint8_t pin)
    {
      uint8_t val = 0;
      for (uint8_t i=0; i<DIGITAL_READ_COUNT; i++) {
        if (digitalRead(pin) > 0) val++;
        delay(DIGITAL_READ_DELAY_MS);
      }

      if (onValue > 0)
        return val >= DIGITAL_READ_THRESHOLD ? ButtonState::On : ButtonState::Off;
      else 
        return val < DIGITAL_READ_THRESHOLD ? ButtonState::On : ButtonState::Off;
    }
};

#endif