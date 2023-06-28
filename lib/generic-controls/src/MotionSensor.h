#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#include <Arduino.h>
#include <functional>

enum class MotionState { 
  None = 0,
  Detected
};

typedef std::function<void(MotionState)> MotionSensorEventCallback;

class MotionSensor
{
  public:
    MotionSensor(uint8_t pin) : pin(pin)
    {
      pinMode(pin, INPUT);
      setState(digitalRead(pin));
    }

    MotionState getState()
    {
      return state;
    }

    void onChanged(MotionSensorEventCallback cb)
    {
      onMotionStateCallback = cb;
    }

    void loop() {
      int s = digitalRead(pin);
      if (s != (int)state) {
        setState(s);

        if (onMotionStateCallback != NULL)
          onMotionStateCallback(state);
      }
    }

  private:
    uint8_t pin;
    MotionState state = MotionState::None, lastState = MotionState::None;
    MotionSensorEventCallback onMotionStateCallback = NULL;

    void setState(int value) {
      log_d("Motion sensor @%d: value=%d", pin, value);

      lastState = state;
      state = value == 0 ? MotionState::None : MotionState::Detected;
    }
};

#endif