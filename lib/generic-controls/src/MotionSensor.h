#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#include <Arduino.h>
#include <functional>

typedef enum MotionState { 
  None = 0,
  Detected
} MotionState_t;

typedef std::function<void(MotionState_t)> MotionSensorEventCallback;

class MotionSensor
{
  public:
    MotionSensor(uint8_t pin, uint8_t defaultValue = 0) : pin(pin)
    {
      pinMode(pin, INPUT);
      // setState(digitalRead(pin));
      setState(defaultValue);
    }

    MotionState_t getState()
    {
      return state;
    }

    void onChanged(MotionSensorEventCallback cb)
    {
      onMotionStateCallback = cb;
    }

    void loop() {
      int s1 = digitalRead(pin);

      // delay(50);
      // int s2 = digitalRead(pin);
      // if (s1 != s2) {
      //   return;
      // }

      if (s1 != (int)state) {
        setState(s1);

        if (onMotionStateCallback != NULL)
          onMotionStateCallback(state);
      }
    }

  private:
    uint8_t pin;
    MotionState_t state = None, lastState = None;
    MotionSensorEventCallback onMotionStateCallback = NULL;

    void setState(int value) {
      log_d("Motion sensor @%d: value=%d", pin, value);

      lastState = state;
      state = value == 0 ? None : Detected;
    }
};

#endif