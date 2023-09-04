#ifndef SWITCH_RELAY_H
#define SWITCH_RELAY_H

#include <Arduino.h>

typedef enum SwitchState { 
  Off,
  On
} SwitchState_t;

class SwitchRelay {
  public:
    void setOn() {
      setState(SwitchState_t::On);
    }

    void setOff() {
      setState(SwitchState_t::Off);
    }

    virtual SwitchState_t getState();
    virtual void setState(SwitchState_t targetState);
};

class SwitchRelayPin : public SwitchRelay {
  public:
    SwitchRelayPin(uint8_t pin) : SwitchRelayPin(pin, 1)
    { }

    SwitchRelayPin(uint8_t pin, uint8_t onValue, uint8_t pinModeType = OUTPUT)
      : pin(pin), onValue(onValue), offValue(onValue == HIGH ? LOW : HIGH)
    { 
      pinMode(pin, pinModeType);
      setState(state);
    }

    virtual SwitchState_t getState() {
      return state;
    }

    virtual void setState(SwitchState_t targetState) {
      digitalWrite(pin, targetState == SwitchState_t::On ? onValue : offValue);
      state = targetState;
    }
  
  private:
    const uint8_t pin, onValue, offValue;
    SwitchState_t state = SwitchState_t::Off;
};

class SwitchRelayMock : public SwitchRelay {
  public:
    SwitchRelayMock(SwitchState_t state = SwitchState_t::Off) : state(state) 
    { }

    virtual SwitchState_t getState() {
      return state;
    }

    virtual void setState(SwitchState_t targetState)
    { }

  private:
    SwitchState_t state = SwitchState_t::Off;
};

#endif