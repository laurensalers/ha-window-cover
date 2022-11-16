#ifndef _TIMER_h
#define _TIMER_h

#include "Arduino.h"

#define MAX_TIMERS 7

class TimerClass
{

public:
  typedef enum
  {
    TIMER_INTERVAL,
    TIMER_ONCE
  } TimerTypes;

  struct TimerItem
  {
    bool enabled;

    unsigned int delay;
    unsigned long lastTime;
    TimerTypes timerType;
    void (*callback)();
  };

  TimerClass();
  void loop();
  byte createTimer(unsigned int delay, bool enabled, TimerTypes timerType, void callback());
  void startTimer(byte id);
  void stopTimer(byte id);
  void resetTimer(byte id);
  void setTimerDelay(byte id, unsigned int delay);

private:
  byte timerIndex;
  TimerItem timers[MAX_TIMERS];
  void handleTimer(byte index);
};

extern TimerClass Timer;

#endif
