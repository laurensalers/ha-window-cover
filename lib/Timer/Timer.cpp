#include "Timer.h"

#define DEBUG false

TimerClass::TimerClass() {}

byte TimerClass::createTimer(unsigned int delay, bool enabled, TimerTypes timerType, void callback())
{
  if (timerIndex >= MAX_TIMERS)
  {
#if DEBUG
    Serial.println("Max timers reached");
#endif
    return -1;
  }

  unsigned long currentMillis = millis();
  byte timerId = timerIndex;

  TimerItem timer = {
      enabled,       // enabled
      delay,         // delay
      currentMillis, // last time
      timerType,     // timer type
      callback       // callback
  };

  timers[timerId] = timer;
  timerIndex++;

#if DEBUG
  Serial.print("Timer: ");
  Serial.print(timerId);
  Serial.print(" ");
  Serial.println(timerIndex);
#endif

  return timerId;
}

void TimerClass::setTimerDelay(byte id, unsigned int delay)
{
  TimerItem timer = timers[id];

  timer.delay = delay;

#if DEBUG
  Serial.print("Timer ");
  Serial.print(id);
  Serial.print(" delay ");
  Serial.println(delay);
#endif

  timers[id] = timer;
}

void TimerClass::startTimer(byte id)
{
  if (id >= timerIndex)
  {
    return;
  }

#if DEBUG
  Serial.print("Timer start ");
  Serial.println(id);
#endif
  TimerItem timer = timers[id];

  if (timer.enabled)
  {
    return;
  }

  timer.lastTime = millis();
  timer.enabled = true;

  timers[id] = timer;
}

void TimerClass::stopTimer(byte id)
{
  if (id >= timerIndex)
  {
    return;
  }

#if DEBUG
  Serial.print("Timer stop: ");
  Serial.println(id);
#endif

  TimerItem timer = timers[id];

  if (!timer.enabled)
  {
    return;
  }

  timer.enabled = false;
  timers[id] = timer;
}

void TimerClass::resetTimer(byte id)
{
  startTimer(id);
}

void TimerClass::handleTimer(byte index)
{
  TimerItem timer = timers[index];

  if (!timer.enabled)
  {
    return;
  }

  unsigned long currentMillis = millis();

  if ((unsigned long)(currentMillis - timer.lastTime) >= timer.delay)
  {
    timer.lastTime = currentMillis;

    if (timer.timerType == TimerTypes::TIMER_ONCE)
    {
      timer.enabled = false;
    }

    timers[index] = timer;
    timer.callback();
  }
}

void TimerClass::loop()
{
  for (byte index = 0; index < timerIndex; index++)
  {
    handleTimer(index);
  }
}

TimerClass Timer;