#ifndef _OBSERVABLEMANAGER_h
#define _OBSERVABLEMANAGER_h

#include <ObservableValue.h>

#define TRIGGER_LIMIT 500

class ObservableManagerClass
{
public:
    ObservableManagerClass(ObservableValueBase *observables[])
    {
        _observables = observables;
    }

    void trigger(bool force = false)
    {
        if (!(millis() - _lastTriggerTime > TRIGGER_LIMIT * 1000))
        {
            return;
        }

#if DEBUG
        debugSerial.printf("ObservableManager trigger, force: %i", force);
#endif

        byte observableCount = sizeof(_observables) + 1;

        for (byte observerIndex = 0; observerIndex < observableCount; observerIndex++)
        {
            ObservableValueBase *observable = _observables[observerIndex];
            observable->trigger(force);
        }

        _lastTriggerTime = millis();
    }

private:
    ObservableValueBase **_observables;
    unsigned long _lastTriggerTime = 0;
};

#endif