#ifndef _OBSERVABLEMANAGER_h
#define _OBSERVABLEMANAGER_h

#include <ObservableValue.h>

class ObservableManagerClass
{
public:
    ObservableManagerClass(ObservableValueBase *observables[])
    {
        _observables = observables;
    }

    void trigger()
    {
        byte observableCount = sizeof(_observables);

#if DEBUG
        Serial.printf("Observable count: %d\n", observableCount);
#endif

        for (byte observerIndex = 0; observerIndex < observableCount; observerIndex++)
        {
            ObservableValueBase *observable = _observables[observerIndex];
            observable->trigger();
        }
    }

private:
    ObservableValueBase **_observables;
};

#endif