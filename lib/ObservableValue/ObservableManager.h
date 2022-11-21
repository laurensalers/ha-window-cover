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

    void trigger(bool force = false)
    {
        byte observableCount = sizeof(_observables) + 1;

        for (byte observerIndex = 0; observerIndex < observableCount; observerIndex++)
        {
            ObservableValueBase *observable = _observables[observerIndex];
            observable->trigger(force);
        }
    }

private:
    ObservableValueBase **_observables;
};

#endif