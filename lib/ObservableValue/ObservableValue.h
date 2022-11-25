#ifndef _OBSERVABLEVALUE_h
#define _OBSERVABLEVALUE_h

class ObservableValueBase
{
public:
    virtual void trigger(bool force = false);
};

template <typename T>

class ObservableValue : public ObservableValueBase
{
public:
    typedef void(observerFn)(T value);

    ObservableValue(T value)
    {
        _value = value;
    };

    T value()
    {
        return _value;
    }

    void setValue(T newValue)
    {
        if (newValue == _value)
        {
            return;
        }

        _value = newValue;
        _didTrigger = false;
        trigger();
    }

    void trigger(bool force = false)
    {
        unsigned long now = millis();
        bool rateLimited = _rateLimit > 0;
        bool timeout = !rateLimited || (now - _lastTrigger) > _rateLimit;

        if (!force && (!timeout || _didTrigger))
        {
            return;
        }

#if DEBUG
        // Serial.printf("ratelimited %i, timeout: %i, force: %i\n", rateLimited, timeout, force);
#endif

        notifyObservers();
        _didTrigger = true;
        _lastTrigger = millis();
    }

    void addObserver(observerFn *observer)
    {
        _observers[_observerCount] = observer;
        _observerCount++;
    }

private:
    T _value;
    byte _observerCount = 0;
    observerFn *_observers[5];
    unsigned long _lastTrigger = 0;
    unsigned int _rateLimit = 250;
    bool _didTrigger = false;

    void notifyObservers()
    {
        for (byte i = 0; i < _observerCount; i++)
        {
            _observers[i](_value);
        }
    }
};

#endif