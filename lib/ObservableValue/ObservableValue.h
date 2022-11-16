#ifndef _OBSERVABLEVALUE_h
#define _OBSERVABLEVALUE_h

template <typename T>

class ObservableValue
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
        notifyObservers(newValue);
    }

    void trigger()
    {
        notifyObservers(_value);
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

    void notifyObservers(T value)
    {
        for (byte i = 0; i < _observerCount; i++)
        {
            _observers[i](value);
        }
    }
};

#endif