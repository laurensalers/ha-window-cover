#ifndef _QUEUEHANDLER_h
#define _QUEUEHANDLER_h

#define MAX_QUEUE_LENGTH 10
#define DEQUEUE_BATCH_SIZE 2

template <typename T>

class QueueHandler
{
public:
    typedef void (*QueueHandlerFn)(T item);

    void setHandler(QueueHandlerFn fn)
    {
        _handler = fn;
    }

    void deQueue()
    {
        if (_itemCount == 0)
        {
            return;
        }

        for (byte i = 0; i < DEQUEUE_BATCH_SIZE && _itemCount > 0; i++)
        {
            T current = _items[_currentItemIndex];

            _handler(current);

            _currentItemIndex++;
            _itemCount--;

            if (_currentItemIndex >= MAX_QUEUE_LENGTH)
            {
                _currentItemIndex = 0;
            }
        }
    }

    T *getQueueEntry()
    {
        byte position = _itemQueuePosition;

        _itemQueuePosition++;
        _itemCount++;

        if (_itemQueuePosition >= MAX_QUEUE_LENGTH)
        {
            _itemQueuePosition = 0;
        }

        return &_items[position];
    }

private:
    QueueHandlerFn _handler;
    T _items[MAX_QUEUE_LENGTH];
    byte _itemCount = 0;
    byte _itemQueuePosition = 0;
    byte _currentItemIndex = 0;
};

#endif