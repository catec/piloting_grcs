#pragma once

#include <QDateTime>
#include <QMutex>
#include <QQueue>
#include <QWaitCondition>

namespace gcs {

class LoggingQueue
{
  public:
    explicit LoggingQueue(const quint32 max_queue_size);
    virtual ~LoggingQueue();

    void setWaitMsg(const bool value);

    void addMessage(const QString&);
    bool removeMessage(QString&);
    void clearMessages();

  private:
    void checkQueueSizeExceeded();

    bool            _wait_msg{true};
    QWaitCondition  _queue_not_empty;
    QMutex          _queue_mutex;
    QQueue<QString> _msg_queue;
    quint32         _max_queue_size;
    quint64         _queue_size{0};
    QDateTime       _last_queue_size_warn;
};

} // namespace gcs
