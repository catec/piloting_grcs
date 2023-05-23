#include "LoggingQueue.h"

#include <QsLog/QsLog.h>

namespace gcs {

LoggingQueue::LoggingQueue(const quint32 max_queue_size) :
        _max_queue_size(max_queue_size)
        /// \note _last_queue_size_warn is set to epoch time since default constructor returns null datetime
        ,
        _last_queue_size_warn(QDateTime::fromSecsSinceEpoch(0))
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

LoggingQueue::~LoggingQueue()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

void LoggingQueue::setWaitMsg(const bool value)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    _wait_msg = value;
    _queue_not_empty.wakeAll();
}

void LoggingQueue::addMessage(const QString& out_msg)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QMutexLocker mutexLocker(&_queue_mutex);

    _msg_queue.enqueue(out_msg);
    _queue_size += 1;

    checkQueueSizeExceeded();

    _queue_not_empty.wakeAll();
}

bool LoggingQueue::removeMessage(QString& out_msg)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QMutexLocker mutexLocker(&_queue_mutex);

    while (_msg_queue.isEmpty()) {
        if (!_wait_msg) {
            return false;
        }

        /// \note Max wait time: 250 milliseconds
        _queue_not_empty.wait(mutexLocker.mutex(), 250);
    }

    out_msg      = _msg_queue.dequeue();
    _queue_size -= 1;

    return true;
}

void LoggingQueue::clearMessages()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    QMutexLocker mutexLocker(&_queue_mutex);

    _msg_queue.clear();
    _queue_size = 0;
}

void LoggingQueue::checkQueueSizeExceeded()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    while (_max_queue_size > 0 && _queue_size > _max_queue_size) {
        auto dropMsg  = _msg_queue.dequeue();
        _queue_size  -= 1;

        auto now = QDateTime::currentDateTime();
        if (_last_queue_size_warn.secsTo(now) > 5) {
            QLOG_ERROR() << __PRETTY_FUNCTION__ << "Queue size exceeded. Dropping oldest queued message";
            _last_queue_size_warn = now;
        }
    }
}

} // namespace gcs
