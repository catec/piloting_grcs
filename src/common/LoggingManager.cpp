#include "LoggingManager.h"

#include <QsLog/QsLog.h>

namespace gcs {

LoggingManager::LoggingManager(const quint32 bufferSize, QObject* parent) : QThread(parent), _queue(bufferSize)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;
}

LoggingManager::~LoggingManager()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    if (_isLogging) {
        stopLogging();
    }
}

bool LoggingManager::startLogging(const QString& logFilePath, const QString& header)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Starting Logging manager thread...";

        if (_isLogging) {
            throw std::runtime_error("Logging manager thread has already been started");
        }

        _logFile.setFileName(logFilePath);
        if (!_logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            throw std::runtime_error("Error opening log file");
        }
        _logFileStream.setDevice(&_logFile);

        _isLogging = true;

        _queue.clearMessages();
        _queue.setWaitMsg(true);
        _queue.addMessage(header);
        this->start();

        return true;
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        return false;
    }
}

bool LoggingManager::stopLogging()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    try {
        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Stopping logging manager thread...";

        if (!_isLogging) {
            throw std::runtime_error("Logging manager thread has already been stopped");
        }

        _isLogging = false;

        _queue.setWaitMsg(false);
        this->wait();

        if (_logFile.isOpen()) {
            _logFile.close();
        }
        _logFileStream.setDevice(nullptr);

        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Logging manager thread terminated!";

        return true;
    } catch (const std::exception& e) {
        QLOG_ERROR() << __PRETTY_FUNCTION__ << "-" << e.what();
        return false;
    }
}

QString LoggingManager::getLogFilePath() const
{
    return _logFile.isOpen() ? _logFile.fileName() : QString();
}

void LoggingManager::pullNewLogMessage(const QString& message)
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    /// \note To avoid queueing more messages when you want to stop the thread
    if (!_isLogging) {
        return;
    }

    _queue.addMessage(message);
}

void LoggingManager::run()
{
    QLOG_TRACE() << __PRETTY_FUNCTION__;

    for (;;) {
        QString message;
        if (!_queue.removeMessage(message)) {
            break;
        }

        if (message.isEmpty()) {
            QLOG_ERROR() << "Message is empty";
            continue;
        }

        QLOG_DEBUG() << __PRETTY_FUNCTION__ << "New log message received:" << message;

        _logFileStream << message << "\n";
    }

    QLOG_DEBUG() << __PRETTY_FUNCTION__ << "Logging manager thread stopped";
}

} // namespace gcs
