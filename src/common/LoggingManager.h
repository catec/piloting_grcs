#pragma once

#include <QFile>
#include <QTextStream>
#include <QThread>

#include "LoggingQueue.h"
#include "gcs_common_export.h"

namespace gcs {

class PILOTING_GRCS_COMMON_EXPORT LoggingManager : public QThread
{
    Q_OBJECT

  public:
    explicit LoggingManager(const quint32 bufferSize, QObject* parent = nullptr);
    virtual ~LoggingManager();

    bool startLogging(const QString&, const QString&);
    bool stopLogging();
    bool isLogging() { return _isLogging; }

    QString getLogFilePath() const;

    void pullNewLogMessage(const QString&);

  protected:
    void run() Q_DECL_OVERRIDE;

  private:
    bool         _isLogging{false};
    LoggingQueue _queue;
    QFile        _logFile;
    QTextStream  _logFileStream;
};

} // namespace gcs
