#include "ConsoleDisplayWidget.h"

#include <QsLog/QsLog.h>

#include <QTime>

#include "../../dtos/GuiDTO.h"
#include "ui_ConsoleDisplayWidget.h"

namespace gcs {

ConsoleDisplayWidget::ConsoleDisplayWidget(GuiDTO& guiDTO, QWidget* parent) :
        QTextEdit(parent), _ui(std::make_unique<Ui::ConsoleDisplayWidget>()), _guiDTO(guiDTO)
{
    _ui->setupUi(this);
}

ConsoleDisplayWidget::~ConsoleDisplayWidget() {}

void ConsoleDisplayWidget::addMessage(const MsgType& type, const QString& msg)
{
    const auto color((type == MsgType::ERROR) ? Qt::red : (type == MsgType::WARNING) ? Qt::yellow : Qt::white);

    setTextColor(color);

    const auto text
            = QString("[%1] (%2) %3").arg(QTime::currentTime().toString("hh:mm:ss")).arg(ToString(type)).arg(msg);
    append(text);

    if (_guiDTO.getMissionStatus() == MissionStatusType::NoMission) {
        return;
    }

    if (_loggingManager) {
        const auto timeStamp = QDateTime::currentDateTime().toMSecsSinceEpoch();
        const auto textLog   = QString("%1,%2,%3")
                                     .arg(QString::number(timeStamp / 1000.0, 'f', 5))
                                     .arg(ToString(type))
                                     .arg("\"" + msg + "\"");
        _loggingManager->pullNewLogMessage(textLog);
    }
}

void ConsoleDisplayWidget::clearMessages()
{
    clear();
}

void ConsoleDisplayWidget::updateLogging(const QString& logFilePath)
{
    if (_loggingManager) {
        if (_loggingManager->getLogFilePath() == logFilePath) {
            return;
        }
        _loggingManager.reset();
    }

    if (logFilePath.isNull() || logFilePath.isEmpty()) {
        return;
    }

    const auto consoleHeader = QString("## timestamp,severity,message");
    _loggingManager          = std::make_unique<LoggingManager>(10);
    if (_loggingManager) {
        _loggingManager->startLogging(logFilePath, consoleHeader);
    }
}

} // namespace gcs
