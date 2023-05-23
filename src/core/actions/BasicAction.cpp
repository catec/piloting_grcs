#include "BasicAction.h"

#include <QsLog/QsLog.h>
#include <dataModel/dtos/MsgConsoleDTO.h>

namespace gcs {

BasicAction::BasicAction(QObject* parent) : QObject(parent), IAction()
{
    QLOG_TRACE() << "BasicAction::BasicAction()";
}

BasicAction::~BasicAction()
{
    QLOG_TRACE() << "BasicAction::~BasicAction()";
}

void BasicAction::sendInfoMsg(const QString& msg)
{
    QLOG_TRACE() << "BasicAction::sendInfoMsg()";

    auto msgConsoleDTO          = createSharedDTO<MsgConsoleDTO>();
    msgConsoleDTO->getMsgType() = MsgType::INFO;
    msgConsoleDTO->getMessage() = msg;
    Q_EMIT sendDTO(msgConsoleDTO);
}

void BasicAction::sendWarningMsg(const QString& msg)
{
    QLOG_TRACE() << "BasicAction::sendWarningMsg()";

    auto msgConsoleDTO          = createSharedDTO<MsgConsoleDTO>();
    msgConsoleDTO->getMsgType() = MsgType::WARNING;
    msgConsoleDTO->getMessage() = msg;
    Q_EMIT sendDTO(msgConsoleDTO);
}

} // namespace gcs
