#include "Core.h"

#include <QsLog/QsLog.h>

namespace gcs {

Core::Core(QObject* parent) : QObject(parent)
{
    QLOG_TRACE() << "Core::Core()";
}

Core::~Core()
{
    QLOG_TRACE() << "Core::~Core()";
}

void Core::setManager(std::unique_ptr<IActionManager> manager)
{
    QLOG_TRACE() << "Core::setManager()";

    _manager = std::move(manager);

    auto managerObj = dynamic_cast<QObject*>(_manager.get());

    managerObj->setParent(this);

    // clang-format off
    connect(managerObj,  SIGNAL(sendDTO(QSharedPointer<IDTO>)),
            this,        SIGNAL(sendDTOToPlugin(QSharedPointer<IDTO>)));
    // clang-format on
}

void Core::managePluginDTO(ActionType action, QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "Core::managePluginDTO()";

    if (_manager) {
        _manager->executeAction(action, dto);
    }
}

} // namespace gcs
