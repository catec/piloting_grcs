#include "ActionManager.h"

#include <QsLog/QsLog.h>

namespace gcs {

ActionManager::ActionManager(QObject* parent) : QObject(parent)
{
    QLOG_TRACE() << "ActionManager::ActionManager()";
}

ActionManager::~ActionManager()
{
    QLOG_TRACE() << "ActionManager::~ActionManager()";
}

int ActionManager::getSize()
{
    QLOG_TRACE() << "ActionManager::getSize()";

    return _actionsMap.size();
}

void ActionManager::addAction(std::unique_ptr<IAction> action)
{
    QLOG_TRACE() << "ActionManager::addAction()";

    if (!action) {
        const QString msg("Action is NULL");
        QLOG_ERROR() << "ActionManager::addAction() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    ActionType type = action->getActionType();

    if (_actionsMap.count(type) != 0) {
        const QString msg("Action type already exists");
        QLOG_ERROR() << "ActionManager::addAction() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    auto actionObj = dynamic_cast<QObject*>(action.get());

    actionObj->setParent(this);

    // clang-format off
   connect(actionObj,   SIGNAL(sendDTO(QSharedPointer<IDTO>)),
           this,        SIGNAL(sendDTO(QSharedPointer<IDTO>)));
    // clang-format on

    _actionsMap.insert(std::make_pair(type, std::move(action)));
}

void ActionManager::removeAction(const ActionType& actionType)
{
    QLOG_TRACE() << "ActionManager::removeAction()";

    std::map<ActionType, std::unique_ptr<IAction>>::iterator it;
    it = _actionsMap.find(actionType);

    if (it == _actionsMap.end()) {
        const QString msg("Action type does not exist");
        QLOG_ERROR() << "ActionManager::removeAction() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    _actionsMap.erase(it);
}

void ActionManager::removeAllActions()
{
    QLOG_TRACE() << "ActionManager::removeAllActions()";

    _actionsMap.clear();
}

void ActionManager::executeAction(const ActionType& actionType, QSharedPointer<IDTO> dto)
{
    QLOG_TRACE() << "ActionManager::executeAction()";

    std::map<ActionType, std::unique_ptr<IAction>>::iterator it;
    it = _actionsMap.find(actionType);

    if (it == _actionsMap.end()) {
        const QString msg("Action type does not exist");
        QLOG_ERROR() << "ActionManager::executeAction() -" << msg;
        throw std::runtime_error(msg.toStdString());
    }

    it->second->execute(dto);
}

} // namespace gcs
