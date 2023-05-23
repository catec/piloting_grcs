#pragma once

#include <QObject>

#include "IActionManager.h"

namespace gcs {

class PILOTING_GRCS_CORE_EXPORT ActionManager : public QObject, public IActionManager
{
    Q_OBJECT

  public:
    explicit ActionManager(QObject* parent = nullptr);
    virtual ~ActionManager();

    int getSize();

    void addAction(std::unique_ptr<IAction> action);

    void removeAction(const ActionType& actionType);

    void removeAllActions();

    void executeAction(const ActionType& actionType, QSharedPointer<IDTO> dto);

  private:
    std::map<ActionType, std::unique_ptr<IAction>> _actionsMap;

  Q_SIGNALS:
    void sendDTO(QSharedPointer<IDTO>);
};

} // namespace gcs
