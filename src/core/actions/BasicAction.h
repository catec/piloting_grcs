#pragma once

#include <core/IAction.h>

#include <QObject>

namespace gcs {

class PILOTING_GRCS_CORE_EXPORT BasicAction : public QObject, public IAction
{
    Q_OBJECT

  public:
    explicit BasicAction(QObject* parent = nullptr);
    virtual ~BasicAction();

    virtual ActionType getActionType() = 0;

    virtual void execute(QSharedPointer<IDTO>) = 0;

  protected:
    void sendInfoMsg(const QString&);
    void sendWarningMsg(const QString&);

  Q_SIGNALS:
    void sendDTO(QSharedPointer<IDTO>) override;
};

} // namespace gcs
