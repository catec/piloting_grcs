#pragma once

#include <QObject>
#include <memory>

#include "gcs_core_export.h"
#include "managers/IActionManager.h"

namespace gcs {

class PILOTING_GRCS_CORE_EXPORT Core : public QObject
{
    Q_OBJECT

  public:
    explicit Core(QObject* parent = nullptr);
    virtual ~Core();

    void setManager(std::unique_ptr<IActionManager> manager);

  public Q_SLOTS:
    void managePluginDTO(ActionType, QSharedPointer<IDTO>);

  private:
    std::unique_ptr<IActionManager> _manager;

  Q_SIGNALS:
    void sendDTOToPlugin(QSharedPointer<IDTO>);
};

} // namespace gcs
