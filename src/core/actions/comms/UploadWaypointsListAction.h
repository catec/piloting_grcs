
#pragma once

#include "../BasicAction.h"

namespace gcs {

class ICommunications;
class DataModel;

class PILOTING_GRCS_CORE_EXPORT UploadWaypointsListAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UploadWaypointsListAction(DataModel& dataModel, ICommunications& comms, QObject* parent = nullptr);
    virtual ~UploadWaypointsListAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&       _dataModel;
    ICommunications& _comms;
    QObject*         _commsObj{nullptr};
};

} // namespace gcs
