#pragma once

#include <dataModel/business/MissionResultsBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DDHLComms;
class DataModel;

class PILOTING_GRCS_CORE_EXPORT UploadMissionResultListToDDHLAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit UploadMissionResultListToDDHLAction(DDHLComms& comms, DataModel& dataModel, QObject* parent = nullptr);
    virtual ~UploadMissionResultListToDDHLAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DDHLComms&             _comms;
    QObject*               _commsObj{nullptr};
    DataModel&             _dataModel;
    MissionResultsBusiness _missionResultsBusiness;
};

} // namespace gcs
