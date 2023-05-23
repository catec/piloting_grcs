#pragma once

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT ManageAutoHeadingWptsAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit ManageAutoHeadingWptsAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~ManageAutoHeadingWptsAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel& _dataModel;
};

} // namespace gcs
