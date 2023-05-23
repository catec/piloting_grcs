
#pragma once

#include <dataModel/business/XmlDataBusiness.h>

#include "../BasicAction.h"

namespace gcs {

class DataModel;

class PILOTING_GRCS_CORE_EXPORT LoadXmlDataFromFolderAction : public BasicAction
{
    Q_OBJECT

  public:
    explicit LoadXmlDataFromFolderAction(DataModel& dataModel, QObject* parent = nullptr);
    virtual ~LoadXmlDataFromFolderAction();

    ActionType getActionType() override;

    void execute(QSharedPointer<IDTO>) override;

  private:
    DataModel&      _dataModel;
    XmlDataBusiness _XmlDataBusiness;
};

} // namespace gcs