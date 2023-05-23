#pragma once

#include <QDialog>
#include <memory>

#include "dataModel/dtos/InspectionPlanDTO.h"

namespace Ui {
class InspectionPlanInfoDialog;
}

namespace gcs {
class InspectionPlanInfoDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit InspectionPlanInfoDialog(QWidget* parent = nullptr);
    virtual ~InspectionPlanInfoDialog();

    void setInspectionPlanInfo(const InspectionPlanDTO&);

  private:
    std::unique_ptr<Ui::InspectionPlanInfoDialog> _ui;
};

} // namespace gcs
