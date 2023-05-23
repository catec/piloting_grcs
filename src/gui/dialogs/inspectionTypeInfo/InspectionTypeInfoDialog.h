#pragma once

#include <QDialog>
#include <memory>

#include "dataModel/dtos/InspectionTypeDTO.h"

namespace Ui {
class InspectionTypeInfoDialog;
}

namespace gcs {
class InspectionTypeInfoDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit InspectionTypeInfoDialog(QWidget* parent = nullptr);
    virtual ~InspectionTypeInfoDialog();

    void setInspectionTypeInfo(const InspectionTypeDTO&);

  private:
    std::unique_ptr<Ui::InspectionTypeInfoDialog> _ui;
};

} // namespace gcs
