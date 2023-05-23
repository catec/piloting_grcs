#pragma once

#include <QDialog>
#include <QPushButton>
#include <functional>
#include <memory>

#include "dataModel/dtos/InspectionTaskDTO.h"

namespace Ui {
class InspectionTaskInfoDialog;
}

namespace gcs {
class InspectionTaskInfoDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit InspectionTaskInfoDialog(QWidget* parent = nullptr);
    virtual ~InspectionTaskInfoDialog();

    void setInspectionTaskInfo(const InspectionTaskDTO&);

  private:
    QPushButton* createCircularButton(const int, std::function<void()>);

    std::unique_ptr<Ui::InspectionTaskInfoDialog> _ui;
};

} // namespace gcs
