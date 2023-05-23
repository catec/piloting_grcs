#pragma once

#include <common/Types.h>

#include <QDialog>
#include <memory>

#include "gcs_gui_export.h"

namespace Ui {
class InspectionTaskActionInfoDialog;
}

namespace gcs {
class PILOTING_GRCS_GUI_EXPORT InspectionTaskActionInfoDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit InspectionTaskActionInfoDialog(QWidget* parent = nullptr);
    virtual ~InspectionTaskActionInfoDialog();

    void setInspTaskActionName(const QString&);
    void setInspTaskActionUUID(const QString&);
    void setInspTaskActionNotes(const QString&);
    void setInspTaskActionProperties(const QString&);

  private:
    std::unique_ptr<Ui::InspectionTaskActionInfoDialog> _ui;
};

} // namespace gcs
