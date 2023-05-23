#pragma once

#include <QDialog>
#include <memory>

#include "gcs_gui_export.h"

namespace Ui {
class WaypointsOptionsDialog;
}

namespace gcs {
class InspectionTaskDTO;
class PILOTING_GRCS_GUI_EXPORT WaypointsOptionsDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit WaypointsOptionsDialog(QWidget* parent = nullptr, bool show_altitude = true);
    virtual ~WaypointsOptionsDialog();

    void  setDefaultAltitude(const float&);
    float getDefaultAltitude() const;

    void setInspectionTaskList(const QList<InspectionTaskDTO>&);

    QString getInspectionTaskUUID() const;

  private Q_SLOTS:
    void itemChanged(int);

  private:
    std::unique_ptr<Ui::WaypointsOptionsDialog> _ui;
};

} // namespace gcs
