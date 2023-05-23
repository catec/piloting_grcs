#pragma once

#include <QButtonGroup>
#include <QDialog>
#include <memory>

#include "gcs_gui_export.h"

namespace Ui {
class WaypointHeadingEditorDialog;
}

namespace gcs {

class PILOTING_GRCS_GUI_EXPORT WaypointHeadingEditorDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit WaypointHeadingEditorDialog(QWidget* parent = nullptr);
    virtual ~WaypointHeadingEditorDialog();

  Q_SIGNALS:
    void modifyPreviewWp(float);
    void modifyCurrentWp(float);

  private Q_SLOTS:
    void sliderReleased();
    void applyButtonPressed();

  private:
    std::unique_ptr<Ui::WaypointHeadingEditorDialog> _ui;
};
} // namespace gcs