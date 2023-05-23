#pragma once

#include <QDialog>
#include <memory>

#include "gcs_gui_export.h"

namespace Ui {
class HomeParametersDialog;
}

namespace gcs {
class HomeDTO;

class PILOTING_GRCS_GUI_EXPORT HomeParametersDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit HomeParametersDialog(QWidget* parent = nullptr);
    virtual ~HomeParametersDialog();

    void setHomeParameters(const HomeDTO&);

    float getPositionX() const;
    float getPositionY() const;
    float getPositionZ() const;

    float getYawAngle() const;

  private:
    std::unique_ptr<Ui::HomeParametersDialog> _ui;
};

} // namespace gcs
