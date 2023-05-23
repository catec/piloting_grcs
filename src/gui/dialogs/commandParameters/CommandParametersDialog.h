#pragma once

#include <QDialog>
#include <QDoubleSpinBox>
#include <QList>
#include <QMap>
#include <memory>

namespace Ui {
class CommandParametersDialog;
}

namespace gcs {
class ParamDTO;
class HLActionItemDTO;
class CommandParametersDialog : public QDialog
{
    Q_OBJECT

  public:
    explicit CommandParametersDialog(QWidget* parent = nullptr);
    virtual ~CommandParametersDialog();

    void            setHLActionInfo(const HLActionItemDTO&);
    void            accept();
    QList<ParamDTO> getParamList();

  private:
    void addEmptyParameters();
    void insertParamList(const QList<ParamDTO>&);

    QList<ParamDTO>                              _paramList;
    std::unique_ptr<Ui::CommandParametersDialog> _ui;
    QMap<QDoubleSpinBox*, ParamDTO>              _paramsMap;
};

} // namespace gcs
