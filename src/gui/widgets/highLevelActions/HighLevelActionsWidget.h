#pragma once

#include <communications/MavSDK/dtos/CommandLongDTO.h>

#include <QMap>
#include <QPushButton>
#include <QWidget>
#include <memory>

namespace Ui {
class HighLevelActionsWidget;
}

namespace gcs {
class HLActionListDTO;
class HLActionItemDTO;
class ParamDTO;
class HighLevelActionsWidget : public QWidget
{
    Q_OBJECT

  public:
    explicit HighLevelActionsWidget(QWidget* parent = nullptr);
    virtual ~HighLevelActionsWidget();

    void receiveHLActionList(const HLActionListDTO&);
    void resetHLActionList();

  private Q_SLOTS:
    void HLActionButtonClicked();

  private:
    std::unique_ptr<Ui::HighLevelActionsWidget> _ui;

    QMap<QPushButton*, HLActionItemDTO> _hlActionListMap;

  Q_SIGNALS:
    void sendCommand(QSharedPointer<CommandLongDTO>);
};

} // namespace gcs
