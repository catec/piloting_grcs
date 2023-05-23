#pragma once

#include <common/LoggingManager.h>
#include <common/Types.h>

#include <QTextEdit>
#include <memory>

namespace Ui {
class ConsoleDisplayWidget;
}

namespace gcs {

class GuiDTO;

class ConsoleDisplayWidget : public QTextEdit
{
    Q_OBJECT

  public:
    explicit ConsoleDisplayWidget(GuiDTO& guiDTO, QWidget* parent = nullptr);
    virtual ~ConsoleDisplayWidget();

    void addMessage(const MsgType&, const QString&);

    void clearMessages();

    void updateLogging(const QString&);

  private:
    std::unique_ptr<Ui::ConsoleDisplayWidget> _ui;

    GuiDTO& _guiDTO;

    std::unique_ptr<LoggingManager> _loggingManager;
};

} // namespace gcs
