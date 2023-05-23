#pragma once

#include <dataModel/DataModel.h>

#include <memory>

class QThread;

namespace gcs {

class ICommunications;
class DDHLComms;
class MainWindowUi;
class IMap;
class Core;

class MainProgram
{
  public:
    explicit MainProgram();
    virtual ~MainProgram();

  private:
    void initializeCore();

    void connectMapToCore();
    void connectViewToCore();
    void connectCommsToCore();
    void connectCommsDDHLToCore();

    void setMultiThread();

    DataModel                        _dataModel;
    std::unique_ptr<ICommunications> _comms;
    std::unique_ptr<DDHLComms>       _commsDDHL;
    std::unique_ptr<MainWindowUi>    _view;
    std::unique_ptr<IMap>            _map;
    std::unique_ptr<Core>            _core;
    std::unique_ptr<QThread>         _commsThread;
    std::unique_ptr<QThread>         _commsDDHLThread;
    std::unique_ptr<QThread>         _coreThread;
};

} // namespace gcs
