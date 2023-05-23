#include "MainProgram.h"

#include <QsLog/QsLog.h>
#include <communications/DDHL/DDHLComms.h>
#include <communications/MavSDK/MavsdkComms.h>
#include <core/Core.h>
#include <core/actions/ActionFactory.h>
#include <core/managers/ActionManager.h>
#include <gui/MainWindowUi.h>
#include <maps/Map3d.h>

#include <QMetaObject>
#include <QThread>

using namespace QsLogging;

namespace gcs {

MainProgram::MainProgram() :
        _comms(ICommunications::create<MavsdkComms>()),
        _commsDDHL(std::make_unique<DDHLComms>()),
        _view(std::make_unique<MainWindowUi>()),
        _map(IMap::create<Map3d>()),
        _core(std::make_unique<Core>()),
        _commsThread(std::make_unique<QThread>()),
        _commsDDHLThread(std::make_unique<QThread>()),
        _coreThread(std::make_unique<QThread>())
{
    QLOG_TRACE() << "MainProgram::MainProgram()";

    initializeCore();

    connectCommsToCore();
    connectCommsDDHLToCore();
    connectViewToCore();
    connectMapToCore();

    setMultiThread();

    _view->setupInit(_map.get()->getMapView());
    _map->initializeRender();
}

MainProgram::~MainProgram()
{
    QLOG_TRACE() << "MainProgram::~MainProgram()";

    if (_comms->isConnected()) {
        auto commsObj = dynamic_cast<QObject*>(_comms.get());
        QMetaObject::invokeMethod(commsObj, "disconnectFrom", Qt::BlockingQueuedConnection);
    }

    _coreThread->quit();
    _commsThread->quit();
    _commsDDHLThread->quit();

    _coreThread->wait();
    _commsThread->wait();
    _commsDDHLThread->wait();
}

void MainProgram::initializeCore()
{
    QLOG_TRACE() << "MainProgram::initializeCore()";

    auto manager = std::make_unique<ActionManager>();

    auto actionType = ActionType::First;
    while (actionType <= ActionType::Last) {
        manager->addAction(ActionFactory::create(actionType, _dataModel, _comms.get(), _commsDDHL.get(), _map.get()));

        actionType = static_cast<ActionType>(static_cast<uint8_t>(actionType) + 1);
    }

    _core->setManager(std::move(manager));
}

void MainProgram::connectMapToCore()
{
    QLOG_TRACE() << "MainProgram::connectMapToCore()";

    auto mapObj = dynamic_cast<QObject*>(_map.get());

    /// IMap -> Core
    QObject::connect(
            mapObj,
            SIGNAL(sendDTOToCore(ActionType, QSharedPointer<IDTO>)),
            _core.get(),
            SLOT(managePluginDTO(ActionType, QSharedPointer<IDTO>)));

    /// Core -> IMap
    QObject::connect(
            _core.get(),
            SIGNAL(sendDTOToPlugin(QSharedPointer<IDTO>)),
            mapObj,
            SLOT(manageCoreDTO(QSharedPointer<IDTO>)));
}

void MainProgram::connectViewToCore()
{
    QLOG_TRACE() << "MainProgram::connectViewToCore()";

    /// MainWindowUi -> Core
    QObject::connect(
            _view.get(),
            SIGNAL(sendDTOToCore(ActionType, QSharedPointer<IDTO>)),
            _core.get(),
            SLOT(managePluginDTO(ActionType, QSharedPointer<IDTO>)));

    /// Core -> MainWindowUi
    QObject::connect(
            _core.get(),
            SIGNAL(sendDTOToPlugin(QSharedPointer<IDTO>)),
            _view.get(),
            SLOT(manageCoreDTO(QSharedPointer<IDTO>)));
}

void MainProgram::connectCommsToCore()
{
    QLOG_TRACE() << "MainProgram::connectCommsToCore()";

    auto commsObj = dynamic_cast<QObject*>(_comms.get());

    /// ICommunications -> Core
    QObject::connect(
            commsObj,
            SIGNAL(sendDTOToCore(ActionType, QSharedPointer<IDTO>)),
            _core.get(),
            SLOT(managePluginDTO(ActionType, QSharedPointer<IDTO>)));
}

void MainProgram::connectCommsDDHLToCore()
{
    QLOG_TRACE() << "MainProgram::connectCommsDDHLToCore()";

    auto commsDDHLObj = dynamic_cast<QObject*>(_commsDDHL.get());

    /// DDHLComms -> Core
    QObject::connect(
            commsDDHLObj,
            SIGNAL(sendDTOToCore(ActionType, QSharedPointer<IDTO>)),
            _core.get(),
            SLOT(managePluginDTO(ActionType, QSharedPointer<IDTO>)));
}

void MainProgram::setMultiThread()
{
    QLOG_TRACE() << "MainProgram::setMultiThread()";

    auto commsObj = dynamic_cast<QObject*>(_comms.get());
    commsObj->moveToThread(_commsThread.get());
    _commsThread->setObjectName("Comms");
    _commsThread->start();

    auto commsDDHLObj = dynamic_cast<QObject*>(_commsDDHL.get());
    commsDDHLObj->moveToThread(_commsDDHLThread.get());
    _commsDDHLThread->setObjectName("CommsDDHL");
    _commsDDHLThread->start();

    _core->moveToThread(_coreThread.get());
    _coreThread->setObjectName("Core");
    _coreThread->start();
}

} // namespace gcs
