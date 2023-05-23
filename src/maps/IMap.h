#pragma once

#include <core/IAction.h>

#include <memory>

class QWidget;
class QVBoxLayout;

namespace gcs {

class MapAdapterDTO;

class IMap
{
  public:
    template <typename _Type, typename... _T>
    static std::unique_ptr<IMap> create(_T&&... _arg)
    {
        return std::make_unique<_Type>(std::forward<_T>(_arg)...);
    }

    virtual ~IMap() {}

    /// @brief Return widget associated with map visualizer
    virtual QWidget* getMapView() = 0;

    /// @brief Initialize render of visualizer 3D
    virtual void initializeRender() = 0;

    /// public Q_SLOTS:
    /// @brief Initialize displays of visualizer 3D
    virtual void initialize() = 0;

    /// @brief Reset all displays of visualizer
    virtual void resetVisualizer() = 0;

    virtual void manageCoreDTO(QSharedPointer<IDTO>) = 0;

    /// Q_SIGNALS:
    virtual void sendDTOToCore(ActionType actionType, QSharedPointer<IDTO> dto = nullptr) = 0;
};

} // namespace gcs
