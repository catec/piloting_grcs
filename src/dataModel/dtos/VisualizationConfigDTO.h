#pragma once

#include <QColor>
#include <QVariant>

#include "common/IDTO.h"

namespace gcs {
class VisualizationConfigDTO : public IDTO
{
  public:
    virtual ~VisualizationConfigDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::VISUALIZATION_CONFIG; }

    bool&       getShowFloorGrid() Q_DECL_NOEXCEPT { return _showFloorGrid; }
    const bool& getShowFloorGrid() const Q_DECL_NOEXCEPT { return _showFloorGrid; }

    bool&       getShowItemsText() Q_DECL_NOEXCEPT { return _showItemsText; }
    const bool& getShowItemsText() const Q_DECL_NOEXCEPT { return _showItemsText; }

    bool&       getShowGlobalAxis() Q_DECL_NOEXCEPT { return _showGlobalAxis; }
    const bool& getShowGlobalAxis() const Q_DECL_NOEXCEPT { return _showGlobalAxis; }

    bool&       getShowHomePosition() Q_DECL_NOEXCEPT { return _showHomePosition; }
    const bool& getShowHomePosition() const Q_DECL_NOEXCEPT { return _showHomePosition; }

    float&       getWaypointSizeScale() Q_DECL_NOEXCEPT { return _wptsSizeScale; }
    const float& getWaypointSizeScale() const Q_DECL_NOEXCEPT { return _wptsSizeScale; }

    float&       getInspPtSizeScale() Q_DECL_NOEXCEPT { return _inspPtSizeScale; }
    const float& getInspPtSizeScale() const Q_DECL_NOEXCEPT { return _inspPtSizeScale; }

    float&       getInspLocationTransp() Q_DECL_NOEXCEPT { return _inspLocationTransp; }
    const float& getInspLocationTransp() const Q_DECL_NOEXCEPT { return _inspLocationTransp; }

    float&       getPointCloudPtsSize() Q_DECL_NOEXCEPT { return _pointCloudPtsSize; }
    const float& getPointCloudPtsSize() const Q_DECL_NOEXCEPT { return _pointCloudPtsSize; }

    QVariant&       getPointCloudColorStyle() Q_DECL_NOEXCEPT { return _pointCloudColorStyle; }
    const QVariant& getPointCloudColorStyle() const Q_DECL_NOEXCEPT { return _pointCloudColorStyle; }

    QColor&       getPointCloudColor() Q_DECL_NOEXCEPT { return _pointCloudColor; }
    const QColor& getPointCloudColor() const Q_DECL_NOEXCEPT { return _pointCloudColor; }

    /// Operators
    VisualizationConfigDTO& operator=(const VisualizationConfigDTO& o) Q_DECL_NOEXCEPT
    {
        _showFloorGrid        = o._showFloorGrid;
        _wptsSizeScale        = o._wptsSizeScale;
        _inspPtSizeScale      = o._inspPtSizeScale;
        _inspLocationTransp   = o._inspLocationTransp;
        _pointCloudPtsSize    = o._pointCloudPtsSize;
        _pointCloudColorStyle = o._pointCloudColorStyle;
        _pointCloudColor      = o._pointCloudColor;
        _showItemsText        = o._showItemsText;
        _showGlobalAxis       = o._showGlobalAxis;
        _showHomePosition     = o._showHomePosition;

        return *this;
    }

    bool operator==(const VisualizationConfigDTO& o) const Q_DECL_NOEXCEPT
    {
        return _showFloorGrid == o._showFloorGrid && _wptsSizeScale == o._wptsSizeScale
            && _inspPtSizeScale == o._inspPtSizeScale && _pointCloudPtsSize == o._pointCloudPtsSize
            && _inspLocationTransp == o._inspLocationTransp && _pointCloudColorStyle == o._pointCloudColorStyle
            && _pointCloudColor == o._pointCloudColor && _showItemsText == o._showItemsText
            && _showGlobalAxis == o._showGlobalAxis && _showHomePosition == o._showHomePosition;
    }

    bool operator!=(const VisualizationConfigDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    bool _showFloorGrid{true};

    float    _wptsSizeScale{1.0f};
    float    _inspPtSizeScale{1.0f};
    float    _pointCloudPtsSize{0.05f};
    float    _inspLocationTransp{0.2f};
    QVariant _pointCloudColorStyle{"RGB8"};
    QColor   _pointCloudColor;
    bool     _showItemsText{true};
    bool     _showGlobalAxis{true};
    bool     _showHomePosition{true};
};
} // namespace gcs
