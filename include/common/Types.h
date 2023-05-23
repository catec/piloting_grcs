#pragma once

#include <QMetaType>
#include <QString>

namespace gcs {

enum class AppStatusType : uint8_t { Panning = 0, CreatingRoute, EditingRoute, EditingLoadedHome, Measuring };

inline const char* ToString(const AppStatusType v)
{
    switch (v) {
        case AppStatusType::Panning: {
            return "Panning";
        }
        case AppStatusType::CreatingRoute: {
            return "CreatingRoute";
        }
        case AppStatusType::EditingRoute: {
            return "EditingRoute";
        }
        case AppStatusType::EditingLoadedHome: {
            return "EditingLoadedHome";
        }
        case AppStatusType::Measuring: {
            return "Measuring";
        }
        default: {
            return "Unknown";
        }
    }
}

enum class ViewModeType : uint8_t { TwoD = 0, ThreeD };

inline const char* ToString(const ViewModeType v)
{
    switch (v) {
        case ViewModeType::TwoD: {
            return "TwoD";
        }
        case ViewModeType::ThreeD: {
            return "ThreeD";
        }
        default: {
            return "Unknown";
        }
    }
}

enum class DialogType : uint8_t { Unknown = 0, WaypointHeadingEdition };

enum class MissionStatusType : uint8_t { NoMission = 0, WithChanges, Saved };

inline const char* ToString(const MissionStatusType v)
{
    switch (v) {
        case MissionStatusType::NoMission: {
            return "NoMission";
        }
        case MissionStatusType::WithChanges: {
            return "WithChanges";
        }
        case MissionStatusType::Saved: {
            return "Saved";
        }
        default: {
            return "Unknown";
        }
    }
}

enum class AlarmStatusType : uint8_t { OK = 0, WARNING, ERROR, UNKNOWN };

inline const char* ToString(const AlarmStatusType v)
{
    switch (v) {
        case AlarmStatusType::OK: {
            return "OK";
        }
        case AlarmStatusType::WARNING: {
            return "WARNING";
        }
        case AlarmStatusType::ERROR: {
            return "ERROR";
        }
        case AlarmStatusType::UNKNOWN:
        default: {
            return "UNKNOWN";
        }
    }
}

enum class MsgType : uint8_t { INFO = 0, WARNING, ERROR, UNKNOWN };

inline const char* ToString(const MsgType v)
{
    switch (v) {
        case MsgType::INFO: {
            return "INFO";
        }
        case MsgType::WARNING: {
            return "WARN";
        }
        case MsgType::ERROR: {
            return "ERROR";
        }
        case MsgType::UNKNOWN:
        default: {
            return "UNKNOWN";
        }
    }
}

enum class WaypointType : uint8_t {
    POSE = 0,
    ACTION,
    UNKNOWN
    // TASK
};

inline const char* ToString(const WaypointType v)
{
    switch (v) {
        case WaypointType::POSE: {
            return "POSE";
        }
        case WaypointType::ACTION: {
            return "ACTION";
        }
        default: {
            return "UNKNOWN";
        }
    }
}

/// \note Each int is related with the site
enum class SiteType : uint8_t {
    VIADUCT  = 1,
    REFINERY = 2,
    TUNNEL   = 3,
    UNKNOWN
    // TASK
};

inline const char* ToString(const SiteType v)
{
    switch (v) {
        case SiteType::VIADUCT: {
            return "VIADUCT";
        }
        case SiteType::REFINERY: {
            return "REFINERY";
        }
        case SiteType::TUNNEL: {
            return "TUNNEL";
        }
        default: {
            return "UNKNOWN";
        }
    }
}

inline WaypointType ToWpType(const std::string v)
{
    if (v == "POSE") {
        return WaypointType::POSE;
    } else if (v == "ACTION") {
        return WaypointType::ACTION;
    } else {
        return WaypointType::UNKNOWN;
    }
}

enum class InspectionTaskLocationType : uint8_t { POINT = 0, AREA, PART };

inline const char* ToString(const InspectionTaskLocationType v)
{
    switch (v) {
        case InspectionTaskLocationType::POINT: {
            return "Point";
        }
        case InspectionTaskLocationType::AREA: {
            return "Area";
        }
        case InspectionTaskLocationType::PART: {
            return "Part";
        }
        default: {
            return "Unknown";
        }
    }
}

enum class MeasurementsType : uint8_t { PHOTOGRAPHY = 0, ULTRASONIC };

inline const char* ToString(const MeasurementsType v)
{
    switch (v) {
        case MeasurementsType::PHOTOGRAPHY: {
            return "Photography";
        }
        case MeasurementsType::ULTRASONIC: {
            return "Ultrasonic";
        }
        default: {
            return "Unknown";
        }
    }
}

enum class CheckStatus : uint8_t { NONE = 0, OK, FAIL };

inline const char* ToString(const CheckStatus v)
{
    switch (v) {
        case CheckStatus::NONE: {
            return "NONE";
        }
        case CheckStatus::OK: {
            return "OK";
        }
        case CheckStatus::FAIL: {
            return "FAIL";
        }
        default: {
            return "UNKNOWN";
        }
    }
}

enum class Axis : uint8_t { X = 0, Y, Z };

inline const char* ToString(const Axis ax)
{
    switch (ax) {
        case Axis::X: {
            return "x";
        }
        case Axis::Y: {
            return "y";
        }
        case Axis::Z: {
            return "z";
        }
        default: {
            return "UNKNOWN";
        }
    }
}

enum class DDHLEndpointMode : uint8_t { IP = 0, URL, UNKNOWN };

inline const char* ToString(const DDHLEndpointMode endpoint)
{
    switch (endpoint) {
        case DDHLEndpointMode::IP: {
            return "ip";
        }
        case DDHLEndpointMode::URL: {
            return "url";
        }
        default: {
            return "UNKNOWN";
        }
    }
}

inline DDHLEndpointMode ToDDHLEndpointMode(const QString& mode)
{
    if (mode == "ip") {
        return DDHLEndpointMode::IP;
    } else if (mode == "url") {
        return DDHLEndpointMode::URL;
    } else {
        return DDHLEndpointMode::UNKNOWN;
    }
}

enum class CommsProgressType : uint8_t { START = 0, STOP };

enum RouteOptionsFlag { NoShow = 0x0, ShowLocal = 0x1, ShowLoaded = 0x2 };
Q_DECLARE_FLAGS(RouteOptions, RouteOptionsFlag)

} // namespace gcs

Q_DECLARE_OPERATORS_FOR_FLAGS(gcs::RouteOptions)
