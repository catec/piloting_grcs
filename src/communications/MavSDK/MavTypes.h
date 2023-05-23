#pragma once

#include <mavsdk/mavlink_include.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/alarm/alarm.h>
#include <mavsdk/plugins/checklist/checklist.h>
#include <mavsdk/plugins/command/command.h>
#include <mavsdk/plugins/hl_action/hl_action.h>
#include <mavsdk/plugins/inspection/inspection.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

namespace gcs {

inline const char* ToString(const mavsdk::ConnectionResult& res)
{
    switch (res) {
        case mavsdk::ConnectionResult::Success: {
            return "Success";
        }
        case mavsdk::ConnectionResult::Timeout: {
            return "Timeout";
        }
        case mavsdk::ConnectionResult::SocketError: {
            return "SocketError";
        }
        case mavsdk::ConnectionResult::BindError: {
            return "BindError";
        }
        case mavsdk::ConnectionResult::SocketConnectionError: {
            return "SocketConnectionError";
        }
        case mavsdk::ConnectionResult::ConnectionError: {
            return "ConnectionError";
        }
        case mavsdk::ConnectionResult::NotImplemented: {
            return "NotImplemented";
        }
        case mavsdk::ConnectionResult::SystemNotConnected: {
            return "SystemNotConnected";
        }
        case mavsdk::ConnectionResult::SystemBusy: {
            return "SystemBusy";
        }
        case mavsdk::ConnectionResult::CommandDenied: {
            return "CommandDenied";
        }
        case mavsdk::ConnectionResult::DestinationIpUnknown: {
            return "DestinationIpUnknown";
        }
        case mavsdk::ConnectionResult::ConnectionsExhausted: {
            return "ConnectionsExhausted";
        }
        case mavsdk::ConnectionResult::ConnectionUrlInvalid: {
            return "ConnectionUrlInvalid";
        }
        case mavsdk::ConnectionResult::BaudrateUnknown: {
            return "BaudrateUnknown";
        }
        default: {
            return "Unknown";
        }
    }
}

inline const char* ToString(const mavsdk::InspectionBase::Result& res)
{
    switch (res) {
        case mavsdk::InspectionBase::Result::Unknown: {
            return "Unknown";
        }
        case mavsdk::InspectionBase::Result::Success: {
            return "Success";
        }
        case mavsdk::InspectionBase::Result::ConnectionError: {
            return "ConnectionError";
        }
        case mavsdk::InspectionBase::Result::ProtocolError: {
            return "ProtocolError";
        }
        case mavsdk::InspectionBase::Result::Busy: {
            return "Busy";
        }
        case mavsdk::InspectionBase::Result::Timeout: {
            return "Timeout";
        }
        case mavsdk::InspectionBase::Result::InvalidArgument: {
            return "InvalidArgument";
        }
        case mavsdk::InspectionBase::Result::TransferCancelled: {
            return "TransferCancelled";
        }
        default: {
            return "Unknown";
        }
    }
}

inline const char* ToString(const mavsdk::InspectionBase::Ack& res)
{
    switch (res) {
        case mavsdk::InspectionBase::Ack::Accepted: {
            return "Accepted";
        }
        case mavsdk::InspectionBase::Ack::Error: {
            return "Error";
        }
        case mavsdk::InspectionBase::Ack::Unsupported: {
            return "Unsupported";
        }
        case mavsdk::InspectionBase::Ack::NoSpace: {
            return "NoSpace";
        }
        case mavsdk::InspectionBase::Ack::Invalid: {
            return "Invalid";
        }
        case mavsdk::InspectionBase::Ack::InvalidParam1: {
            return "InvalidParam1";
        }
        case mavsdk::InspectionBase::Ack::InvalidParam2: {
            return "InvalidParam2";
        }
        case mavsdk::InspectionBase::Ack::InvalidParam3: {
            return "InvalidParam3";
        }
        case mavsdk::InspectionBase::Ack::InvalidParam4: {
            return "InvalidParam4";
        }
        case mavsdk::InspectionBase::Ack::InvalidParam5: {
            return "InvalidParam5";
        }
        case mavsdk::InspectionBase::Ack::InvalidParam6: {
            return "InvalidParam6";
        }
        case mavsdk::InspectionBase::Ack::InvalidParam7: {
            return "InvalidParam7";
        }
        case mavsdk::InspectionBase::Ack::InvalidSequence: {
            return "InvalidSequence";
        }
        case mavsdk::InspectionBase::Ack::Cancelled: {
            return "Cancelled";
        }
        case mavsdk::InspectionBase::Ack::Unknown: {
            return "Unknown";
        }
        default: {
            return "Unknown";
        }
    }
}

inline const char* ToString(const mavsdk::ChecklistBase::Result& res)
{
    switch (res) {
        case mavsdk::ChecklistBase::Result::Unknown: {
            return "Unknown";
        }
        case mavsdk::ChecklistBase::Result::Success: {
            return "Success";
        }
        case mavsdk::ChecklistBase::Result::ConnectionError: {
            return "ConnectionError";
        }
        case mavsdk::ChecklistBase::Result::ProtocolError: {
            return "ProtocolError";
        }
        case mavsdk::ChecklistBase::Result::Busy: {
            return "Busy";
        }
        case mavsdk::ChecklistBase::Result::Timeout: {
            return "Timeout";
        }
        case mavsdk::ChecklistBase::Result::InvalidArgument: {
            return "InvalidArgument";
        }
        case mavsdk::ChecklistBase::Result::TransferCancelled: {
            return "TransferCancelled";
        }
        default: {
            return "Unknown";
        }
    }
}

inline const char* ToString(const mavsdk::AlarmBase::Result& res)
{
    switch (res) {
        case mavsdk::AlarmBase::Result::Unknown: {
            return "Unknown";
        }
        case mavsdk::AlarmBase::Result::Success: {
            return "Success";
        }
        case mavsdk::AlarmBase::Result::ConnectionError: {
            return "ConnectionError";
        }
        case mavsdk::AlarmBase::Result::ProtocolError: {
            return "ProtocolError";
        }
        case mavsdk::AlarmBase::Result::Busy: {
            return "Busy";
        }
        case mavsdk::AlarmBase::Result::Timeout: {
            return "Timeout";
        }
        case mavsdk::AlarmBase::Result::InvalidArgument: {
            return "InvalidArgument";
        }
        case mavsdk::AlarmBase::Result::TransferCancelled: {
            return "TransferCancelled";
        }
        default: {
            return "Unknown";
        }
    }
}

inline const char* ToString(const mavsdk::HLActionBase::Result& res)
{
    switch (res) {
        case mavsdk::HLActionBase::Result::Unknown: {
            return "Unknown";
        }
        case mavsdk::HLActionBase::Result::Success: {
            return "Success";
        }
        case mavsdk::HLActionBase::Result::ConnectionError: {
            return "ConnectionError";
        }
        case mavsdk::HLActionBase::Result::ProtocolError: {
            return "ProtocolError";
        }
        case mavsdk::HLActionBase::Result::Busy: {
            return "Busy";
        }
        case mavsdk::HLActionBase::Result::Timeout: {
            return "Timeout";
        }
        case mavsdk::HLActionBase::Result::InvalidArgument: {
            return "InvalidArgument";
        }
        case mavsdk::HLActionBase::Result::TransferCancelled: {
            return "TransferCancelled";
        }
        default: {
            return "Unknown";
        }
    }
}

inline const char* ToString(const mavsdk::CommandBase::Result& res)
{
    switch (res) {
        case mavsdk::CommandBase::Result::Success: {
            return "Success";
        }
        case mavsdk::CommandBase::Result::ConnectionError: {
            return "ConnectionError";
        }
        case mavsdk::CommandBase::Result::Busy: {
            return "Busy";
        }
        case mavsdk::CommandBase::Result::Timeout: {
            return "Timeout";
        }
        case mavsdk::CommandBase::Result::Cancelled: {
            return "Cancelled";
        }
        case mavsdk::CommandBase::Result::InProgress: {
            return "InProgress";
        }
        default: {
            return "Unknown";
        }
    }
}

inline const char* ToString(const MAV_RESULT& res)
{
    switch (res) {
        case MAV_RESULT::MAV_RESULT_ACCEPTED: {
            return "MAV_RESULT_ACCEPTED";
        }
        case MAV_RESULT::MAV_RESULT_TEMPORARILY_REJECTED: {
            return "MAV_RESULT_TEMPORARILY_REJECTED";
        }
        case MAV_RESULT::MAV_RESULT_DENIED: {
            return "MAV_RESULT_DENIED";
        }
        case MAV_RESULT::MAV_RESULT_UNSUPPORTED: {
            return "MAV_RESULT_UNSUPPORTED";
        }
        case MAV_RESULT::MAV_RESULT_FAILED: {
            return "MAV_RESULT_FAILED";
        }
        case MAV_RESULT::MAV_RESULT_IN_PROGRESS: {
            return "MAV_RESULT_IN_PROGRESS";
        }
        case MAV_RESULT::MAV_RESULT_CANCELLED: {
            return "MAV_RESULT_CANCELLED";
        }
        default: {
            return "Unknown";
        }
    }
}
} // namespace gcs