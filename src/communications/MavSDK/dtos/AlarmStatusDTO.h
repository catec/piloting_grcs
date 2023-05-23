#ifndef ALARM_STATUS_DTO_H_
#define ALARM_STATUS_DTO_H_

#include <common/IDTO.h>
#include <common/Types.h>

namespace gcs {

class AlarmStatusDTO : public IDTO
{
  public:
    virtual ~AlarmStatusDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ALARM_STATUS; }

    quint16& getIndex() Q_DECL_NOEXCEPT { return _index; }

    const quint16& getIndex() const Q_DECL_NOEXCEPT { return _index; }

    AlarmStatusType& getAlarmStatus() Q_DECL_NOEXCEPT { return _alarmStatus; }

    const AlarmStatusType& getAlarmStatus() const Q_DECL_NOEXCEPT { return _alarmStatus; }

    quint16& getErrorsCount() Q_DECL_NOEXCEPT { return _errorsCount; }

    const quint16& getErrorsCount() const Q_DECL_NOEXCEPT { return _errorsCount; }

    quint16& getWarningsCount() Q_DECL_NOEXCEPT { return _warningsCount; }

    const quint16& getWarningsCount() const Q_DECL_NOEXCEPT { return _warningsCount; }

    quint32& getStampMs() Q_DECL_NOEXCEPT { return _stampMs; }

    const quint32& getStampMs() const Q_DECL_NOEXCEPT { return _stampMs; }

    /// Operators
    AlarmStatusDTO& operator=(const AlarmStatusDTO& o) Q_DECL_NOEXCEPT
    {
        _index         = o._index;
        _alarmStatus   = o._alarmStatus;
        _errorsCount   = o._errorsCount;
        _warningsCount = o._warningsCount;
        _stampMs       = o._stampMs;

        return *this;
    }

    bool operator==(const AlarmStatusDTO& o) const Q_DECL_NOEXCEPT
    {
        return _index == o._index && _alarmStatus == o._alarmStatus && _errorsCount == o._errorsCount
            && _warningsCount == o._warningsCount && _stampMs == o._stampMs;
    }

    bool operator!=(const AlarmStatusDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    quint16         _index;
    AlarmStatusType _alarmStatus;
    quint16         _errorsCount;
    quint16         _warningsCount;
    quint32         _stampMs;
};

} // namespace gcs

#endif