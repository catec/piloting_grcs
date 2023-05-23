#ifndef ALARM_LIST_DTO_H_
#define ALARM_LIST_DTO_H_

#include <common/IDTO.h>

#include "AlarmItemDTO.h"

namespace gcs {

class AlarmListDTO : public IDTO
{
  public:
    virtual ~AlarmListDTO() {}

    /// Accessors
    virtual DTOType getDTOType() override { return DTOType::ALARM_LIST; }

    QList<AlarmItemDTO>& getAlarmList() Q_DECL_NOEXCEPT { return _alarmList; }

    const QList<AlarmItemDTO>& getAlarmList() const Q_DECL_NOEXCEPT { return _alarmList; }

    /// Operators
    AlarmListDTO& operator=(const AlarmListDTO& o) Q_DECL_NOEXCEPT
    {
        _alarmList = o._alarmList;

        return *this;
    }

    bool operator==(const AlarmListDTO& o) const Q_DECL_NOEXCEPT { return _alarmList == o._alarmList; }

    bool operator!=(const AlarmListDTO& o) const Q_DECL_NOEXCEPT { return !(*this == o); }

  private:
    QList<AlarmItemDTO> _alarmList;
};

} // namespace gcs

#endif