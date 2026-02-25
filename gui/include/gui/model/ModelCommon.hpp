// gui/include/gui/model/ModelCommon.hpp
#ifndef MODELCOMMON_HPP
#define MODELCOMMON_HPP
#include <stdint.h>


enum class OperationDirection
{
    NONE,
    UP,
    RIGHT,
    DOWN,
    LEFT
};

typedef enum {
    SENSOR_REQUEST_NONE,
    SENSOR_REQUEST_MEASURE_US0,
    SENSOR_REQUEST_MEASURE_US1
} SensorRequestType_t;

typedef struct {
    SensorRequestType_t requestType;
    // OperationDirection originalDirection; // 선택적: 어떤 버튼 클릭으로 이 요청이 왔는지 정보 저장용
} SensorRequestMessage_t;

typedef struct {
    SensorRequestType_t originalRequestType; // 어떤 요청에 대한 응답인지 (US0 또는 US1)
    float distance_cm;
    bool  success; // 측정 성공 여부
} SensorResultMessage_t;

const int MAX_DIRECTIONS = 4;

#endif // MODELCOMMON_HPP
