#ifndef SHARED_SENSOR_TYPES_H
#define SHARED_SENSOR_TYPES_H

#include <stdint.h>  // uint16_t, float 등을 위해
#include <stdbool.h> // C에서 bool 타입을 사용하기 위해 (C++에서는 내장)

// 센서 측정 요청 타입 정의 (C 스타일 enum)
typedef enum {
    C_SENSOR_REQUEST_NONE,        // 이름 충돌 방지를 위해 접두사 C_ 추가 (선택적)
    C_SENSOR_REQUEST_MEASURE_UP,
    C_SENSOR_REQUEST_MEASURE_RIGHT,
    C_SENSOR_REQUEST_MEASURE_DOWN,
    C_SENSOR_REQUEST_MEASURE_LEFT
} CSensorRequestType_t; // C 스타일 typedef 이름

// Model -> SensorTask 메시지 구조체 (C 스타일)
typedef struct {
    CSensorRequestType_t requestType;
    // 필요시 추가 C 호환 필드
} CSensorRequestMessage_t;

// SensorTask -> Model 메시지 구조체 (C 스타일)
typedef struct {
    CSensorRequestType_t originalRequestType;
    float distance_cm;
    bool  success;
} CSensorResultMessage_t;

typedef enum {
    PROXIMITY_CMD_START_POLLING_IMMEDIATE, // 즉시 폴링 시작
    PROXIMITY_CMD_START_POLLING_DELAYED,   // 지연 후 폴링 시작
    PROXIMITY_CMD_STOP_POLLING             // 폴링 중지
} ProximityCommand_t;

typedef struct {
    ProximityCommand_t command;
    uint32_t delay_ms; // PROXIMITY_CMD_START_POLLING_DELAYED 일 때 사용될 지연 시간
} ModelToProximityMsg_t;

typedef enum {
    PROXIMITY_EVENT_DETECTED
} ProximityEvent_t;

typedef struct {
    ProximityEvent_t event;
} ProximityToModelMsg_t;

#endif // SHARED_SENSOR_TYPES_H
