#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "main.h"
#include "cmsis_os.h" // osMessageQueuePut, osMessageQueueGet 등을 위해
#include "shared_sensor_types.h"

#define SERVO_MIN_PULSE 3274
#define SERVO_MAX_PULSE 6548
#define SERVO_INIT_POSITION_PULSE 4911
#define SERVO_NEUT_POSITION_PULSE 4911
#define SERVO_EVEN_POSITION_PULSE 3474
#define SERVO_ODD_POSITION_PULSE  6548
#define SERVO_HIGH_POSITION_PULSE 3174
#define SERVO_LOW_POSITION_PULSE  6448

extern osMessageQueueId_t sensorRequestQueueHandle;
extern osMessageQueueId_t sensorResultQueueHandle;

extern osMessageQueueId_t modelToProximityQueueHandle;
extern osMessageQueueId_t proximityToModelQueueHandle;

Model::Model() :
    modelListener(0),
    currentSequenceState(ServoSensorSequenceState::IDLE),
    currentOperationDirection(OperationDirection::NONE),
    stateStartTime(0),
    // 화면 관리 멤버 변수 초기화
    isScreenOff_m(false),       // 초기 화면은 켜짐
    lastActivityTime_m(0),
    screenTimeoutMs_m(30000)  // 30초 (30000 ms)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_INIT_POSITION_PULSE);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_NEUT_POSITION_PULSE);
    for (int i = 0; i < MAX_DIRECTIONS; ++i) { // MAX_DIRECTIONS 사용
        capacityValues[i] = 0.0f; // float으로 초기화
    }

    lastActivityTime_m = HAL_GetTick(); // 현재 시간으로 마지막 활동 시간 초기화
    controlBacklight(true);             // 애플리케이션 시작 시 백라이트 켬
}

void Model::processProximityEvents() // Model::tick() 내부에서 주기적으로 호출되거나, 메시지 수신 기반으로 처리
{
    ProximityToModelMsg_t rxProxMsg;
    osStatus_t status = osMessageQueueGet(proximityToModelQueueHandle, &rxProxMsg, NULL, 0U); // Non-blocking

    if (status == osOK)
    {
        if (rxProxMsg.event == PROXIMITY_EVENT_DETECTED)
        {
            if (isScreenOff_m) // 화면이 꺼져있을 때만 반응
            {
                // recordUserActivity(); // 화면 켜고 타임아웃 리셋
                // 또는 직접 turnScreenOnInternal() 호출 후 Presenter에 화면 전환 요청
                turnScreenOnInternal(); // 백라이트 켜고 isScreenOff_m = false;
                if (modelListener) {
                    modelListener->requestGoToMainScreen();
                }
            }
        }
    }
}

void Model::startServoSensorSequence(OperationDirection dir)
{
    if (isScreenOff_m) {
        turnScreenOnInternal();
    }

    if (currentSequenceState == ServoSensorSequenceState::IDLE) {
        currentOperationDirection = dir;
        currentSequenceState = ServoSensorSequenceState::START_SEQUENCE;
        stateStartTime = HAL_GetTick();
        recordUserActivity(); // 시퀀스 시작도 사용자 활동으로 간주하여 타임아웃 리셋
    } else {
    }
}

void Model::tick()
{
    uint32_t currentTime = HAL_GetTick();

    processProximityEvents();

    if (isScreenOff_m) {
        return;
    }

    switch (currentSequenceState)
    {
        case ServoSensorSequenceState::IDLE:
            processIdleState();
            break;
        case ServoSensorSequenceState::START_SEQUENCE:
            processStartSequenceState();
            break;
        case ServoSensorSequenceState::MOVING_CLASSIFIER1:
            processMovingClassifierState1(currentTime);
            break;
        case ServoSensorSequenceState::MOVING_CLASSIFIER2:
            processMovingClassifierState2(currentTime);
            break;
        case ServoSensorSequenceState::ALIGNING_SENSOR:
            processAligningSensorState(currentTime);
            break;
        case ServoSensorSequenceState::SENDING_MEASUREMENT_REQUEST:
            processSendingMeasurementRequestState();
            break;
        case ServoSensorSequenceState::WAITING_FOR_SENSOR_RESULT:
            processWaitingForSensorResultState(currentTime);
            break;
        case ServoSensorSequenceState::RETURNING_CLASSIFIER_TO_INIT:
            processReturningClassifierToInitState(currentTime);
            break;
        case ServoSensorSequenceState::SEQUENCE_COMPLETE:
            processSequenceCompleteState();
            break;
        default:
            currentSequenceState = ServoSensorSequenceState::IDLE;
            currentOperationDirection = OperationDirection::NONE;
            break;
    }
}

void Model::processIdleState()
{
    checkScreenTimeout();
}

void Model::processStartSequenceState()
{
    touchgfx_printf("State: START_SEQUENCE -> MOVING_CLASSIFIER\n");
    currentSequenceState = ServoSensorSequenceState::MOVING_CLASSIFIER1;
    stateStartTime = HAL_GetTick();

    moveClassifierToPosition1(currentOperationDirection);
}

void Model::processMovingClassifierState1(uint32_t currentTime)
{
    if (currentTime - stateStartTime >= CLASSIFIER_MOVE_TIME) {
        touchgfx_printf("State: MOVING_CLASSIFIER -> ALIGNING_SENSOR\n");
        currentSequenceState = ServoSensorSequenceState::MOVING_CLASSIFIER2;
        stateStartTime = currentTime; // 새 상태 시작 시간 기록

        moveClassifierToPosition2(currentOperationDirection);
    }
}

void Model::processMovingClassifierState2(uint32_t currentTime)
{
    if (currentTime - stateStartTime >= CLASSIFIER_MOVE_TIME) {
        touchgfx_printf("State: MOVING_CLASSIFIER -> ALIGNING_SENSOR\n");
        currentSequenceState = ServoSensorSequenceState::ALIGNING_SENSOR;
        stateStartTime = currentTime; // 새 상태 시작 시간 기록

        alignSensorServo();
    }
}

void Model::processAligningSensorState(uint32_t currentTime)
{
    if (currentTime - stateStartTime >= SENSOR_ALIGN_TIME) {
        touchgfx_printf("State: ALIGNING_SENSOR -> SENDING_MEASUREMENT_REQUEST\n");
        currentSequenceState = ServoSensorSequenceState::SENDING_MEASUREMENT_REQUEST;
        stateStartTime = currentTime; // 새 상태 시작 시간 기록
    }
}

void Model::processSendingMeasurementRequestState()
{
    CSensorRequestMessage_t c_reqMsg; // <--- C 호환 구조체로 메시지 생성
    c_reqMsg.requestType = C_SENSOR_REQUEST_NONE; // C 호환 enum 사용

    // Model 내부의 currentOperationDirection (enum class) 값을 기반으로 C 호환 enum 값 설정
    switch (currentOperationDirection)
    {
        case OperationDirection::UP:
            c_reqMsg.requestType = C_SENSOR_REQUEST_MEASURE_UP;
            break;
        case OperationDirection::RIGHT:
            c_reqMsg.requestType = C_SENSOR_REQUEST_MEASURE_RIGHT;
            break;
        case OperationDirection::DOWN:
            c_reqMsg.requestType = C_SENSOR_REQUEST_MEASURE_DOWN;
            break;
        case OperationDirection::LEFT:
            c_reqMsg.requestType = C_SENSOR_REQUEST_MEASURE_LEFT;
            break;
        default:
            touchgfx_printf("Error: Invalid op_dir in SENDING_MEASUREMENT_REQUEST.\n");
            currentSequenceState = ServoSensorSequenceState::IDLE;
            currentOperationDirection = OperationDirection::NONE; // C++ enum class 사용
            return;
    }

    if (c_reqMsg.requestType != C_SENSOR_REQUEST_NONE) {
        if (osMessageQueuePut(sensorRequestQueueHandle, &c_reqMsg, 0U, 0U) == osOK) { // C 구조체 전송
            touchgfx_printf("C_ReqMsg sent. Type: %d (Dir: %d)\n", c_reqMsg.requestType, (int)currentOperationDirection);
            currentSequenceState = ServoSensorSequenceState::WAITING_FOR_SENSOR_RESULT;
            stateStartTime = HAL_GetTick();
        } else {
            touchgfx_printf("Failed to send C_ReqMsg.\n");
            currentSequenceState = ServoSensorSequenceState::IDLE;
            currentOperationDirection = OperationDirection::NONE;
        }
    } else {
        currentSequenceState = ServoSensorSequenceState::IDLE;
        currentOperationDirection = OperationDirection::NONE;
    }
}

// 새로운 상태: SensorTask로부터 결과 대기
void Model::processWaitingForSensorResultState(uint32_t currentTime)
{
    CSensorResultMessage_t c_resMsg; // <--- C 호환 구조체로 메시지 수신
    osStatus_t status = osMessageQueueGet(sensorResultQueueHandle, &c_resMsg, NULL, 10);
    int index = (static_cast<int>(currentOperationDirection)) - 1;

    if (status == osOK) {
        if (index >= 0 && index < MAX_DIRECTIONS) {
            if (c_resMsg.success) {
                capacityValues[index] = int(100*(30-c_resMsg.distance_cm)/15)
                		;
            } else {
                capacityValues[index] = 43.5f;
            }
        }
        currentSequenceState = ServoSensorSequenceState::RETURNING_CLASSIFIER_TO_INIT;
        stateStartTime = currentTime;
        returnClassifierToInitialPosition();
    } else if (status == osErrorTimeout) {
        // 메시지가 아직 도착하지 않음, 타임아웃 체크
        if (currentTime - stateStartTime > MAX_SENSOR_WAIT_TIME_MS) {
            // 실패 처리: 현재 방향에 대해 에러 값 설정 등
            if (index >= 0 && index < MAX_DIRECTIONS) {
                capacityValues[index] = -2.0f; // 타임아웃 에러 값
                 if (modelListener) {
                    modelListener->notifyCapacityUpdated(currentOperationDirection, capacityValues[index]);
                }
            }
            // 시퀀스를 중단하고 초기화 상태로 돌아갈 수 있음
            returnClassifierToInitialPosition(); // 일단 서보는 원위치
            currentSequenceState = ServoSensorSequenceState::RETURNING_CLASSIFIER_TO_INIT; // 또는 바로 IDLE
            stateStartTime = HAL_GetTick();
            // currentOperationDirection = OperationDirection::NONE; // 시퀀스 실패로 간주
        }
        // else: 아직 대기 시간 남음, 다음 tick에서 다시 확인
    } else {
        // 큐에서 다른 오류 발생
        returnClassifierToInitialPosition();
        currentSequenceState = ServoSensorSequenceState::RETURNING_CLASSIFIER_TO_INIT;
        stateStartTime = HAL_GetTick();
    }
}

float Model::getCapacityForDirection(OperationDirection dir) const
{
    if (dir != OperationDirection::NONE) {
        int index = -1;
        switch (dir) {
            case OperationDirection::UP:    index = 0; break;
            case OperationDirection::RIGHT: index = 1; break;
            case OperationDirection::DOWN:  index = 2; break;
            case OperationDirection::LEFT:  index = 3; break;
            default: break;
        }
        if (index != -1) {
            return capacityValues[index];
        }
    }
    return 0; // 또는 적절한 기본/오류 값
}

void Model::processReturningClassifierToInitState(uint32_t currentTime)
{
    if (currentTime - stateStartTime >= CLASSIFIER_RETURN_TIME) {
        touchgfx_printf("State: RETURNING_CLASSIFIER_TO_INIT -> SEQUENCE_COMPLETE\n");
        currentSequenceState = ServoSensorSequenceState::SEQUENCE_COMPLETE;
    }
}

void Model::processSequenceCompleteState()
{
    touchgfx_printf("State: SEQUENCE_COMPLETE. Notifying Presenter.\n");
    if (modelListener) {
        modelListener->notifyActionSequenceComplete(currentOperationDirection);
    }
    currentSequenceState = ServoSensorSequenceState::IDLE;
    currentOperationDirection = OperationDirection::NONE;
    recordUserActivity();
    touchgfx_printf("Sequence finished. Returning to IDLE.\n");
}

void Model::moveClassifierToPosition1(OperationDirection dir)
{
    touchgfx_printf("Moving classifier for direction: %d\n", (int)dir);

    switch (dir) {
        case OperationDirection::UP:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_ODD_POSITION_PULSE);
            break;
        case OperationDirection::RIGHT:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_EVEN_POSITION_PULSE);
            break;
        case OperationDirection::DOWN:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_ODD_POSITION_PULSE);
            break;
        case OperationDirection::LEFT:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_EVEN_POSITION_PULSE);
            break;
        default:
            break;
    }
}

void Model::moveClassifierToPosition2(OperationDirection dir)
{
    touchgfx_printf("Moving classifier for direction: %d\n", (int)dir);

    switch (dir) {
        case OperationDirection::UP:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_HIGH_POSITION_PULSE);
            break;
        case OperationDirection::RIGHT:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_HIGH_POSITION_PULSE);
            break;
        case OperationDirection::DOWN:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_LOW_POSITION_PULSE);
            break;
        case OperationDirection::LEFT:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_LOW_POSITION_PULSE);
            break;
        default:
            break;
    }
}

void Model::alignSensorServo()
{
    touchgfx_printf("Aligning sensor servo.\n");
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_NEUT_POSITION_PULSE);
}

void Model::returnClassifierToInitialPosition()
{
    touchgfx_printf("Returning classifier to initial position.\n");
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_INIT_POSITION_PULSE);
}

void Model::recordUserActivity()
{
    // 이 함수는 Presenter (버튼 클릭, 화면 터치 시) 또는 SensorTask 등에서 호출됨
    lastActivityTime_m = HAL_GetTick();

    if (isScreenOff_m) {
        // 화면이 꺼져있을 때 활동이 감지되면 화면을 켬
        turnScreenOnInternal();
    }
    // 화면이 이미 켜져 있다면, lastActivityTime_m만 업데이트되어 타임아웃이 연장됨
}

void Model::sensorDetectedProximity()
{
    // 이 함수는 초음파 센서 담당 Task에서 호출됨
    if (isScreenOff_m) {
        turnScreenOnInternal();
    }
    // 이미 화면이 켜져있다면 아무것도 안 하거나, lastActivityTime_m만 갱신할 수도 있음 (선택)
    // else { lastActivityTime_m = HAL_GetTick(); }
}


void Model::checkScreenTimeout()
{
    // 이 함수는 model.tick() 내에서 currentSequenceState == IDLE 일 때만 호출됨
    if (!isScreenOff_m && (HAL_GetTick() - lastActivityTime_m > screenTimeoutMs_m)) {
        turnScreenOffInternal(true);
    }
}

void Model::turnScreenOnInternal()
{
    if (isScreenOff_m) {
        controlBacklight(true);
        isScreenOff_m = false;
        lastActivityTime_m = HAL_GetTick();

        // ProximityTask에 폴링 중지 명령 전송
        ModelToProximityMsg_t txProxCmdMsg;
        txProxCmdMsg.command = PROXIMITY_CMD_STOP_POLLING;
        txProxCmdMsg.delay_ms = 0;

        if (modelListener) {
            modelListener->requestGoToMainScreen(); // Presenter에게 MainScreen 전환 요청
        }
    }
}

void Model::turnScreenOffInternal(bool dueToTimeout) // 타임아웃 여부 파라미터 추가
{
    if (!isScreenOff_m) { // 이미 꺼져있으면 아무것도 안 함
        controlBacklight(false);
        isScreenOff_m = true;

        ModelToProximityMsg_t txProxCmdMsg;
        if (dueToTimeout) {
            txProxCmdMsg.command = PROXIMITY_CMD_START_POLLING_IMMEDIATE;
            txProxCmdMsg.delay_ms = 0;
        } else { // "Off" 버튼 등으로 꺼진 경우
            txProxCmdMsg.command = PROXIMITY_CMD_START_POLLING_DELAYED;
            txProxCmdMsg.delay_ms = 10000; // 10초
        }

        osMessageQueuePut(modelToProximityQueueHandle, &txProxCmdMsg, 0U, 0U);
        if (modelListener) {
            modelListener->requestGoToWaitScreen();
        }
    }
}

void Model::controlBacklight(bool on)
{
    // LCD_BL_CTRL:PG15

    if (on) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);
    }
}

void Model::forceTurnScreenOff()
{
    turnScreenOffInternal(false); // 타임아웃이 아님을 알림 (Off 버튼)
}
