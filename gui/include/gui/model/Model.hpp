#ifndef MODEL_HPP
#define MODEL_HPP

#include <touchgfx/Utils.hpp>
#include <stdint.h>
#include <gui/model/ModelListener.hpp>
#include "main.h"
#include <gui/model/ModelCommon.hpp>

/**
 * The Model class defines the data model in the model-view-presenter paradigm.
 * The Model is a singular object used across all presenters. The currently active
 * presenter will have a pointer to the Model through deriving from ModelListener.
 *
 * The Model will typically contain UI state information that must be kept alive
 * through screen transitions. It also usually provides the interface to the rest
 * of the system (the backend). As such, the Model can receive events and data from
 * the backend and inform the current presenter of such events through the modelListener
 * pointer, which is automatically configured to point to the current presenter.
 * Conversely, the current presenter can trigger events in the backend through the Model.
 */

enum class ServoSensorSequenceState
{
    IDLE,
    START_SEQUENCE,
    MOVING_CLASSIFIER1,
    MOVING_CLASSIFIER2,
    ALIGNING_SENSOR,
    SENDING_MEASUREMENT_REQUEST,
    WAITING_FOR_SENSOR_RESULT,
    RETURNING_CLASSIFIER_TO_INIT,
    SEQUENCE_COMPLETE
};


class Model
{
public:
    Model();
    void startServoSensorSequence(OperationDirection dir);

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
    float getCapacityForDirection(OperationDirection dir) const;

    // --- 화면 관리 인터페이스 함수 ---
    void recordUserActivity();  // 사용자 활동(터치, 버튼) 시 호출
    void forceTurnScreenOff();  // "Off" 버튼 등으로 강제 화면 끄기
    bool isScreenCurrentlyOff() const { return isScreenOff_m; }
    void sensorDetectedProximity(); // 센서가 접근 감지 시 호출
    void turnScreenOnInternal();
    void turnScreenOffInternal(bool dueToTimeout);

protected:
    ModelListener* modelListener;

private:
    ServoSensorSequenceState currentSequenceState;
    OperationDirection currentOperationDirection;
    uint32_t stateStartTime; // 현재 시퀀스 상태 시작 시간

    static const uint32_t CLASSIFIER_MOVE_TIME = 500;
    static const uint32_t SENSOR_ALIGN_TIME = 3000;
    // static const uint32_t CAPACITY_MEASURE_TIME = 200; // 사용되지 않는 것으로 보임
    static const uint32_t CLASSIFIER_RETURN_TIME = 500;
    static const uint32_t MAX_SENSOR_WAIT_TIME_MS = 5000;

    float capacityValues[MAX_DIRECTIONS];

    // --- 화면 관리용 멤버 변수 ---
    bool isScreenOff_m;
    uint32_t lastActivityTime_m;
    uint32_t screenTimeoutMs_m;

    // --- 화면 관리용 내부 함수 ---
    void checkScreenTimeout();
    void controlBacklight(bool on); // 실제 백라이트 제어

    // 내부 시퀀스 헬퍼 함수
    void processIdleState();
    void processStartSequenceState();
    void processMovingClassifierState1(uint32_t currentTime);
    void processMovingClassifierState2(uint32_t currentTime);
    void processAligningSensorState(uint32_t currentTime);
    void processSendingMeasurementRequestState();
    void processWaitingForSensorResultState(uint32_t currentTime);
    void processReturningClassifierToInitState(uint32_t currentTime);
    void processSequenceCompleteState();

    void moveClassifierToPosition1(OperationDirection dir);
    void moveClassifierToPosition2(OperationDirection dir);
    void alignSensorServo();
    void returnClassifierToInitialPosition();
    void processProximityEvents();
};

#endif /* MODEL_HPP */
