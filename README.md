# Recyclinator

STM32H735G-DK 보드와 TouchGFX GUI를 활용하여 4종류의 쓰레기를 자동으로 분류하고 통의 잔여 용량을 실시간으로 모니터링하는 스마트 분리수거 쓰레기통 프로젝트

1. 프로젝트 개요

    목적: 초음파 센서로 사용자 접근 감지, 터치 스크린 조작을 통한 쓰레기 자동 분류 및 용량 표시

    기능: 
  
    -GUI 버튼과 서보모터를 이용한 4방향 분류
  
    -초음파 센서를 이용한 적재량 측정
  
    -사용자 감지를 통한 자동 화면 On/Off
  
<br>

2. 기술 스택
  
    MCU: STM32H735IGK6 (Cortex-M7, 550MHz)

   GUI: TouchGFX (MVP 패턴 기반)

   OS: FreeRTOS

   Peripherals:

    -TIM1, TIM4: 서보 모터 PWM 제어

   -TIM23: 초음파 센서 에코 측정(Input Capture)

   -OCTOSPI: 외부 Flash, HyperRam 연동

   -DWT: 마이크로초 단위 정밀 딜레이

<br>
    
3. 시스템 구조

   i.FreeRTOS의 멀티태스킹 구조
   
   -TouchGFXTask: UI 렌더링 및 사용자 입력 처리
   
   -VideoTask: LCD 영상 재생 및 그래픽 처리
   
   -TouchGFX의 model의 요청에 따라 특정 초음파(US0 또는 US1) 센서로 용량 측정
   
   -ProximityTask: 사용자 감지 센서(US2)를 폴링하여 사람이 접근하면 화면 활성화

   <br>

   ii. C/C++ 브릿지 (shared_sensor_types.h)
   
   -C 기반의 main.c Task와 C++ 기반의 TouchGFX Model 사이의 데이터 통신을 위해 공통 구조체와 
   열거형을 정의하여 RTOS 메시지 큐를 통해 상호작용

   <br>
   
4. 핵심 로직

   쓰레기 분류 및 측정 시퀀스는 Model.cpp에서 상태 머신으로 관리 

   -IDLE: 사용자 입력 대기 및 화면 타임아웃(30초) 확인

   -MOVING_CLASSIFIER: 서보 모터를 구동하여 분류기를 해당 방향으로 회전 및 기울임

   -ALIGNING_SENSOR: 측정을 위해 분류기를 수평으로 정렬

   -SENDING_MEASUREMENT_REQUEST: SensorTask에 적재량 측정 메지 전송

   -WAITING_FOR_SENSOR_RESULT: 초음파 측정값 수신 및 백분율로 매핑하여 잔여 용량 표시

   -RETURNING_TO_INIT: 모든 서보를 초기 위치로 복귀

  <br>

5. 하드웨어 연결

   | 기능 | 핀 번호 | 설명 |
   | --- | --- | --- |
   | SM0 (Servo) |PE14 (TIM1_CH4) | 분류 방향 조정 서보 |
   | SM1 (Servo) | PD15 (TIM4_CH4) | 분류기 기울기 제어 서보 |
   | US0 (Ultrasonic) | PG5 (Trig) / PF9 (Echo)	| 상단 적재량 측정 |
   | US1 (Ultrasonic) | PE3 (Trig) / PF8 (Echo) | 하단 적재량 측정 |
   | US2 (Ultrasonic) | PG4 (Trig) / PF6 (Echo) | 사람 접근 감지 |
   | LCD_BL_CTRL | PG15 | LCD 백라이트 제어 |

