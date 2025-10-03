# 🚀 실내 층간 이동 자율주행 배송로봇
> 엘리베이터 API 없이 **어디서나 동작하는 층간 배송 로봇**  
> ROS2 + 로봇팔 + 비전(OCR/YOLO/Depth) 기반 통합 시스템

---

## 1. 프로젝트 개요

### 1-1. 프로젝트 소개
- **프로젝트 명** : 실내 층간 이동 자율주행 배송로봇  
- **프로젝트 정의** : 2D LiDAR SLAM, 4자유도 매니퓰레이터, Depth Camera + YOLO + OCR 기반으로 엘리베이터 버튼을 직접 누르며 층간 배송을 수행하는 로봇 시스템  
- **한 줄 설명** : API 연동 공사 없이 기존 건물에서도 즉시 도입 가능한 차세대 배송 로봇  

### 1-2. 개발 배경 및 필요성
- 국내 택배 물동량은 연간 **51억 건(+52.9%)**으로 급증  
- 라스트마일 배송은 물류비의 **50% 이상 차지**  
- 기존 층간 배송 로봇은 **특정 엘리베이터 API 의존**, 다중 업체 기술 결합 → **비용↑, 안정성↓, 연동성↓**  
- 따라서 **범용·저비용·고신뢰성** 층간 배송 로봇 필요성 증대

### 1-3. 프로젝트 특장점
- **범용성** : 로봇팔 + 비전 기반 버튼 조작 → 제조사·모델 불문 즉시 활용  
- **비용절감** : API 연동 불필요 → 설치/운영비 절감  
- **신뢰성** : 자율주행·센서퓨전·팔 제어를 단일 시스템으로 통합  
- **확장성** : 병원·호텔·사무실·연구시설 등 다양한 활용 가능  

### 1-4. 주요 기능
- 2D LiDAR + IMU + 엔코더 센서퓨전 기반 자율주행 (SLAM, Nav2 waypoint)  
- 4자유도 매니퓰레이터 역/순기구학 제어 및 QT5 GUI  
- YOLO + OCR + ZED Depth Camera → 엘리베이터 버튼 인식 및 3D 좌표화  
- Web/App 기반 사용자 UI (Node.js + React + roslibjs)  
- ROS2 기반 AMR + Manipulator 통합 제어  

### 1-5. 기대 효과 및 활용 분야
- **기대 효과** : 인건비 절감, 배송 효율 극대화, 오배송·분실 최소화  
- **활용 분야** :  
  - 병원: 긴급 의약품·검체 배송  
  - 호텔: 룸서비스·어메니티 배송  
  - 오피스: 층간 문서/우편 배송  
  - 연구시설/도서관: 샘플 및 장비 운송  

### 1-6. 기술 스택
- **OS/Framework** : Ubuntu 22.04, ROS2 Humble  
- **언어** : C++, Python, JavaScript, HTML/CSS  
- **프론트엔드** : React, QT5 GUI  
- **백엔드** : Node.js, WebSocket (roslibjs)  
- **AI/ML** : YOLOv11, EasyOCR, PyTorch, OpenCV, ZED SDK  
- **하드웨어** : Jetson Orin NX, Raspberry Pi4, RPLiDAR, ZED2i Depth Camera, Dynamixel XM540/XM430, OpenCR1.0  
- **기타** : Docker, GitHub Actions, Fusion360 (기구 설계)

---

## 2. 팀원 소개
| 이름 | 역할 | 담당 |
|------|------|------|
| 이석권 | 팀장 | 전체 PM, 시스템 통합 |
| 정재윤 | 주행 파트 | SLAM, Navigation2, Odometry |
| 금민기| 매니퓰레이터 파트 | 역/순기구학, GUI, 로봇팔 제어 |
| 서채은| 비전 파트 | YOLO, OCR, Depth 좌표 추출 |
| 김도영| 서버/UX 파트 | Node.js, React, rosbridge, Web/App UI |
| 서지훈 멘토 | 기술 자문 | ROS2/로봇 제어 컨설팅 |

---

## 3. 시스템 구성도
### 3-1. S/W 아키텍처
(<img width="1040" height="537" alt="image" src="https://github.com/user-attachments/assets/07d7d6b4-3419-40e0-97b2-314696d8007f" />
)

### 3-2. H/W 구성도
(<img width="797" height="338" alt="image" src="https://github.com/user-attachments/assets/b0e35d72-93a9-4a43-a31b-727bee040943" />
)
<img width="712" height="550" alt="image" src="https://github.com/user-attachments/assets/a1727058-d969-446a-8296-a83875f5e654" />
<img width="811" height="308" alt="image" src="https://github.com/user-attachments/assets/9858b446-1678-4bff-af09-9659ac600aac" />


### 3-3. 서비스 흐름도
(<img width="612" height="666" alt="image" src="https://github.com/user-attachments/assets/3eb922e9-9a3e-46f6-947b-eec0f5a23a67" />
)

---

## 4. 작품 소개영상
https://www.youtube.com/watch?v=7-eT-tHrdZ0  
🔗 프로젝트 시연 영상

---

## 5. 핵심 소스코드
### 5-1. 로봇팔 역기구학 코드 (Python)
```python
print()
```

### 5-2. WebSocket 기반 로봇 제어
```javascript
```

### 5-3. Magwickfilter 이용한 IMU 활용 코드
```c++
class MadgwickFilter {
private:
    float beta;
    float q0, q1, q2, q3;
    float invSampleFreq;
    
    // 역제곱근
    inline float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        y = y * (1.5f - (halfx * y * y));
        return y; 
    }
    
public:
    MadgwickFilter() : beta(0.1f), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}
    
    void begin(float sampleFrequency) {
        invSampleFreq = 1.0f / sampleFrequency;
    }
    
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533f;
        gy *= 0.0174533f;
        gz *= 0.0174533f;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion
        q0 += qDot1 * invSampleFreq;
        q1 += qDot2 * invSampleFreq;
        q2 += qDot3 * invSampleFreq;
        q3 += qDot4 * invSampleFreq;

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
    
    // Get quaternion components
    float getQ0() { return q0; }
    float getQ1() { return q1; }
    float getQ2() { return q2; }
    float getQ3() { return q3; }
};

```

### 5-4. ROS2에서 MDrobot의 듀얼채널 모터드라이버를 제어하기 위한 코드
```c++
    InitSerial();   //communication initialization in com.cpp
    while (rclcpp::ok()) {
        
        ReceiveDataFromController(Motor.InitMotor);
        if(++byCnt2500us == 50)
        {
            byCnt2500us = 0;
            
            if(fgInitsetting == ON)
            {
                switch(++byCntComStep)
                {
                case 1:{ //create tf & update motor position //maybe not needed. change this logic to get odom
                    geometry_msgs::msg::TransformStamped transformStamped;
                    transformStamped.header.stamp = node->now();
                    transformStamped.header.frame_id = "world";
                    transformStamped.child_frame_id = "motor_joint";

                    transformStamped.transform.translation.x = 0.0;
                    transformStamped.transform.translation.y = 0.0;
                    transformStamped.transform.translation.z = 0.15;

                    Motor.current_tick     = Com.position;

                    Motor.last_diff_tick   = Motor.current_tick - Motor.last_tick;
                    Motor.last_tick        = Motor.current_tick;
                    Motor.last_rad        += Motor.Tick2RAD * (double)Motor.last_diff_tick;
                    // printf("%f\n", Motor.Tick2RAD);

                    tf2::Quaternion q;
                    q.setRPY(0, 0, -Motor.last_rad);
                    transformStamped.transform.rotation.x = q.x();
                    transformStamped.transform.rotation.y = q.y();
                    transformStamped.transform.rotation.z = q.z();
                    transformStamped.transform.rotation.w = q.w();

                    tf_broadcaster_.sendTransform(transformStamped);
                    auto end = std::chrono::high_resolution_clock::now();
                    break;
                }
                case 2: //Control motor & request motor info
                    if(++byCntCase[byCntComStep] == TIME_100MS)
                    {
                        byCntCase[byCntComStep] = 0;

                        if(SendCmdRpm)
                        {
                            left_iData = Short2Byte(left_rpm_ * Motor.GearRatio); // #1,2 Wheel RPM
                            right_iData = Short2Byte(right_rpm_ * Motor.GearRatio);

                            nArray[0] = 1; //left motor enable
                            nArray[1] = left_iData.byLow; //left motor rpm (lower 8bits)
                            nArray[2] = left_iData.byHigh; //left motor rpm (higher 8bits)
                            nArray[3] = 1; //right motor enable
                            nArray[4] = right_iData.byLow; //right motor rpm (lower 8bits)
                            nArray[5] = right_iData.byHigh; //right motor rpm (higher 8bits)
                            nArray[6] = 0; //no return data

                            PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor.ID, nArray); //dual channel motor controller -> pid 207 (md ros manual)

                            //n대의 모터드라이버에게 동시에 main data를 요청할 경우 data를 받을 때 데이터가 섞임을 방지.
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);  // Main data request

                            SendCmdRpm = OFF;
                        }
                        else
                        {
                            //n대의 모터드라이버에게 동시에 main data를 요청할 경우 data를 받을 때 데이터가 섞임을 방지.
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor.ID, nArray);  // Main data request
                            //------------------------------------------------------------------
                        }
                        
                    }
                    byCntComStep=0;
                    break;  
                }
            }
```

---


