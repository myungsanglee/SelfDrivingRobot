# ADAS와 딥러닝을 이용한 자율 주행 시스템
<div>
<img src="https://user-images.githubusercontent.com/55565351/87013550-c8efb880-c205-11ea-9dcc-cbfb0bbcc6af.jpg" width="300" height="300"/>
<img src="https://user-images.githubusercontent.com/55565351/87013555-cab97c00-c205-11ea-99a6-d14d5a8bb0f2.jpg" width="300" height="300"/>
<img src="https://user-images.githubusercontent.com/55565351/87013566-ce4d0300-c205-11ea-9e0e-f909223a6efc.jpg" width="300" height="300"/>
<img src="https://user-images.githubusercontent.com/55565351/87013575-d016c680-c205-11ea-959f-a0c94a8997f4.jpg" width="300" height="300"/>
</div>

## 프로젝트 개요
* OpenCV를 이용한 실시간 Lane Detection
* TensorFlow Lite와 Google Coral을 이용한 실시간 Object Detection 
* On-Device AI

## 개발 환경
제목 | 내용
--------- | --------
OS | Ubuntu 18.04, ROS Melodic
언어 | C++, Python
하드웨어 | Pi 4, Jetson Nano, Google Coral USB Accerelator, Turtlebot3
카메라 | Pi Camera V2 + IMX219-D160 (Lane Detection), Intel RealSense D435i (Object Detection)
라이브러리 | OpenCV 3.4.0
프레임워크 | TensorFlow 1.15, TensorFlow Lite
객체 인식 모델 | SSD MobileNet v2 Quantized

## 주요 기능
* 차선 유지 보조 시스템
  + OpenCV를 이용하여 차선을 인식하고 센터값을 구해서 방향을 정한후, 정확한 제어를 위해 PD제어를 이용하여 구동체를 제어한다.
* 자율 주차 시스템
  + 레이저 센서를 이용하여 좌면과 후면의 거리를 감지후, 각각의 거리를 구하여 자율 주차를 한다.
* 전방 충돌방지 보조 시스템
  + 자동차 및 사람을 객체 인식하고 객체와의 거리를 측정하여 충돌하지 않도록 차량을 제어한다.
* 도로 상황 보조 시스템
  + 신호등 및 표지판을 객체 인식하고 현재 도로 상황을 판단하여 차량을 제어한다.
 
 ## 프로젝트 결과
 [![img](http://img.youtube.com/vi/K70YHFUSsn0/0.jpg)](https://youtu.be/K70YHFUSsn0?t=0s "img")
