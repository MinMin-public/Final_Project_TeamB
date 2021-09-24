# Programmers Autonomous-Driving Dev course. Final_Project_TeamB
[K-Digital-Training] 자율주행 최종 경진대회

## Video
---


## Goal
---
1. 직선구간
  - 출발
    - 신호등 인식하여 파란불에 출발
  - 차선 변경
    - 2차선 출발, 1차선에서 주행중인 차량을 피해 차선 변경 후 트랙 주행
2. 교차로 구간
  - 정지선
    - 교차로 진입시 정지선 인식 및 저이
  - 신호등
    - 신호인식 후 주행
  - T주차
    - AR태그 인식 후 T주차 진행 (전면, 후면 가능)
  - 타겟정차
    - 미션 수행을 위해 사람, 고양이 순으로 각 이미지 앞 5초간 정차
    - 5초 이상 정차하지 않을 경우 정차로 인정 불가
3. 곡선 구간
  - 차선인식
    - 차선을 인식하여 곡선 주행
    - 블럭을 넘어뜨리지 않고 주행
4. 경사로 구간
  - 진입
    - 경사로 내 주행
  - 정지선
    - 경사로 퇴출 후 정지선 인식 및 정지
5. 로터리 구간
  - 회전 주행
    - 회전중인 차량 피해 진입 후 충돌하지 않고 주행
6. 장애물 구간
  - 갓길 정차된 장애물 차량 피해 좁은길 주행 (차선 이탈 임시 허용)
7. 평행주차 구간
  - AR태그 인식 후 비어있는 공간을 찾아 평행 주차

## Environment
---
- Ubuntu 18.04
- ROS Melodic
- Xycar Model D
- Nvidia TX 2

## Structure
---
~~~
final_project
  └─ calibration
  │    └─ ost.txt
  │    └─ ost.yaml
  └─ config
  │    └─ mapping.lua
  │    └─ xycar_localization_v1.lua
  └─ launch
  │    └─ TEAM_BOLD.launch           
  └─ maps
  │    └─ xycar_map_9_17_final.pbstream
  └─ path
  │    └─ reference_path_final1.pkl
  │    └─ reference_path_final2.pkl
  │    └─ reference_path_final3.pkl
  │    └─ draw_path.py
  │    └─ map_drawer.py
  │    └─ rosbag_to_map.py
  └─ rviz
  │    └─ localization.rviz
  │    └─ offline_mapping.rviz
  │    └─ offline_mapping_youngjin.rviz
  └─ src
  │    └─ Control
  │    │    └─ T_Parking.py
  │    │    └─ motor.py
  │    │    └─ parallel.py
  │    │    └─ pid.py
  │    │    └─ stanley.py
  │    │    └─ stanley_follower.py
  │    └─ Perception
  │    │    └─ avoid_obstacle.py
  │    │    └─ interrupt.py
  │    │    └─ rotary_lidar.py
  │    │    └─ stopline.py
  │    │    └─ traffic_sign.py
  │    │    └─ yolo.py
  │    └─ TEAM_BOLD.py
  │    └─ xycar.py
  └─ urdf
  │    └─ xycar.urdf          
~~~

## Usage
---
~~~bash
$ roslaunch final_project TEAM_BOLD.launch
~~~

## Procedure & Try
---
### mapping
![image (2)](https://user-images.githubusercontent.com/65532515/134633108-9ed5957a-f9e4-4f48-b7af-3e2ac3cf81aa.png)
- mapping 전용 lua 파일을 이용하여 주행할 맵을 매핑한다.
### localization
![image](https://user-images.githubusercontent.com/65532515/134635003-8f8fad1a-d4a3-4ae6-8a53-b621d457782a.png)
- 위에서 제작한 맵을 토대로 localization을 진행한다. 
### path planning & control
![image](https://user-images.githubusercontent.com/65532515/134119319-62f924a7-be56-4271-8923-5a333136f601.png)
- 맵의 x, y 좌표와 차의 현재 위치(x, y)좌표를 비교하여 heading error와 cross track error(cte) 를 구하고, steering angle 값을 도출하여 reference path대로 따라갈 수 있도록 path planning 진행.
### Stopline
![image](https://user-images.githubusercontent.com/65532515/134637921-ea0ce2f4-2841-41a2-8f3a-794d42f41662.png)
- localization을 통한 차량의 현재위치를 구해 reference path 상의 정지선 위치에서 정차하도록 구현.
### Traffic Light
![image](https://user-images.githubusercontent.com/65532515/134639892-8dbe0e81-a148-4a3f-915a-42c75a65f61e.png)
- localization을 통한 차량의 현재위치를 구해 reference path 상의 신호등 위치에서 정차한 후 hsv 값을 이용하여 초록불을 인지 후 출발.
### T_parking
![image](https://user-images.githubusercontent.com/65532515/134636638-3afb65bd-5c98-4a9f-81e9-0ddcf431cffb.png)
- AR 태그를 인식해 차량의 yaw값을 구해 그 값에 10.0 만큼 곱한 값을 angle값을 사용하여 주차공간에 주차.
### Target Stop
![image](https://user-images.githubusercontent.com/65532515/134637519-e8cdd67f-f211-4e86-acc0-1eb0a69b6a18.png)
- localization을 통한 차량의 현재위치를 구해 reference path 상의 타겟위치에서 5초간 정차하도록 구현.
### Slope
![image](https://user-images.githubusercontent.com/65532515/134640162-a825e691-13e7-439e-8e2b-34637f598846.png)
- imu 데이터를 이용해 오르막길로 인식되면 모터 출력을 강하게 하여 올라간 후 내리막길로 인식되면 모터 출력을 0으로 하여 정차를 원할히 할 수 있도록 함.
### Rotary
![image](https://user-images.githubusercontent.com/65532515/134640778-f2e804e9-a716-4f79-8413-c2bdbe77de4c.png)
- 라이다의 오른쪽 45° 값을 받아 1.5이하이면 차가 지나간 것으로 인식하여 출발


## Limitations
---
- 바닥에 비친 형광등, 기둥을 차선으로 인식하여 차선을 벗어나는 경우가 있었다. 
  - 카메라 노출도 조정과 한쪽 차선만 검출하여 해결
- PID 제어를 사용할 때 직선 구간에서 똑바로 가지 못함 
  - PID값을 조절하여 P = 0.25, I = 0.0005, D = 0.25 으로 설정했을 때 가장 안정적이었음.
- 하지만 위 PID 값을 적용했을 때, 곡선에서 차선 이탈을 하는 문제가 있었음 
  - 곡선과 직선에서의 PID값을 따로 주어 해결 -> 곡선 P = 0.5, I = 0.0, D = 0.25 로 설정. 

## What I've learned
---
- Image Processing의 다양한 Noise 해결 방법
- hough transform을 사용한 lane detection
- 상황에 맞게 PID와 MovingAverageFilter를 조절해서 부드러운 조향각 제어
