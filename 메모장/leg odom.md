# Leg odometry (Go2)
## 입력/출력
- 입력: /lowstate (joint q/dq, IMU gyro, foot_force)
- 출력: /go2/leg_odom/twist, /go2/leg_odom/odom
- 내부: FK + Jacobian(수치 미분), stance 평균, moving window filter

## 구현 위치
- `example/go2_locomotion/src/leg_odometry_ros2/src/leg_odometry_node.cpp`
  - `/lowstate` 구독 → foot force threshold로 stance 판정
  - Go2 FK + Jacobian(수치 미분)으로 leg velocity 계산
  - stance 평균 후 moving window filter 적용
  - `/go2/leg_odom/twist`, `/go2/leg_odom/odom` publish
- `example/go2_locomotion/src/leg_odometry_ros2/src/leg_kinematics.cpp`
- `example/go2_locomotion/src/leg_odometry_ros2/include/leg_odometry_ros2/leg_kinematics.hpp`
  - Go2 링크 길이/오프셋 기반 FK + Jacobian
- `example/go2_locomotion/src/leg_odometry_ros2/include/leg_odometry_ros2/moving_window_filter.hpp`
  - moving window filter
- `example/go2_locomotion/src/leg_odometry_ros2/config/leg_odometry.yaml`
  - 파라미터 기본값(현재 contact_force_threshold = -1로 all-stance)
- `example/go2_locomotion/src/leg_odometry_ros2/CMakeLists.txt`
- `example/go2_locomotion/src/leg_odometry_ros2/package.xml`
  - unitree_go, Eigen3 의존성 추가

## 참고
- `lowstate_topic` 기본값은 `/lowstate`.
- unitree mujoco가 pub하는 토픽 중 /sportmodestate가 /footforce를 제공하지 않는 관계로 [contact_force_threshold_](https://github.com/redEddie/go2_locomotion/blob/main/src/leg_odometry_ros2/src/leg_odometry_node.cpp#L203)를 0으로 설정해야 동작함.

## 빌드/실행
```bash
colcon build --base-paths /home/user/unitree_mujoco/example/go2_locomotion/src
source install/setup.bash
ros2 run leg_odometry_ros2 leg_odometry_node --ros-args --params-file /home/user/unitree_mujoco/example/go2_locomotion/src/leg_odometry_ros2/config/leg_odometry.yaml
```

## 다음 작업 (TODO)
- [x] `/lowstate` vs `/rt/lowstate` 실제 토픽 확인 후 파라미터 고정
- [ ] Jacobian 해석식으로 교체해 정확도/속도 개선
- [ ] foot_force 단위 확인 후 주석 구체화 및 threshold 튜닝
- [ ] foot force 값이 0이므로 Ground Reaction Force 추정 기술 구현 필요
