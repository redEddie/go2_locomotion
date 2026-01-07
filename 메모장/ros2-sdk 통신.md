## ROS2 ↔ Unitree SDK (Cyclone DDS)

### 핵심 요약
- ROS2와 Cyclone DDS는 같은 DDS 계층을 쓰므로 별도 브리지 없이 통신 가능
- 동일한 RMW(Cyclone), 동일 DDS domain, 동일 네트워크 인터페이스가 핵심
- topic/type가 Unitree IDL과 일치해야 함

### 확인된 매핑 (이 레포 기준)
- Unitree SDK DDS 토픽: `rt/lowstate`, `rt/lowcmd` (`example/cpp/stand_go2.cpp`)
- ROS2 토픽: `/lowstate`, `/lowcmd` (`example/ros2/src/stand_go2.cpp`)
- ROS2는 DDS 토픽 이름에 `rt/` prefix를 붙이므로 `/lowstate` ↔ `rt/lowstate`로 매칭
  - 그래서 ROS2에서 `/lowstate` 구독하면 Unitree SDK와 연결됨

### 시뮬레이션 설정 매칭
- 시뮬레이터 DDS domain/interface: `simulate/config.yaml`
  - `domain_id: 1`, `interface: "lo"`
- ROS2도 동일하게 맞춰야 함

### ROS2 환경 설정 (Foxy)
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
```

### 동작 확인
- `unitree_go::msg::LowState`로 `/lowstate` 구독 → 시뮬레이터 LowState 수신
- `unitree_go::msg::LowCmd`를 `/lowcmd`로 publish → 시뮬레이터 제어 가능

### 정리
- ROS2가 Cyclone DDS를 RMW로 쓰면 Unitree SDK DDS와 직통
- 시뮬레이터 실행 시 domain/interface 매칭이 가장 중요
