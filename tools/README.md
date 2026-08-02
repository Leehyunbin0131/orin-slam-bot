# 측정 도구

파라미터를 바꿀 때 "유의미한 차이인지"를 판정하는 도구들입니다.
전부 단독 실행 스크립트라 빌드가 필요 없습니다.

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 tools/<도구>.py
```

대부분 스택이 떠 있어야 합니다 (`ros2 launch orinbot_navigation navigation.launch.py`).

| 도구 | 재는 것 | 스택 필요 |
|---|---|---|
| `measure_load.py <라벨>` | 노드별 CPU/메모리. 고정 시나리오(c070 통로 왕복) | 전체 |
| `map_quality.py <라벨>` | SLAM 자세 오차와 지도 흔들림 (c070+c090 왕복) | 전체 |
| `corridor_test.py [폭...]` | 폭별 통로 직진 통과 가능 여부와 소요 시간 | 전체 |
| `turnaround_test.py [폭...]` | 통로 안 제자리 회전 가능 여부 (편심 스윕) | **시뮬만** |
| `slam_accuracy.py` | 지정 동작 후 SLAM vs Gazebo 실제 자세 | 전체 |
| `stream_rates.py [초]` | 카메라 스트림별 실제 주기와 타임스탬프 일치도 | 시뮬 |
| `jump_check.py [초]` | TF 발행자 중복과 map->odom / odom->base 점프 | 전체 |
| `render_map.py` | `/map` 을 PNG 로 저장하고 프론티어 표시 | 전체 |
| `verify_frontier.py` | 현재 지도에서 탐사 목표 후보를 뽑아 봄 | 전체 |

## 반드시 읽을 것 — 측정이 거짓말하는 세 가지 경우

**1. `ps` 의 `%CPU` 를 쓰지 마세요.** 프로세스 수명 전체의 평균이라
시나리오 구간의 실제 부하가 희석됩니다. `measure_load.py` 는
`/proc/<pid>/stat` 차분으로 순간 사용률을 씁니다.

**2. 같은 세션에서 파라미터를 바꿔가며 비교할 때 `rtabmap` 항목은 못 믿습니다.**
지도가 계속 자라 시간이 갈수록 저절로 올라갑니다 (한 세션에서 25 → 38 → 54 %p
관측). RTAB-Map 이 관련된 비교는 **매번 전체를 새로 띄우세요.**

**3. 기준 밖(0.70 m 미만) 통로를 시나리오에 넣지 마세요.** 로봇이 끼여
복구에 쓰는 시간이 표본에 섞이면 "파라미터 효과"와 "그날 얼마나 끼였나"를
구분할 수 없습니다. 그래서 시나리오를 c060 → c070 으로 바꿨습니다.

`turnaround_test.py` 에는 네 번째가 있습니다: **통로 안으로 순간이동시키면
안 됩니다.** Gazebo 는 정적 물체와의 초기 관통을 밀어내지 않아, 벽에 80 mm
박힌 채로 그대로 회전합니다. 반드시 바깥에서 몰고 들어가게 해야 합니다.

## 현재 기준선 (2026-08-02)

| 항목 | 값 |
|---|---|
| CPU / 메모리 | 1.16 코어 / 1.38 GB (Orin 환산 5.2 / 6) |
| SLAM 자세오차 | 중앙값 0.027 m, 90%값 0.043 m |
| map→odom 최대 보정 | 0.111 m |

측정 방법과 시나리오가 바뀌기 전의 값과는 절대값을 비교하지 마세요.
자세한 내역은 `orin-resource-budget` 스킬에 있습니다.
