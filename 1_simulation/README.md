# 1_simulation — Simulink 모델과 스크립트

[English](README_EN.md) · [Tiếng Việt](README_VI.md) · 전체 매뉴얼: [../README.md](../README.md) (13–17 장)

## 파일

| 파일 | 내용 |
|---|---|
| `XY_setup.m` | 모든 파라미터. **가장 먼저 실행** |
| `build_xy_models.m` | 모델 생성기. 아래 `.slx` 는 이 파일로만 만든다(손으로 고치지 않음) |
| `XY_1_pen_sim.slx` | 가상 실험: 제어기 + 로봇·스테이지 운동모델 + 센서, ROS 2 루프백 (`ROS_DOMAIN_ID` 77) |
| `XY_2_pen_ros2.slx` | 실기: 제어기만. 실제 로봇·스테이지와 같은 토픽으로 연결 (`ROS_DOMAIN_ID` 0) |
| `XY_1_pen_sim_v2.slx`, `XY_2_pen_ros2_v2.slx` | 2 판: 실기 모델의 내부 PI(속도·요 각속도) 포함 (매뉴얼 21.2 절) |
| `XY_1_pen_sim_run.m` | 가상 실험 한 번 실행 |
| `XY_run_all.m` | 설정 → 빌드 → 실행 → 그림 → 동영상, `logs/status.txt` 에 단계별 결과 |
| `XY_compare.m`, `XY_compare_plot.m` | 설계 선택 비교 10 경우 (매뉴얼 20 장) |
| `XY_animate.m`, `XY_stop.m` | Run 버튼 실시간 화면, 종료 시 결과 그림 |
| `XY_record_live.m` | 대시보드 동영상 (화면 밖 렌더링) |
| `XY_plot.m` | 결과 그림 (`img/XY_1_result_*.png`) |
| `img/`, `video/` | 모델 그림, 결과 그래프, 동영상 |

## 실행

```matlab
XY_setup                      % 파라미터
build_xy_models               % 1 판 모델 두 개 (2 판: build_xy_models([1 2], 2))
open_system('XY_1_pen_sim')   % Run 버튼 → 실시간 화면, 끝나면 결과 그림
```

- 필요: MATLAB/Simulink R2024b, ROS Toolbox, Stateflow
- 기준 결과(30 mm/s): 펜 횡오차 RMS 약 1.2 mm, 보상 없음 약 14–15 mm
- `../4_CAD/AVT-3C.stl` 이 없으면 애니메이션의 로봇은 420 × 343 × 120 mm 상자로 그려진다
- 모든 `.m` 과 모델 안 MATLAB Function 블록의 주석은 한국어·영어 병기
