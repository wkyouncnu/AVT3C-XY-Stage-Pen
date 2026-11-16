# 5_references — 참고 자료

[English](README.md) · [Tiếng Việt](README_VI.md)

## 스테이지 매뉴얼 (Zaber)

| 파일 | 내용 |
|---|---|
| [manuals/X-LSM-E-Manual.pdf](manuals/X-LSM-E-Manual.pdf) | X-LSM-E 시리즈 사용자 매뉴얼 (설치, 배선, 명령, 사양) |
| [manuals/X-LSM-E-Datasheet.pdf](manuals/X-LSM-E-Datasheet.pdf) | X-LSM-E 데이터시트 (행정, 속도, 추력, 정밀도) |

- X·Y 두 축은 같은 계열의 스테이지 두 대이므로 매뉴얼 하나가 두 축에 함께 적용된다.
- 명령 프로토콜 (ASCII): <https://www.zaber.com/protocol-manual?protocol=ASCII>

## 참고 논문

| 번호 | 논문 | 매뉴얼에서 쓰인 곳 |
|---|---|---|
| [1] | T. Zou, J. Angeles, F. Hassani, "Dynamic modeling and trajectory tracking control of unmanned tracked vehicles," *Robotics and Autonomous Systems*, vol. 110, pp. 102–111, 2018. [doi:10.1016/j.robot.2018.09.008](https://doi.org/10.1016/j.robot.2018.09.008) | 궤도 로봇 기구학·동역학, 회전 저항 부호, 외부 루프 (8.2–8.5 절) |
| [2] | A. D. Sabiha, M. A. Kamel, E. Said, W. M. Hussein, "ROS-based trajectory tracking control for autonomous tracked vehicle using optimized backstepping and sliding mode control," *Robotics and Autonomous Systems*, vol. 152, 104058, 2022. [doi:10.1016/j.robot.2022.104058](https://doi.org/10.1016/j.robot.2022.104058) | 기구학·동역학 대조와 원 논문 오타 (8.2–8.3 절), 제어기 비교 (20 장) |

- 두 논문은 출판사 저작권이 있어 PDF 를 공개 저장소에 올리지 않는다. 연구실 내부 사본은 `papers/` 에 두며 `.gitignore` 로 제외한다.
