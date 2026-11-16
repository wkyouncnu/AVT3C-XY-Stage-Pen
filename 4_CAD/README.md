# 4_CAD — 로봇 형상 (CAD)

[English](README_EN.md) · [Tiếng Việt](README_VI.md)

| 파일 | 내용 | 저장소 |
|---|---|---|
| `AVT-3C.stl` | AVT-3C 로봇 외형, 420 × 343 × 100 mm | 올리지 않음 (설계 기밀) |

- STL 은 연구실 내부 공유 폴더에만 둔다. 이 폴더에 넣으면 `.gitignore` 로 제외되어 GitHub 에 올라가지 않는다.
- 쓰는 곳: `1_simulation/XY_animate.m` (3D 실시간 화면), `XY_setup.m` 의 `stl_file`.
- STL 이 없어도 모든 모델·스크립트는 돈다. 애니메이션의 로봇만 420 × 343 × 120 mm 상자로 그려진다.
- 궤도 접지 길이 `l_trk = 0.30 m` 는 STL 길이 420 mm 에서 추정한 값이다 (`XY_setup.m`).
