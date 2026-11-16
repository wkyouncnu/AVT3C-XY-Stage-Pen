# 2_docs — 매뉴얼 PDF 와 그림

[English](README.md) · [Tiếng Việt](README_VI.md)

## 구성

| 경로 | 내용 |
|---|---|
| `manual/AVT3C_XY_Stage_Pen.pdf` · `_EN.pdf` · `_VI.pdf` | 전체 매뉴얼 (한국어 · 영어 · 베트남어). 원본은 저장소 맨 위 `README_KO.md` · `README.md` · `README_VI.md` |
| `manual/AVT3C_XY_Stage_Pen_Control.pdf` · `_EN` · `_VI` | 펜 기구학 보상 발췌본 (3 장, 4.1 절, 6 장, 8.6 절, 14.6 절) |
| `img/diagram_*.svg` | 다이어그램 (한국어). `diagram_*_en.svg` 는 영어판, 베트남어판도 영어판을 씀 |
| `img/fig_*.png` | 좌표계·펜 보상 수식 그림 |
| `img/src/*.tex` | 다이어그램 TikZ 원본 (`xystyle.tex` 공통 스타일) |
| `img/src/make_fig_math.m`, `make_fig_pen.m` | 수식 그림 생성 MATLAB 스크립트 |
| `make_pen_excerpt.py` | 발췌본 마크다운 생성 (`--lang en` / `--lang vi`) |

## 다시 만들기 (저장소 맨 위에서)

```bash
py 2_docs/make_pen_excerpt.py --lang vi
bash ../../_tools/md2pdf.sh README_VI.md
```

- PDF 는 Chrome 헤드리스(marked.js)로 만든다. 만든 뒤 `manual/` 로 옮긴다.
- 다이어그램: `bash ../../_tools/tikz2svg.sh img/src/diagram_X.tex img/diagram_X.svg` (`2_docs` 에서 실행)
