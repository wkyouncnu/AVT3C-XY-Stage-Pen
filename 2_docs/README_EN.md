# 2_docs — Manual PDFs and figures

[한국어](README.md) · [Tiếng Việt](README_VI.md)

## Contents

| Path | Content |
|---|---|
| `manual/AVT3C_XY_Stage_Pen.pdf` · `_EN.pdf` · `_VI.pdf` | Full manual (Korean · English · Vietnamese). Sources are `README.md` · `README_EN.md` · `README_VI.md` at the repository root |
| `manual/AVT3C_XY_Stage_Pen_Control.pdf` · `_EN` · `_VI` | Pen kinematic compensation excerpt (Chapter 3, Section 4.1, Chapter 6, Section 8.6, Section 14.6) |
| `img/diagram_*.svg` | Diagrams (Korean). `diagram_*_en.svg` is the English version, also used by the Vietnamese edition |
| `img/fig_*.png` | Frame and pen-compensation equation figures |
| `img/src/*.tex` | TikZ sources of the diagrams (`xystyle.tex` shared style) |
| `img/src/make_fig_math.m`, `make_fig_pen.m` | MATLAB scripts that draw the equation figures |
| `make_pen_excerpt.py` | Builds the excerpt Markdown (`--lang en` / `--lang vi`) |

## Rebuilding (from the repository root)

```bash
py 2_docs/make_pen_excerpt.py --lang vi
bash ../../_tools/md2pdf.sh README_VI.md
```

- PDFs are made with headless Chrome (marked.js) and then moved into `manual/`.
- Diagrams: `bash ../../_tools/tikz2svg.sh img/src/diagram_X.tex img/diagram_X.svg` (run in `2_docs`)
