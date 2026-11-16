# 2_docs — Tài liệu PDF và hình

[한국어](README.md) · [English](README_EN.md)

## Cấu trúc

| Đường dẫn | Nội dung |
|---|---|
| `manual/AVT3C_XY_Stage_Pen.pdf` · `_EN.pdf` · `_VI.pdf` | Tài liệu đầy đủ (tiếng Hàn · tiếng Anh · tiếng Việt). Bản gốc là `README.md` · `README_EN.md` · `README_VI.md` ở thư mục gốc kho mã |
| `manual/AVT3C_XY_Stage_Pen_Control.pdf` · `_EN` · `_VI` | Bản trích bù động học bút (Chương 3, Mục 4.1, Chương 6, Mục 8.6, Mục 14.6) |
| `img/diagram_*.svg` | Sơ đồ (tiếng Hàn). `diagram_*_en.svg` là bản tiếng Anh, bản tiếng Việt cũng dùng bản này |
| `img/fig_*.png` | Hình hệ tọa độ và phương trình bù động học bút |
| `img/src/*.tex` | Mã nguồn TikZ của sơ đồ (kiểu chung `xystyle.tex`) |
| `img/src/make_fig_math.m`, `make_fig_pen.m` | Script MATLAB vẽ các hình phương trình |
| `make_pen_excerpt.py` | Tạo Markdown bản trích (`--lang en` / `--lang vi`) |

## Tạo lại (từ thư mục gốc kho mã)

```bash
py 2_docs/make_pen_excerpt.py --lang vi
bash ../../_tools/md2pdf.sh README_VI.md
```

- PDF được tạo bằng Chrome headless (marked.js), sau đó chuyển vào `manual/`.
- Sơ đồ: `bash ../../_tools/tikz2svg.sh img/src/diagram_X.tex img/diagram_X.svg` (chạy trong `2_docs`)
