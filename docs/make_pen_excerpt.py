"""README.md 에서 펜 제어 부분만 뽑아 PenControl.md 를 만든다 (PDF 는 md2pdf.sh 로).
Extract the pen-control part of README.md into PenControl.md (convert to PDF with md2pdf.sh).

    py docs/make_pen_excerpt.py            # 저장소 맨 위에서 실행 / run from the repository root
    bash ../../_tools/md2pdf.sh PenControl.md
    mv PenControl.pdf docs/manual/AVT3C_XY_Stage_Pen_Control.pdf && rm PenControl.md

장·절·식·그림 번호는 통합본과 같게 둔다 (발췌본에 없는 번호는 통합본을 본다).
Section, equation and figure numbers are kept as in the full manual.
"""
import io, re

src = io.open('README.md', encoding='utf-8').read()

def block(start, stop):
    """start 줄부터 stop 줄 앞까지 / from the start heading up to the stop heading."""
    a = re.search(r'(?m)^' + re.escape(start), src).start()
    b = re.search(r'(?m)^' + re.escape(stop), src[a + 1:]).start() + a + 1
    return src[a:b].rstrip().rstrip('-').rstrip() + '\n\n'

def heading(prefix):
    return re.search(r'(?m)^' + re.escape(prefix) + r'.*$', src).group(0) + '\n\n'

parts = [
    '# AVT-3C 벽면 로봇 + Zaber XY 스테이지: 펜 기구학 보상 (발췌본)\n\n',
    '- 작성: 충남대학교 자율운항시스템공학과 윤원근 교수\n',
    '- 저장소: <https://github.com/wkyouncnu/AVT3C-XY-Stage-Pen>\n',
    '- 통합본(PDF): [docs/manual/AVT3C_XY_Stage_Pen.pdf](docs/manual/AVT3C_XY_Stage_Pen.pdf)\n',
    '- 이 문서는 통합본에서 펜 제어(PenComp)에 관한 부분만 뽑은 것이다. 장·절·식·그림 번호는 통합본과 같으며, 여기에 없는 번호는 통합본에 있다.\n\n',
    '| 실린 부분 | 내용 |\n|---|---|\n',
    '| 3 장 | 벽면·바디·스테이지 좌표계와 회전행렬 |\n',
    '| 4.1 절 | 펜 끝의 위치식 |\n',
    '| 6 장 | 펜 기구학 보상: 기호, 바디·벽면·합친 그림, 1000 과 $s_c$, 수치 예 |\n',
    '| 8.6 절 | 같은 내용의 개조식 유도와 부호 점검 |\n',
    '| 14.6 절 | Simulink PenCtrl 블록 |\n\n---\n\n',
    block('## 3. ', '## 4. '),
    heading('## 4. '), block('### 4.1 ', '### 4.2 '),
    block('## 6. ', '## 7. '),
    heading('## 8. '), block('### 8.6 ', '### 8.7 '),
    heading('## 14. '), block('### 14.6 ', '### 14.7 '),
    '---\n\n', src[re.search(r'(?m)^## 참고문헌', src).start():].split('\n---\n')[0].rstrip() + '\n',
]
io.open('PenControl.md', 'w', encoding='utf-8', newline='').write(''.join(parts))
print('PenControl.md written')
