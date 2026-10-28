#!/usr/bin/env python3
"""Generate two actual-size A4 sheets for the classroom's four ArUco markers."""
from pathlib import Path

import cv2
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'output/pdf/aruco_0-3_A4_80-100mm.pdf'
FONT_ROOT = Path('/usr/share/fonts/truetype/nanum')
QUIET_MM = 10


def crop_marks(document, left, bottom, size):
    """Keep every cutting guide outside the marker's blank 10 mm quiet zone."""
    document.saveState()
    document.setStrokeColorCMYK(0, 0, 0, .4)
    document.setLineWidth(.2)
    gap, length = .8 * mm, 3 * mm
    for x, direction_x in ((left, -1), (left + size, 1)):
        for y, direction_y in ((bottom, -1), (bottom + size, 1)):
            document.line(x + direction_x * gap, y, x + direction_x * length, y)
            document.line(x, y + direction_y * gap, x, y + direction_y * length)
    document.restoreState()


def marker(document, dictionary, identifier, size_mm, center_y_mm):
    size = size_mm * mm
    x, y = (A4[0] - size) / 2, center_y_mm * mm - size / 2
    # Four data bits + one black border bit on each side = a 6 x 6 grid.
    bits = cv2.aruco.drawMarker(dictionary, identifier, 6, borderBits=1)
    cell = size / 6
    document.setFillColorCMYK(0, 0, 0, 1)
    document.rect(x, y, size, size, stroke=0, fill=1)
    document.setFillColorCMYK(0, 0, 0, 0)
    for row in range(6):
        for column in range(6):
            if bits[row, column] == 255:
                document.rect(x + column * cell, y + (5 - row) * cell,
                              cell, cell, stroke=0, fill=1)
    crop_marks(document, x - QUIET_MM * mm, y - QUIET_MM * mm,
               size + 2 * QUIET_MM * mm)


def main():
    pdfmetrics.registerFont(TTFont('Nanum', str(FONT_ROOT / 'NanumGothic.ttf')))
    pdfmetrics.registerFont(TTFont('Nanum-Bold', str(FONT_ROOT / 'NanumGothicBold.ttf')))
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    document = canvas.Canvas(str(OUTPUT), pagesize=A4, pageCompression=1)
    document.setTitle('Classroom ArUco 0-3 | A4 | 80 mm and 100 mm')
    document.setAuthor('SMART VISION AMR')
    document.setSubject('DICT_4X4_50, black outer square: IDs 0/1 80 mm; IDs 2/3 100 mm')
    document.setViewerPreference('PrintScaling', 'None')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    sheets = (((0, '홈', 80), (1, 'Sector 1', 80)),
              ((2, 'Sector 2', 100), (3, 'Sector 3', 100)))
    for page, markers in enumerate(sheets, 1):
        document.setFillColorCMYK(0, 0, 0, 1)
        document.setFont('Nanum-Bold', 12)
        document.drawCentredString(A4[0] / 2, 286 * mm, 'ArUco 마커 출력용 - A4')
        document.setFont('Nanum', 8)
        document.drawCentredString(A4[0] / 2, 279 * mm,
                                  'DICT_4X4_50  |  실제 크기 100%  |  용지에 맞춤 / 축소 해제')
        for (identifier, name, size_mm), center_y, label_y in zip(markers, (206, 71), (271, 136)):
            document.setFillColorCMYK(0, 0, 0, 1)
            document.setFont('Nanum-Bold', 10)
            document.drawCentredString(A4[0] / 2, label_y * mm,
                                      f'ID {identifier}  /  {name}  /  {size_mm} x {size_mm} mm')
            marker(document, dictionary, identifier, size_mm, center_y)
        document.setFillColorCMYK(0, 0, 0, .75)
        document.setFont('Nanum', 7)
        document.drawCentredString(A4[0] / 2, 5.5 * mm,
                                  f'검정 외곽이 지정 크기입니다. 흰 여백 10 mm 유지. 인쇄 후 자로 확인하세요.    {page} / 2')
        document.showPage()
    document.save()
    print(OUTPUT)


if __name__ == '__main__':
    main()
