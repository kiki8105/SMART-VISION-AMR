#!/usr/bin/env python3
"""Render a one-slide, presentation-ready comparison diagram."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


OUTPUT = Path(__file__).resolve().parent
REGULAR = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
BOLD = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")

BG = "#F4F7FB"
NAVY = "#172B44"
BODY = "#38506B"
MUTED = "#62748A"
BASE = "#63748B"
BASE_LIGHT = "#EEF2F6"
BASE_BORDER = "#DAE2E9"
TEAM = "#067D83"
TEAM_LIGHT = "#E8F7F5"
TEAM_BORDER = "#B7E4DF"


def rounded(ax, x, y, w, h, face, edge="none", radius=20, lw=1.2):
    ax.add_patch(
        FancyBboxPatch(
            (x, y), w, h,
            boxstyle=f"round,pad=0,rounding_size={radius}",
            linewidth=lw, edgecolor=edge, facecolor=face,
        )
    )


def txt(ax, x, y, value, size=18, color=BODY, bold=False, ha="left", va="center"):
    ax.text(
        x, y, value, ha=ha, va=va, color=color,
        fontsize=size, fontproperties=BOLD if bold else REGULAR,
        linespacing=1.35,
    )


def arrow(ax, start, end, color, lw=2.5, scale=20):
    ax.add_patch(
        FancyArrowPatch(
            start, end, arrowstyle="-|>", mutation_scale=scale,
            linewidth=lw, color=color, shrinkA=0, shrinkB=0,
        )
    )


def flow_box(ax, x, y, w, h, value, team=False, size=17):
    rounded(
        ax, x, y, w, h,
        TEAM_LIGHT if team else BASE_LIGHT,
        TEAM_BORDER if team else BASE_BORDER,
        radius=15,
    )
    txt(ax, x + w / 2, y + h / 2, value, size, TEAM if team else NAVY, True, "center")


def render():
    plt.rcParams["svg.fonttype"] = "path"  # keep Korean text portable in PowerPoint
    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1920)
    ax.set_ylim(0, 1080)
    ax.axis("off")

    rounded(ax, 76, 1008, 12, 48, TEAM, radius=6)
    txt(ax, 111, 1032, "A*에서 실시간 배송 로봇까지", 35, NAVY, True)
    txt(ax, 112, 973, "경로 계획 방식과 TurtleBot3 운용 구조를 한눈에 비교", 18, MUTED)

    txt(ax, 80, 918, "01  경로 계획", 21, NAVY, True)
    rounded(ax, 80, 528, 850, 356, "#FFFFFF", BASE_BORDER, radius=25)
    rounded(ax, 990, 528, 850, 356, "#FFFFFF", TEAM_BORDER, radius=25)
    rounded(ax, 112, 811, 47, 47, BASE_LIGHT, radius=13)
    rounded(ax, 1022, 811, 47, 47, TEAM_LIGHT, radius=13)
    txt(ax, 135, 835, "A", 19, BASE, True, "center")
    txt(ax, 1045, 835, "B", 19, TEAM, True, "center")
    txt(ax, 176, 835, "A* 단독 계획", 22, NAVY, True)
    txt(ax, 1086, 835, "우리 팀: A* + 실시간 대응", 22, NAVY, True)

    for x, label in [(120, "정적 지도\n+ 시작·목표"), (375, "A* 격자 탐색"), (630, "경로 산출")]:
        flow_box(ax, x, 700, 225, 77, label)
    arrow(ax, (352, 738), (365, 738), BASE, 2.7, 20)
    arrow(ax, (607, 738), (620, 738), BASE, 2.7, 20)
    rounded(ax, 120, 568, 770, 96, BASE_LIGHT, BASE_BORDER, radius=17)
    txt(ax, 505, 625, "경로 계산이 핵심", 19, NAVY, True, "center")
    txt(ax, 505, 590, "환경 변화 대응·속도 제어는 별도 로직이 필요", 16, BODY, False, "center")

    for x, label in [(1030, "지도·목표"), (1285, "A* 경로"), (1540, "Nav2 추종")]:
        flow_box(ax, x, 700, 225, 77, label, team=True)
    arrow(ax, (1262, 738), (1275, 738), TEAM, 2.7, 20)
    arrow(ax, (1517, 738), (1530, 738), TEAM, 2.7, 20)
    rounded(ax, 1030, 565, 770, 103, TEAM_LIGHT, TEAM_BORDER, radius=17)
    txt(ax, 1415, 634, "LiDAR 새 장애물 → 국소 우회 후 경로 합류", 17, TEAM, True, "center")
    txt(ax, 1415, 592, "장애물 기억 · 우회 불가 시 전체 재계획/안전 후진", 15, BODY, False, "center")
    arrow(ax, (1398, 669), (1398, 694), TEAM, 2.4, 18)

    txt(ax, 80, 478, "02  시스템 구성", 21, NAVY, True)
    rounded(ax, 80, 87, 850, 359, "#FFFFFF", BASE_BORDER, radius=25)
    rounded(ax, 990, 87, 850, 359, "#FFFFFF", TEAM_BORDER, radius=25)
    rounded(ax, 112, 374, 47, 47, BASE_LIGHT, radius=13)
    rounded(ax, 1022, 374, 47, 47, TEAM_LIGHT, radius=13)
    txt(ax, 135, 398, "A", 19, BASE, True, "center")
    txt(ax, 1045, 398, "B", 19, TEAM, True, "center")
    txt(ax, 176, 398, "TurtleBot3 기본 운용 예시", 22, NAVY, True)
    txt(ax, 1086, 398, "우리 팀 통합 배송 시스템", 22, NAVY, True)

    for y, label in [
        (315, "운용자   │   RViz에서 위치·목표 지정"),
        (227, "PC   │   저장 지도 · AMCL · Nav2"),
        (139, "로봇   │   LiDAR · 바퀴/IMU · 모터"),
    ]:
        flow_box(ax, 120, y, 770, 61, label)
    arrow(ax, (505, 313), (505, 291), BASE, 2.2, 16)
    arrow(ax, (505, 225), (505, 203), BASE, 2.2, 16)
    txt(ax, 505, 112, "Nav2의 장애물 회피는 설정에 따라 사용 가능", 13, MUTED, False, "center")

    for y, label in [
        (315, "웹   │   재고 요청 · 관제 · 조이스틱"),
        (227, "PC   │   AMCL · A* / 장애물 기억 · Nav2"),
        (139, "로봇 Pi   │   LiDAR · 바퀴/IMU EKF · 후방캠 · 안전 mux"),
    ]:
        flow_box(ax, 1030, y, 770, 61, label, team=True, size=16 if y == 139 else 17)
    arrow(ax, (1415, 313), (1415, 291), TEAM, 2.2, 16)
    arrow(ax, (1415, 225), (1415, 203), TEAM, 2.2, 16)
    txt(ax, 1415, 112, "ArUco 섹터 확인 → 다음 배송 → Home 복귀", 13, TEAM, False, "center")

    txt(ax, 80, 41, "비교 기준: A* 단독 경로 탐색 / ROBOTIS의 TurtleBot3 Nav2 실습 예시 / 팀 저장소의 실차 구성", 12, MUTED)

    png = OUTPUT / "navigation_system_comparison.png"
    svg = OUTPUT / "navigation_system_comparison.svg"
    fig.savefig(png, dpi=120, facecolor=BG, pad_inches=0)
    fig.savefig(svg, format="svg", facecolor=BG, pad_inches=0)
    plt.close(fig)
    # Matplotlib puts trailing spaces on multiline SVG path data; keep the
    # generated artifact clean for repository whitespace checks.
    svg.write_text("\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n")
    print(png)
    print(svg)


if __name__ == "__main__":
    render()
