#!/usr/bin/env python3
"""Create a theme-neutral, transparent diagram for one presentation slide."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyArrowPatch


HERE = Path(__file__).resolve().parent
REGULAR = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
BOLD = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")

INK = "#20232A"
MUTED = "#5D6570"
RULE = "#B5BBC3"


def label(ax, x, y, value, *, size=20, bold=False, color=INK, va="center"):
    ax.text(
        x,
        y,
        value,
        color=color,
        fontsize=size,
        fontproperties=BOLD if bold else REGULAR,
        ha="left",
        va=va,
    )


def arrow(ax, start, end, y):
    ax.add_patch(
        FancyArrowPatch(
            (start, y),
            (end, y),
            arrowstyle="-|>",
            mutation_scale=12,
            linewidth=1.5,
            color=RULE,
            shrinkA=0,
            shrinkB=0,
        )
    )


def flow(ax, items, spans, y):
    for x, value in items:
        label(ax, x, y, value, size=21)
    for start, end in spans:
        arrow(ax, start, end, y)


def main():
    plt.rcParams["svg.fonttype"] = "path"
    fig = plt.figure(figsize=(16, 9), dpi=120)
    fig.patch.set_alpha(0)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1920)
    ax.set_ylim(0, 1080)
    ax.axis("off")

    # The two comparisons share the same column grid and use no filled cards.
    label(ax, 96, 1004, "01   경로 계획 방식", size=22, bold=True)
    ax.plot([96, 1824], [960, 960], color=RULE, linewidth=1.4)
    ax.plot([960, 960], [643, 918], color=RULE, linewidth=1.0)

    label(ax, 96, 905, "A* 단독 계획", size=26, bold=True)
    label(ax, 1020, 905, "우리 팀의 주행", size=26, bold=True)
    flow(
        ax,
        [(100, "저장 지도·목표"), (433, "A* 탐색"), (733, "경로 산출")],
        [(323, 405), (565, 704)],
        791,
    )
    flow(
        ax,
        [(1024, "저장 지도·목표"), (1350, "A* 경로"), (1660, "Nav2 추종")],
        [(1245, 1321), (1471, 1631)],
        791,
    )
    label(ax, 100, 688, "경로 생성 단계. 회피·속도 제어는 별도 구성", size=18, color=MUTED)
    label(ax, 1024, 688, "LiDAR 관측 → 국소 우회·장애물 기억 → 필요 시 재계획", size=18, color=MUTED)

    label(ax, 96, 562, "02   로봇 운용 시스템", size=22, bold=True)
    ax.plot([96, 1824], [518, 518], color=RULE, linewidth=1.4)
    ax.plot([960, 960], [163, 476], color=RULE, linewidth=1.0)

    label(ax, 96, 463, "TurtleBot3 · Nav2 실습 예시", size=26, bold=True)
    label(ax, 1020, 463, "우리 팀 통합 운용", size=26, bold=True)
    flow(
        ax,
        [(100, "RViz 목표"), (430, "AMCL·Nav2"), (752, "로봇 구동")],
        [(253, 401), (577, 723)],
        339,
    )
    flow(
        ax,
        [(1024, "웹 요청·관제"), (1320, "AMCL·A*·Nav2"), (1661, "안전 제어")],
        [(1211, 1291), (1516, 1632)],
        339,
    )
    label(ax, 100, 236, "RViz에서 목표 지정 · Nav2로 경로 추종", size=18, color=MUTED)
    label(ax, 1024, 252, "LiDAR 장애물 대응 · 바퀴/IMU EKF · 후방 ArUco 확인", size=18, color=MUTED)
    label(ax, 1024, 208, "재고 요청 연속 처리 · 섹터 도착 확인 · Home 복귀", size=18, color=MUTED)

    for ext in ("png", "svg"):
        fig.savefig(
            HERE / f"navigation_system_comparison.{ext}",
            dpi=120,
            transparent=True,
            pad_inches=0,
        )
    plt.close(fig)

    # Matplotlib's SVG writer adds whitespace to path-data lines.
    svg = HERE / "navigation_system_comparison.svg"
    svg.write_text("\n".join(line.rstrip() for line in svg.read_text().splitlines()) + "\n")


if __name__ == "__main__":
    main()
