#!/usr/bin/env python3
"""Generate a detailed, theme-neutral architecture diagram for one PPT slide."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


HERE = Path(__file__).resolve().parent
REGULAR = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
BOLD = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")

INK = "#20232A"
MUTED = "#5D6570"
RULE = "#B5BBC3"


def text(ax, x, y, value, size=17, *, bold=False, color=INK, ha="left"):
    ax.text(
        x, y, value,
        color=color,
        fontsize=size,
        fontproperties=BOLD if bold else REGULAR,
        ha=ha,
        va="center",
        linespacing=1.18,
    )


def line(ax, x1, y1, x2, y2, *, width=1.15):
    ax.plot([x1, x2], [y1, y2], color=RULE, linewidth=width)


def arrow(ax, x1, y1, x2, y2, *, width=1.45):
    ax.add_patch(
        FancyArrowPatch(
            (x1, y1), (x2, y2),
            arrowstyle="-|>", mutation_scale=11,
            linewidth=width, color=RULE,
            shrinkA=0, shrinkB=0,
        )
    )


def box(ax, x, cy, width, height, value, *, size=16.5, strong=False):
    ax.add_patch(
        FancyBboxPatch(
            (x, cy - height / 2), width, height,
            boxstyle="round,pad=0.0,rounding_size=6",
            linewidth=1.45 if strong else 1.1,
            edgecolor=INK if strong else RULE,
            facecolor="none",
        )
    )
    if "\n" in value:
        size = min(size, 13.5)
    text(ax, x + width / 2, cy, value, size, bold=strong, ha="center")


def row(ax, cy, specs, *, height=64, size=16.5, strong_index=None):
    for i, (x, width, value) in enumerate(specs):
        box(ax, x, cy, width, height, value, size=size, strong=(i == strong_index))
        if i:
            px, pw, _ = specs[i - 1]
            arrow(ax, px + pw + 7, cy, x - 7, cy)


def down(ax, x, upper_center, lower_center, height=56):
    arrow(ax, x, upper_center - height / 2 - 4, x, lower_center + height / 2 + 4)


def main():
    plt.rcParams["svg.fonttype"] = "path"
    fig = plt.figure(figsize=(16, 9), dpi=120)
    fig.patch.set_alpha(0)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1920)
    ax.set_ylim(0, 1080)
    ax.axis("off")

    # Top comparison: a standalone grid search versus a complete driving loop.
    text(ax, 80, 1017, "01   경로 계획과 장애물 대응", 24, bold=True)
    line(ax, 80, 974, 1830, 974, width=1.4)
    line(ax, 555, 384, 555, 944)
    text(ax, 92, 925, "기본 A* 탐색", 22, bold=True)
    text(ax, 606, 925, "우리 팀 주행 로직", 22, bold=True)

    box(ax, 112, 841, 356, 56, "저장 지도 + 시작·목표", size=18)
    box(ax, 112, 743, 356, 56, "격자 A* 탐색", size=18)
    box(ax, 112, 645, 356, 56, "경로 좌표 산출", size=18)
    down(ax, 290, 841, 743)
    down(ax, 290, 743, 645)
    text(ax, 112, 557, "A*는 경로 좌표만 계산", 15, color=MUTED)
    text(ax, 112, 515, "속도 명령·회피·추종은 별도 구성", 15, color=MUTED)

    text(ax, 605, 841, "계획", 15, bold=True, color=MUTED)
    row(ax, 841, [
        (725, 248, "저장 지도·AMCL\n섹터 목표"),
        (1006, 235, "전역 A* 경로"),
        (1274, 241, "Nav2 FollowPath"),
        (1548, 247, "/cmd_vel · 주행"),
    ])

    text(ax, 605, 748, "실시간", 15, bold=True, color=MUTED)
    row(ax, 748, [
        (725, 186, "LiDAR /scan"),
        (942, 194, "현재 장애물\n관측층"),
        (1167, 172, "경로 차단 시"),
        (1370, 192, "국소 A* 우회"),
        (1593, 202, "기존 경로\n앞쪽 합류"),
    ], size=16)
    text(ax, 725, 692, "차단이 없으면 기존 경로 유지 · 확정 전 관측도 계획에 즉시 반영", 13.5, color=MUTED)

    text(ax, 605, 623, "기억", 15, bold=True, color=MUTED)
    row(ax, 623, [
        (725, 236, "같은 셀 5회·2초\n반복 관측"),
        (992, 243, "장애물 셀 기억"),
        (1266, 249, "다음 섹터 A*\n계획에 반영"),
        (1546, 249, "빈 공간 5회·2초\n재확인 시 해제"),
    ], size=16, strong_index=1)
    text(ax, 725, 567, "실행 중 섹터 간 유지 · 프로세스 재시작 시 초기화 · 원본 SLAM 지도 불변", 13.5, color=MUTED)

    text(ax, 605, 489, "복구", 15, bold=True, color=MUTED)
    row(ax, 489, [
        (725, 232, "국소 우회 실패"),
        (988, 224, "전체 A* 재계획"),
        (1243, 275, "후방 안전 시 짧게 후진\n새 위치에서 재계획"),
        (1549, 246, "안전 경로 없으면\n목표 유지·정지"),
    ], size=15.5)
    text(ax, 725, 438, "대체 경로가 있으면 바로 추종 · 후진은 직전 궤적과 후방 공간을 확인한 경우에만 수행", 13.5, color=MUTED)
    box(ax, 725, 398, 309, 33, "실패 접근 기억 · 최대 120초", size=14, strong=True)
    text(ax, 1053, 398, "정체 지점의 같은 진입 방향을 잠시 피함 (장애물 셀 기억과 별도)", 13, color=MUTED)

    # Bottom comparison: the operator and robot runtime, including sensor feedback.
    line(ax, 80, 365, 1830, 365, width=1.4)
    text(ax, 80, 337, "02   TurtleBot3 운용 시스템", 24, bold=True)
    line(ax, 80, 302, 1830, 302, width=1.4)
    line(ax, 555, 50, 555, 280)
    text(ax, 92, 271, "기본 Nav2 실습 예시", 21, bold=True)
    text(ax, 606, 271, "우리 팀 통합 시스템", 21, bold=True)

    box(ax, 112, 217, 356, 46, "RViz 초기 위치·Goal", size=16)
    box(ax, 112, 158, 356, 46, "AMCL · Nav2", size=16)
    box(ax, 112, 99, 356, 46, "/cmd_vel → TurtleBot3", size=16)
    down(ax, 290, 217, 158, height=46)
    down(ax, 290, 158, 99, height=46)
    text(ax, 112, 47, "Nav2도 설정에 따라 로컬 회피 가능", 14, color=MUTED)

    row(ax, 203, [
        (725, 240, "웹 재고 요청·관제"),
        (997, 254, "PC: 지도·AMCL\nA*·Nav2"),
        (1283, 265, "Pi: EKF·정지 감시\n속도 중재"),
        (1580, 215, "모터·차량"),
    ], height=62, size=16)
    text(ax, 725, 136, "센서 피드백", 14.5, bold=True)
    text(ax, 894, 136, "LiDAR /scan · 바퀴+IMU → EKF odom · 후방 카메라 → ArUco 섹터 확인", 14.5)
    text(ax, 725, 92, "배송 흐름", 14.5, bold=True)
    text(ax, 894, 92, "섹터 도착 → 마커 확인 → 5초 대기 → 다음 요청 / Home 복귀", 14.5)
    text(ax, 725, 48, "제어 우선순위", 14.5, bold=True)
    text(ax, 894, 48, "웹 수동 조종·비상정지 > 자율주행 · 센서/TF 이상 시 정지", 14.5)

    for ext in ("png", "svg"):
        fig.savefig(HERE / f"navigation_system_comparison.{ext}", dpi=120, transparent=True, pad_inches=0)
    plt.close(fig)

    # Keep the SVG suitable for git diff --check without changing its geometry.
    svg = HERE / "navigation_system_comparison.svg"
    svg.write_text("\n".join(part.rstrip() for part in svg.read_text().splitlines()) + "\n")


if __name__ == "__main__":
    main()
