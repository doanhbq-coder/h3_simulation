# -*- coding: utf-8 -*-
"""
PiTag generator by ID.
Tra cứu cross-ratio từ database XML của cob_fiducials rồi gọi pi-tag_gen.py.
"""
import sys
import os
import subprocess

# Toàn bộ ID lấy từ piTagIni_2.xml + piTagIni_3.xml
PITAG_DB = {
    0:  (0.40, 0.60, 0.20, 0.80),
    1:  (0.30, 0.55, 0.25, 0.70),
    2:  (0.25, 0.52, 0.25, 0.62),
    3:  (0.45, 0.75, 0.40, 0.74),
    4:  (0.30, 0.55, 0.25, 0.50),
    5:  (0.25, 0.50, 0.30, 0.65),
    6:  (0.25, 0.50, 0.30, 0.75),
    7:  (0.40, 0.65, 0.25, 0.50),
    8:  (0.25, 0.50, 0.40, 0.75),
    9:  (0.30, 0.55, 0.25, 0.65),
    10: (0.30, 0.65, 0.25, 0.65),
    11: (0.25, 0.60, 0.30, 0.75),
    12: (0.40, 0.65, 0.25, 0.65),
    13: (0.40, 0.75, 0.25, 0.65),
    15: (0.30, 0.65, 0.25, 0.70),
    16: (0.30, 0.75, 0.25, 0.75),
    17: (0.40, 0.65, 0.25, 0.70),
    18: (0.40, 0.75, 0.25, 0.70),
    19: (0.30, 0.55, 0.35, 0.65),
    20: (0.35, 0.60, 0.30, 0.65),
    21: (0.35, 0.60, 0.30, 0.75),
    22: (0.40, 0.65, 0.35, 0.65),
    23: (0.35, 0.60, 0.40, 0.75),
    24: (0.30, 0.55, 0.35, 0.75),
    25: (0.30, 0.65, 0.35, 0.75),
    26: (0.35, 0.70, 0.30, 0.75),
    27: (0.40, 0.65, 0.35, 0.75),
    28: (0.40, 0.75, 0.35, 0.75),
    29: (0.30, 0.55, 0.45, 0.75),
    30: (0.45, 0.70, 0.30, 0.65),
    31: (0.45, 0.70, 0.30, 0.75),
    32: (0.40, 0.65, 0.45, 0.75),
    33: (0.45, 0.70, 0.40, 0.75),
    36: (0.40, 0.60, 0.35, 0.60),
    38: (0.40, 0.60, 0.20, 0.50),
    48: (0.25, 0.45, 0.20, 0.55),
    55: (0.30, 0.55, 0.25, 0.55),
    64: (0.20, 0.45, 0.20, 0.80),
    69: (0.20, 0.55, 0.25, 0.70),
    73: (0.25, 0.70, 0.20, 0.80),
    79: (0.35, 0.65, 0.25, 0.75),
}

AVAILABLE_IDS = sorted(PITAG_DB.keys())


def print_table():
    print("\n{:<6} {:<6} {:<6} {:<6} {:<6}".format("ID", "AB0", "AC0", "AB1", "AC1"))
    print("-" * 35)
    for id_, (ab0, ac0, ab1, ac1) in sorted(PITAG_DB.items()):
        print("{:<6} {:<6} {:<6} {:<6} {:<6}".format(id_, ab0, ac0, ab1, ac1))


A4_WIDTH_CM  = 21.0
A4_HEIGHT_CM = 29.7
A4_MARGIN_CM = 0.5  # margin mỗi cạnh khi in


def fill_a4_params(marker_size=10.0, circle_radius=0.9, circle_clearance=0.2):
    """Scale tất cả tỷ lệ thuận để outer square lấp đầy A4 (chiều ngắn 21cm - 2*margin)."""
    available = A4_WIDTH_CM - 2 * A4_MARGIN_CM  # 20cm
    outer_square_ref = marker_size + 2 * circle_radius + 2 * circle_clearance
    scale = available / outer_square_ref
    return (
        round(marker_size * scale, 3),
        round(circle_radius * scale, 3),
        round(circle_clearance * scale, 3),
    )


def generate(tag_id, marker_size=10.0, circle_radius=0.9, circle_clearance=0.2,
             output_file=None, a4=True, pdf=True, show_info=False, fill_a4=False):
    if tag_id not in PITAG_DB:
        print(f"[LỖI] ID {tag_id} không tồn tại trong database.")
        print(f"Các ID hợp lệ: {AVAILABLE_IDS}")
        return False

    if fill_a4:
        marker_size, circle_radius, circle_clearance = fill_a4_params(marker_size, circle_radius, circle_clearance)
        a4 = True
        print(f"[FILL-A4] marker_size={marker_size} cm | circle_radius={circle_radius} cm | circle_clearance={circle_clearance} cm")
        print(f"  -> Dùng LineWidthHeight value=\"{marker_size/100:.4f}\" trong XML model")

    ab0, ac0, ab1, ac1 = PITAG_DB[tag_id]

    if output_file is None:
        output_file = f"pitag_ID{tag_id}.svg"

    script = os.path.join(os.path.dirname(__file__), "pi-tag_gen.py")

    cmd = [
        sys.executable, script,
        str(ab0), str(ac0), str(ab1), str(ac1),
        "--marker_size", str(marker_size),
        "--circle_radius", str(circle_radius),
        "--circle_clearance", str(circle_clearance),
        "--output_file", output_file,
    ]
    if a4:
        cmd.append("--A4")
    if pdf:
        cmd.append("--pdf")
    if show_info:
        cmd.append("--show_info")

    print(f"\n[THÔNG TIN] ID={tag_id} | AB0={ab0} AC0={ac0} AB1={ab1} AC1={ac1}")
    print(f"[LỆNH] {' '.join(cmd)}\n")

    result = subprocess.run(cmd)
    if result.returncode == 0:
        base = os.path.splitext(output_file)[0]
        print(f"\n[OK] Đã tạo: {base}.svg" + (f" và {base}.pdf" if pdf else ""))
        return True
    else:
        print("\n[LỖI] Sinh marker thất bại.")
        return False


def interactive():
    print("=" * 50)
    print("       PiTag Generator theo ID")
    print("=" * 50)
    print(f"Database có {len(PITAG_DB)} ID: {AVAILABLE_IDS}")
    print("\nGõ 'list' để xem bảng đầy đủ, 'exit' để thoát.\n")

    while True:
        raw = input("Nhập ID (hoặc 'list'/'exit'): ").strip().lower()

        if raw == "exit":
            break
        if raw == "list":
            print_table()
            continue

        try:
            tag_id = int(raw)
        except ValueError:
            print("  -> Phải nhập số nguyên.")
            continue

        size_raw = input(f"  Kích thước marker cm [mặc định 10.0]: ").strip()
        marker_size = float(size_raw) if size_raw else 10.0

        a4_raw = input("  Căn giữa trang A4? [Y/n]: ").strip().lower()
        a4 = a4_raw != "n"

        pdf_raw = input("  Xuất thêm PDF? [Y/n]: ").strip().lower()
        pdf = pdf_raw != "n"

        info_raw = input("  In thông tin kỹ thuật lên marker? [y/N]: ").strip().lower()
        show_info = info_raw == "y"

        generate(tag_id, marker_size=marker_size, a4=a4, pdf=pdf, show_info=show_info)
        print()


def cli():
    """Chạy từ dòng lệnh: python3 pitag_by_id.py <ID> [--size 10] [--no-a4] [--no-pdf] [--info]"""
    import argparse
    parser = argparse.ArgumentParser(description="Sinh PiTag theo ID định nghĩa sẵn")
    parser.add_argument("id", type=int, help="ID của marker (vd: 0, 1, 2, 3 ...)")
    parser.add_argument("--size", type=float, default=10.0, help="Kích thước marker (cm), mặc định 10.0")
    parser.add_argument("--radius", type=float, default=0.9, help="Bán kính vòng tròn (cm), mặc định 0.9")
    parser.add_argument("--clearance", type=float, default=0.2, help="Khoảng trắng (cm), mặc định 0.2")
    parser.add_argument("--output", type=str, default=None, help="Tên file đầu ra (mặc định: pitag_ID<n>.svg)")
    parser.add_argument("--no-a4", action="store_true", help="Không căn giữa A4")
    parser.add_argument("--no-pdf", action="store_true", help="Không xuất PDF")
    parser.add_argument("--info", action="store_true", help="In thông tin kỹ thuật lên marker")
    parser.add_argument("--fill-a4", action="store_true", help="Phóng to tag lấp đầy tờ A4 (bỏ qua --size)")
    parser.add_argument("--list", action="store_true", help="Liệt kê tất cả ID có sẵn")
    args = parser.parse_args()

    if args.list:
        print_table()
        return

    generate(
        args.id,
        marker_size=args.size,
        circle_radius=args.radius,
        circle_clearance=args.clearance,
        output_file=args.output,
        a4=not args.no_a4,
        pdf=not args.no_pdf,
        show_info=args.info,
        fill_a4=args.fill_a4,
    )


if __name__ == "__main__":
    if len(sys.argv) > 1:
        cli()
    else:
        interactive()
