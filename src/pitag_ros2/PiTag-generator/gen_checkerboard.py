#!/usr/bin/env python3
"""
Tạo checkerboard PDF để calibrate camera với ros2 camera_calibration.

Mặc định: 8x7 ô vuông (= 7x6 góc trong), mỗi ô 25mm → khớp với lệnh:
  ros2 run camera_calibration cameracalibrator --size 7x6 --square 0.025
"""
import argparse
import os
import svgwrite
from svgwrite import mm
import cairosvg


def gen_checkerboard(cols=8, rows=7, square_mm=25.0, output="checkerboard"):
    """
    cols, rows  : số ô vuông (không phải góc trong)
    square_mm   : kích thước mỗi ô (mm)
    """
    # Kích thước trang A4
    page_w_mm, page_h_mm = 210.0, 297.0

    board_w_mm = cols * square_mm
    board_h_mm = rows * square_mm

    # Căn giữa trang
    offset_x = (page_w_mm - board_w_mm) / 2
    offset_y = (page_h_mm - board_h_mm) / 2

    svg_file = output + ".svg"
    dwg = svgwrite.Drawing(svg_file,
                           size=(page_w_mm * mm, page_h_mm * mm),
                           debug=False)

    # Nền trắng
    dwg.add(dwg.rect(insert=(0, 0),
                     size=(page_w_mm * mm, page_h_mm * mm),
                     fill="white"))

    # Viền ngoài (giúp cắt chuẩn khi in)
    dwg.add(dwg.rect(insert=(offset_x * mm, offset_y * mm),
                     size=(board_w_mm * mm, board_h_mm * mm),
                     fill="none", stroke="black", stroke_width=0.5))

    # Vẽ các ô đen
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                x = offset_x + c * square_mm
                y = offset_y + r * square_mm
                dwg.add(dwg.rect(insert=(x * mm, y * mm),
                                 size=(square_mm * mm, square_mm * mm),
                                 fill="black"))

    # Thông tin in bên dưới
    inner_corners_x = cols - 1
    inner_corners_y = rows - 1
    info = (f"Checkerboard {cols}x{rows} ô — góc trong: {inner_corners_x}x{inner_corners_y} "
            f"— mỗi ô: {square_mm:.0f}mm")
    cmd  = (f"ros2 run camera_calibration cameracalibrator "
            f"--size {inner_corners_x}x{inner_corners_y} "
            f"--square {square_mm/1000:.3f} "
            f"--ros-args -r image:=/camera/imx335/image_raw -r camera:=/camera/imx335")

    text_y = offset_y + board_h_mm + 8
    dwg.add(dwg.text(info,
                     insert=(page_w_mm / 2 * mm, text_y * mm),
                     text_anchor="middle",
                     font_size="4mm",
                     fill="black"))
    dwg.add(dwg.text(cmd,
                     insert=(page_w_mm / 2 * mm, (text_y + 6) * mm),
                     text_anchor="middle",
                     font_size="3mm",
                     fill="#444444",
                     font_family="monospace"))
    dwg.add(dwg.text(f"⚠ In đúng tỉ lệ 100% (không chọn 'Fit to page')",
                     insert=(page_w_mm / 2 * mm, (text_y + 11) * mm),
                     text_anchor="middle",
                     font_size="3.5mm",
                     fill="red"))

    dwg.save()
    print(f"[OK] SVG: {svg_file}")

    pdf_file = output + ".pdf"
    cairosvg.svg2pdf(url=svg_file, write_to=pdf_file)
    print(f"[OK] PDF: {pdf_file}")
    print(f"\nLệnh calibration:")
    print(f"  {cmd}")
    print(f"\nIn file: {os.path.abspath(pdf_file)}")
    print(f"  → chọn 'actual size' / '100%' khi in, KHÔNG chọn 'fit to page'")
    print(f"  → sau khi in đo ô vuông thực tế = {square_mm:.0f}mm là đúng")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tạo checkerboard PDF cho camera calibration")
    parser.add_argument("--cols",   type=int,   default=8,    help="Số cột ô vuông (mặc định 8)")
    parser.add_argument("--rows",   type=int,   default=7,    help="Số hàng ô vuông (mặc định 7)")
    parser.add_argument("--square", type=float, default=25.0, help="Kích thước ô (mm, mặc định 25)")
    parser.add_argument("--output", type=str,   default="checkerboard", help="Tên file đầu ra")
    args = parser.parse_args()

    gen_checkerboard(args.cols, args.rows, args.square, args.output)
