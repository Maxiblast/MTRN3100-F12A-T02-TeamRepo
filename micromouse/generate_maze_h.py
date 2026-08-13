#!/usr/bin/env python3
"""Convert maze_data.txt (ASCII grid) into maze_data.h for the Arduino Maze class."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

MAZE_SIZE = 9
HORIZONTAL_LINE_LEN = 1 + MAZE_SIZE * 4
VERTICAL_LINE_LEN = MAZE_SIZE * 4 + 1
EXPECTED_LINES = 2 * MAZE_SIZE + 1


def set_bit(rows: list[int], row_index: int, bit: int) -> None:
    rows[row_index] |= 1 << bit


def parse_lines(lines: list[str]) -> tuple[list[int], list[int]]:
    vertical = [0] * MAZE_SIZE
    horizontal = [0] * MAZE_SIZE
    h_index = 0
    v_index = 0

    for line_number, line in enumerate(lines, start=1):
        if line.startswith("+"):
            if len(line) != HORIZONTAL_LINE_LEN:
                raise ValueError(
                    f"Line {line_number}: expected horizontal line length "
                    f"{HORIZONTAL_LINE_LEN}, got {len(line)}"
                )

            for i in range(MAZE_SIZE):
                segment = line[1 + i * 4 : 1 + i * 4 + 3]
                if "-" in segment:
                    if h_index < MAZE_SIZE:
                        set_bit(horizontal, i, h_index)
                    else:
                        set_bit(horizontal, i, MAZE_SIZE)

            h_index += 1
            continue

        if len(line) != VERTICAL_LINE_LEN:
            raise ValueError(
                f"Line {line_number}: expected vertical line length "
                f"{VERTICAL_LINE_LEN}, got {len(line)}"
            )

        for i in range(MAZE_SIZE):
            if line[i * 4] == "|":
                set_bit(vertical, v_index, i)

        if line[MAZE_SIZE * 4] == "|":
            set_bit(vertical, v_index, MAZE_SIZE)

        v_index += 1

    if h_index != MAZE_SIZE + 1:
        raise ValueError(
            f"Expected {MAZE_SIZE + 1} horizontal lines, found {h_index}"
        )
    if v_index != MAZE_SIZE:
        raise ValueError(f"Expected {MAZE_SIZE} vertical lines, found {v_index}")

    return vertical, horizontal


def read_maze_txt(path: Path) -> list[str]:
    lines: list[str] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].rstrip("\n")
        if line.strip():
            lines.append(line)
    return lines


def format_array(name: str, values: list[int], size_name: str) -> str:
    rows = ",\n".join(f"    0b{value:010b}" for value in values)
    return f"inline int {name}[{size_name}] = {{\n{rows}\n}};"


def generate_header(vertical: list[int], horizontal: list[int], source: str) -> str:
    return f"""// Auto-generated from {source} by generate_maze_h.py — do not edit.

#pragma once

{format_array("MAZE_VERTICAL_WALLS", vertical, "MAZE_SIZE")}

{format_array("MAZE_HORIZONTAL_WALLS", horizontal, "MAZE_SIZE")}
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input",
        nargs="?",
        default="maze_data.txt",
        help="ASCII maze input file (default: maze_data.txt)",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="maze_data.h",
        help="Generated header output file (default: maze_data.h)",
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.exists():
        print(f"error: input file not found: {input_path}", file=sys.stderr)
        return 1

    lines = read_maze_txt(input_path)
    if len(lines) != EXPECTED_LINES:
        print(
            f"error: expected {EXPECTED_LINES} maze lines, found {len(lines)}",
            file=sys.stderr,
        )
        return 1

    try:
        vertical, horizontal = parse_lines(lines)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    output_path.write_text(
        generate_header(vertical, horizontal, input_path.name),
        encoding="utf-8",
    )
    print(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
