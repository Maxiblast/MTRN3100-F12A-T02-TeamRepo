import cv2
import numpy as np
import json
import os
import sys
import subprocess
from collections import deque


def open_image(path):
    path = os.path.abspath(path)
    try:
        if sys.platform.startswith("win"):
            os.startfile(path)  
        elif sys.platform == "darwin":
            subprocess.run(["open", path], check=False)
        else:
            subprocess.run(["xdg-open", path], check=False)
    except Exception as e:
        print(f"(Could not auto-open {path}: {e}. Open it manually.)")

IMAGE_PATH   = "test3.jpg"   
START_CELL   = (9, 6)     
GOAL_CELL    = (5, 0)     
START_HEADING = "S"         
                            
OUTPUT_PREFIX = "maze_run"  

def load_and_crop(image_path):
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, bright = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    x, y, w, h = cv2.boundingRect(contours[0])
    crop = img[y:y + h, x:x + w]
    return crop

def detect_grid(crop):
    b, g, r = cv2.split(crop.astype(int))
    teal_mask = ((b > 110) & (g > 110) & (r < g - 15) & (r < b - 15)).astype(np.uint8) * 255
    kernel = np.ones((9, 9), np.uint8)
    teal_mask = cv2.dilate(teal_mask, kernel, iterations=1)
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(teal_mask)
    pts = centroids[1:]
    if len(pts) < 8:
        raise RuntimeError("Not enough grid markers detected - check lighting/colour thresholds.")

    # rough initial spacing estimate from 1D clustering
    def cluster_1d(vals, gap=30):
        vals = np.sort(vals)
        clusters, cur = [], [vals[0]]
        for v in vals[1:]:
            if v - cur[-1] > gap:
                clusters.append(cur); cur = [v]
            else:
                cur.append(v)
        clusters.append(cur)
        return [np.mean(c) for c in clusters]

    xc = cluster_1d(pts[:, 0])
    yc = cluster_1d(pts[:, 1])
    x0, dx = xc[0], np.mean(np.diff(xc))
    y0, dy = yc[0], np.mean(np.diff(yc))
    N = max(len(xc), len(yc))  # nodes per side

    for _ in range(5):
        col_idx = np.round((pts[:, 0] - x0) / dx).astype(int)
        row_idx = np.round((pts[:, 1] - y0) / dy).astype(int)
        A = np.vstack([col_idx, np.ones(len(col_idx))]).T
        dx, x0 = np.linalg.lstsq(A, pts[:, 0], rcond=None)[0]
        A2 = np.vstack([row_idx, np.ones(len(row_idx))]).T
        dy, y0 = np.linalg.lstsq(A2, pts[:, 1], rcond=None)[0]
        resid = np.sqrt((pts[:, 0] - (x0 + dx * col_idx)) ** 2 +
                         (pts[:, 1] - (y0 + dy * row_idx)) ** 2)
        pts = pts[resid < 10]

    return x0, dx, y0, dy, N

# Dectecting walls
def node(x0, dx, y0, dy, i, j):
    return (x0 + i * dx, y0 + j * dy)


def is_wall(gray, p1, p2, thresh=130, frac=0.5, perp_win=15):
    x1, y1 = p1
    x2, y2 = p2
    length = np.hypot(x2 - x1, y2 - y1)
    if length == 0:
        return False
    ux, uy = (x2 - x1) / length, (y2 - y1) / length
    px, py = -uy, ux
    dark = total = 0
    for t in np.linspace(0.12, 0.88, 25):
        x, y = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
        vals = []
        for off in range(-perp_win, perp_win + 1):
            xi, yi = int(round(x + px * off)), int(round(y + py * off))
            if 0 <= yi < gray.shape[0] and 0 <= xi < gray.shape[1]:
                vals.append(gray[yi, xi])
        if vals:
            total += 1
            if min(vals) < thresh:
                dark += 1
    return total > 0 and (dark / total) >= frac


def detect_walls(crop, x0, dx, y0, dy, N):
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    h_edges, v_edges = {}, {}
    for j in range(N):
        for i in range(N - 1):
            h_edges[(i, j)] = is_wall(gray, node(x0, dx, y0, dy, i, j),
                                       node(x0, dx, y0, dy, i + 1, j))
    for j in range(N - 1):
        for i in range(N):
            v_edges[(i, j)] = is_wall(gray, node(x0, dx, y0, dy, i, j),
                                       node(x0, dx, y0, dy, i, j + 1))
    return h_edges, v_edges


# Building the graph
def blocked(h_edges, v_edges, r, c, direction, ncells):
    if direction == "N":
        if r == 0:
            return True
        return h_edges.get((c, r), True)
    if direction == "S":
        if r == ncells - 1:
            return True
        return h_edges.get((c, r + 1), True)
    if direction == "E":
        if c == ncells - 1:
            return True
        return v_edges.get((c + 1, r), True)
    if direction == "W":
        if c == 0:
            return True
        return v_edges.get((c, r), True)
    raise ValueError(direction)


DIRS = {"N": (-1, 0), "S": (1, 0), "E": (0, 1), "W": (0, -1)}


def bfs(h_edges, v_edges, start, goal, ncells):
    q = deque([start])
    visited = {start: None}
    while q:
        cur = q.popleft()
        if cur == goal:
            break
        r, c = cur
        for d, (dr, dc) in DIRS.items():
            if blocked(h_edges, v_edges, r, c, d, ncells):
                continue
            nxt = (r + dr, c + dc)
            if nxt not in visited:
                visited[nxt] = cur
                q.append(nxt)
    if goal not in visited:
        return None
    path = [goal]
    while path[-1] != start:
        path.append(visited[path[-1]])
    path.reverse()
    return path

TURN_ORDER = ["N", "E", "S", "W"]  # clockwise order


def path_to_commands(path, start_heading):
    commands = []
    heading = start_heading
    for a, b in zip(path, path[1:]):
        dr, dc = b[0] - a[0], b[1] - a[1]
        want = [d for d, v in DIRS.items() if v == (dr, dc)][0]
        cur_i = TURN_ORDER.index(heading)
        want_i = TURN_ORDER.index(want)
        diff = (want_i - cur_i) % 4
        if diff == 1:
            commands.append("R")
        elif diff == 3:
            commands.append("L")
        elif diff == 2:
            commands += ["R", "R"]
        commands.append("F")
        heading = want
    return commands

def save_overlay(crop, x0, dx, y0, dy, N, h_edges, v_edges, path, out_path):
    overlay = crop.copy()

    def nd(i, j):
        return (int(x0 + i * dx), int(y0 + j * dy))

    for (i, j), val in h_edges.items():
        if val:
            cv2.line(overlay, nd(i, j), nd(i + 1, j), (0, 0, 255), 3)
    for (i, j), val in v_edges.items():
        if val:
            cv2.line(overlay, nd(i, j), nd(i, j + 1), (0, 255, 0), 3)

    if path:
        pts = [nd(c + 0.5, r + 0.5) for r, c in path]
        for p1, p2 in zip(pts, pts[1:]):
            cv2.line(overlay, p1, p2, (255, 0, 255), 4)
        cv2.circle(overlay, pts[0], 10, (0, 200, 0), -1)
        cv2.circle(overlay, pts[-1], 10, (0, 0, 200), -1)

    cv2.imwrite(out_path, overlay)

def main():
    crop = load_and_crop(IMAGE_PATH)

    # Save the input photo (cropped to the board) FIRST, unconditionally,
    # so you always have something to show even if detection/solving fails.
    input_path = f"{OUTPUT_PREFIX}_input.png"
    cv2.imwrite(input_path, crop)

    x0, dx, y0, dy, N = detect_grid(crop)
    ncells = N - 1
    print(f"Detected grid: {N}x{N} nodes -> {ncells}x{ncells} cells")

    h_edges, v_edges = detect_walls(crop, x0, dx, y0, dy, N)

    path = bfs(h_edges, v_edges, START_CELL, GOAL_CELL, ncells)

    overlay_path = f"{OUTPUT_PREFIX}_overlay.png"
    save_overlay(crop, x0, dx, y0, dy, N, h_edges, v_edges, path, overlay_path)

    # Combined ss
    combined_path = f"{OUTPUT_PREFIX}_input_vs_output.png"
    out_img = cv2.imread(overlay_path)
    combined = cv2.hconcat([crop, out_img])
    cv2.imwrite(combined_path, combined)

    if path is None:
        print("NO PATH FOUND between", START_CELL, "and", GOAL_CELL)
        print(f"Saved anyway:\n  {os.path.abspath(input_path)}\n  {os.path.abspath(overlay_path)}\n  {os.path.abspath(combined_path)}")
        print("(check START_CELL/GOAL_CELL are valid, walkable cells)")
        open_image(combined_path)
        return

    commands = path_to_commands(path, START_HEADING)

    print("\nCell path:", path)
    print("\nCommand sequence:")
    print(",".join(commands))

    result_path = f"{OUTPUT_PREFIX}_result.json"
    json.dump({
        "grid_cells": ncells,
        "start": START_CELL, "goal": GOAL_CELL,
        "path": path, "commands": commands,
    }, open(result_path, "w"), indent=2)

    print("\nSaved:")
    for p in (input_path, overlay_path, combined_path, result_path):
        print(" ", os.path.abspath(p))

    open_image(combined_path)


if __name__ == "__main__":
    main()