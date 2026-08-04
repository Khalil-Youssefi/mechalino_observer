#!/usr/bin/env python3

import re
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


ROWS = 4
COLS = 11

ESP_DEBUG_URL = "http://192.168.50.15/debug"

CELL_PX = 60
GRID_LINE_PX = 2

IMG_W = COLS * CELL_PX
IMG_H = ROWS * CELL_PX


class debug_map_viz(Node):
    def __init__(self):
        super().__init__("debug_map_viz_node")

        self.pub_visits = self.create_publisher(Image, "/debug/map_visits", 10)
        self.pub_penalties = self.create_publisher(Image, "/debug/map_penalties_x10", 10)
        self.pub_obstacles = self.create_publisher(Image, "/debug/map_obstacles", 10)

        self.timer = self.create_timer(5.0, self.timer_cb)

        self.get_logger().info("Debug map viz node started")

    def timer_cb(self):
        try:
            text = self.fetch_debug_text()

            visits = self.parse_matrix(text, "VISITS")
            penalties = self.parse_matrix(text, "PENALTIES_X10")
            obstacles = self.parse_obstacles(text)

            self.pub_visits.publish(
                self.matrix_to_image(visits, mode="visits", frame_id="debug_map_visits")
            )
            self.pub_penalties.publish(
                self.matrix_to_image(penalties, mode="penalties", frame_id="debug_map_penalties")
            )
            self.pub_obstacles.publish(
                self.matrix_to_image(obstacles, mode="obstacles", frame_id="debug_map_obstacles")
            )

        except Exception as e:
            self.get_logger().warn(f"Could not update debug maps: {e}")

    def fetch_debug_text(self):
        with urllib.request.urlopen(ESP_DEBUG_URL, timeout=7.0) as response:
            return response.read().decode("utf-8", errors="replace")

    def parse_matrix(self, text, key):
        m = re.search(rf"^{key}=(.+)$", text, re.MULTILINE)
        if not m:
            raise ValueError(f"{key} not found")

        rows_txt = m.group(1).strip().split(";")

        if len(rows_txt) != ROWS:
            raise ValueError(f"{key} has wrong row count")

        mat = []

        for row_txt in rows_txt:
            vals = [int(x) for x in row_txt.split(",")]

            if len(vals) != COLS:
                raise ValueError(f"{key} has wrong column count")

            mat.append(vals)

        return mat

    def parse_obstacles(self, text):
        m = re.search(r"^OBSTACLES=(.+)$", text, re.MULTILINE)
        if not m:
            raise ValueError("OBSTACLES not found")

        masks = m.group(1).strip().split(",")

        if len(masks) != ROWS:
            raise ValueError("OBSTACLES has wrong row count")

        mat = []

        for r in range(ROWS):
            mask = int(masks[r], 16)
            row = []

            for c in range(COLS):
                occupied = 1 if (mask & (1 << c)) else 0
                row.append(occupied)

            mat.append(row)

        return mat

    def matrix_to_image(self, mat, mode, frame_id):
        max_val = max(max(row) for row in mat)
        if max_val <= 0:
            max_val = 1

        data = bytearray(IMG_W * IMG_H * 3)

        for r in range(ROWS):
            for c in range(COLS):
                value = mat[r][c]

                if mode == "visits":
                    rgb = self.color_scale(value, max_val, base=(40, 80, 255))

                elif mode == "penalties":
                    rgb = self.color_scale(value, max_val, base=(255, 120, 20))

                elif mode == "obstacles":
                    rgb = (220, 20, 20) if value > 0 else (240, 240, 240)

                else:
                    rgb = (255, 255, 255)

                self.fill_cell(data, r, c, rgb)

        self.draw_grid(data)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = IMG_H
        msg.width = IMG_W
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = IMG_W * 3
        msg.data = bytes(data)

        return msg

    def color_scale(self, value, max_val, base):
        if value <= 0:
            return (245, 245, 245)

        t = value / max_val
        if t > 1.0:
            t = 1.0

        r = int(245 * (1.0 - t) + base[0] * t)
        g = int(245 * (1.0 - t) + base[1] * t)
        b = int(245 * (1.0 - t) + base[2] * t)

        return (r, g, b)

    def fill_cell(self, data, r, c, rgb):
        y0 = r * CELL_PX
        y1 = y0 + CELL_PX
        x0 = c * CELL_PX
        x1 = x0 + CELL_PX

        for y in range(y0, y1):
            for x in range(x0, x1):
                idx = (y * IMG_W + x) * 3
                data[idx + 0] = rgb[0]
                data[idx + 1] = rgb[1]
                data[idx + 2] = rgb[2]

    def draw_grid(self, data):
        grid_rgb = (0, 0, 0)

        for r in range(ROWS + 1):
            y = r * CELL_PX
            if y >= IMG_H:
                y = IMG_H - 1

            for dy in range(GRID_LINE_PX):
                yy = y + dy
                if yy >= IMG_H:
                    continue

                for x in range(IMG_W):
                    idx = (yy * IMG_W + x) * 3
                    data[idx + 0] = grid_rgb[0]
                    data[idx + 1] = grid_rgb[1]
                    data[idx + 2] = grid_rgb[2]

        for c in range(COLS + 1):
            x = c * CELL_PX
            if x >= IMG_W:
                x = IMG_W - 1

            for dx in range(GRID_LINE_PX):
                xx = x + dx
                if xx >= IMG_W:
                    continue

                for y in range(IMG_H):
                    idx = (y * IMG_W + xx) * 3
                    data[idx + 0] = grid_rgb[0]
                    data[idx + 1] = grid_rgb[1]
                    data[idx + 2] = grid_rgb[2]


def main(args=None):
    rclpy.init(args=args)
    node = debug_map_viz()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()