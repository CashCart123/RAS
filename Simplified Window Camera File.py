
#!/usr/bin/env python3
"""
Streamlined multi‑RTSP viewer (Pygame + OpenCV)

- Hard‑coded camera URLs at the top (no JSON / config file).
- Three layouts selectable from header: Basic grid, 2-by-X, 3-by-X.
- Drag feeds to swap places; right‑click a feed (Basic layout) to zoom, others line up below.
- Per‑feed capture thread with graceful shutdown.
- Aspect‑ratio aware sizing; minimal dependencies.

Notes:
- If your RTSP endpoint needs the standard scheme, change "rtsp:/" to "rtsp://".
- Rotation/flip below matches the original behavior. Tweak ROTATE_TIMES and FLIP_VERTICAL if needed.
"""

import sys
import threading
import cv2
import pygame
import imutils

# ==============================
# HARD-CODED CAMERA ENDPOINTS
# ==============================
CAMERA_URLS = [
    'rtsp:/ras:ras@192.168.1.13:8554/profile0',
    'rtsp:/ras:ras@192.168.1.14:8554/profile0',
    'rtsp:/ras:ras@192.168.1.15:8554/profile0',
    'rtsp:/ras:ras@192.168.1.18:8554/profile0',
    'rtsp:/ras:ras@192.168.1.19:8554/profile0',
    'rtsp:/ras:ras@192.168.1.22:8554/profile0',
]

# Orientation controls (kept to mirror original behavior)
ROTATE_TIMES = 3        # 3 x 90° CW = 270°
FLIP_VERTICAL = True    # flip after rotation

# UI constants
HEADER_HEIGHT = 50
LAYOUT_OPTIONS = ["Basic", "2byX", "3byX"]
FPS = 30

pygame.init()
screen_info = pygame.display.Info()
SCREEN_W, SCREEN_H = screen_info.current_w, screen_info.current_h
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.RESIZABLE)
clock = pygame.time.Clock()


class Camera:
    def __init__(self, url: str, index: int):
        self.url = url
        self.index = index
        self.position = [0, 0]
        self.size = [320, 240]
        self.state = 'normal'
        self.aspect_ratio = 1.0

        self.frame = None
        self.capture = None
        self.running = False
        self.thread = None

    def start(self):
        def loop():
            self.capture = cv2.VideoCapture(self.url)
            if not self.capture.isOpened():
                print(f"[WARN] Camera {self.index+1}: failed to open {self.url}")
                return
            while self.running:
                ok, frame = self.capture.read()
                if not ok:
                    print(f"[WARN] Camera {self.index+1}: stream ended / disconnected")
                    break
                h, w = frame.shape[:2]
                self.aspect_ratio = h / w if w else 1.0
                self.frame = frame
            try:
                if self.capture is not None:
                    self.capture.release()
            except Exception:
                pass

        self.running = True
        self.thread = threading.Thread(target=loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.capture:
            try:
                self.capture.release()
            except Exception:
                pass

    def draw(self, surface: pygame.Surface):
        if self.frame is None:
            return
        x, y = self.position
        w, h = self.size

        # keep header area free
        if y < HEADER_HEIGHT:
            return

        # fit by height to preserve aspect
        frame = imutils.resize(self.frame, height=h)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # rotation
        for _ in range(ROTATE_TIMES % 4):
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        # optional vertical flip
        if FLIP_VERTICAL:
            frame = cv2.flip(frame, 0)

        surf = pygame.surfarray.make_surface(frame)
        surface.blit(surf, (x, y))

        # label
        font = pygame.font.SysFont(None, 24)
        lbl = font.render(f"Camera {self.index + 1}", True, (255, 255, 255))
        surface.blit(lbl, (x + 5, y + 5))


class CameraManager:
    def __init__(self, urls):
        self.cameras = [Camera(url, i) for i, url in enumerate(urls)]
        for cam in self.cameras:
            cam.start()

        self.selected_layout = "Basic"
        self.zoomed_index = None

        # drag state
        self.dragging_index = None
        self.offset = (0, 0)
        self.drag_original_pos = None
        self.drag_original_size = None

        self.header_buttons = []
        self.apply_layout()

    # ------- layout helpers -------
    def _scale_size_to_fit(self, target_size, aspect_ratio):
        max_w, max_h = target_size
        h_based_on_w = int(max_w * aspect_ratio)
        if h_based_on_w <= max_h:
            return max_w, h_based_on_w
        w_based_on_h = int(max_h / aspect_ratio) if aspect_ratio else max_w
        return w_based_on_h, max_h

    def _layout_row_compact(self, indices, top_y, avail_w, avail_h):
        if not indices:
            return
        n = len(indices)
        equal_w = max(1, avail_w // n)

        # natural sizes by equal width, then scale to fit within avail_h and avail_w
        sizes = []
        for idx in indices:
            aspect = self.cameras[idx].aspect_ratio
            h = int(equal_w * aspect)
            sizes.append({'i': idx, 'w': equal_w, 'h': h, 'aspect': aspect})

        max_h = max(s['h'] for s in sizes)
        if max_h > avail_h:
            scale = avail_h / max_h
            for s in sizes:
                s['h'] = int(s['h'] * scale)
                s['w'] = int(s['h'] / (s['aspect'] or 1.0))

        total_w = sum(s['w'] for s in sizes)
        if total_w > avail_w:
            scale = avail_w / total_w
            for s in sizes:
                s['w'] = int(s['w'] * scale)
                s['h'] = int(s['w'] * s['aspect'])

        x = 0
        for s in sizes:
            cam = self.cameras[s['i']]
            cam.position = [x, top_y]
            cam.size = [s['w'], s['h']]
            cam.state = 'normal'
            x += s['w']

    def _grid_layout(self, indices):
        n = len(indices)
        if n == 0:
            return
        cols = max(1, int(n ** 0.5))
        rows = (n + cols - 1) // cols

        avail_h = SCREEN_H - HEADER_HEIGHT
        cell_w = SCREEN_W // cols
        cell_h = avail_h // rows

        for k, idx in enumerate(indices):
            cam = self.cameras[idx]
            aspect = cam.aspect_ratio or 1.0
            col = k % cols
            row = k // cols

            w = cell_w
            h = int(w * aspect)
            if h > cell_h:
                h = cell_h
                w = int(h / aspect)

            x = col * cell_w + (cell_w - w) // 2
            y = HEADER_HEIGHT + row * cell_h + (cell_h - h) // 2

            cam.position = [x, y]
            cam.size = [w, h]
            cam.state = 'normal'

    def apply_layout(self):
        self.zoomed_index = None
        n = len(self.cameras)

        if self.selected_layout == "2byX":
            focused = [i for i in range(min(2, n))]
            rest = [i for i in range(n) if i not in focused]

            half_w = SCREEN_W // 2
            top_h = int(half_w * (self.cameras[focused[0]].aspect_ratio if focused else 1.0))

            for i, cam_idx in enumerate(focused):
                cam = self.cameras[cam_idx]
                cam.position = [i * half_w, HEADER_HEIGHT]
                cam.size = [half_w, top_h]
                cam.state = 'focused'

            row_y = HEADER_HEIGHT + top_h
            self._layout_row_compact(rest, row_y, SCREEN_W, max(0, SCREEN_H - row_y))

        elif self.selected_layout == "3byX":
            focused = [i for i in range(min(3, n))]
            rest = [i for i in range(n) if i not in focused]

            third_w = SCREEN_W // 3 if 3 <= n else (SCREEN_W // max(1, len(focused) or 1))
            top_h = int(third_w * (self.cameras[focused[0]].aspect_ratio if focused else 1.0))

            for i, cam_idx in enumerate(focused):
                cam = self.cameras[cam_idx]
                cam.position = [i * third_w, HEADER_HEIGHT]
                cam.size = [third_w, top_h]
                cam.state = 'focused'

            row_y = HEADER_HEIGHT + top_h
            self._layout_row_compact(rest, row_y, SCREEN_W, max(0, SCREEN_H - row_y))

        else:
            # Basic grid
            self._grid_layout([i for i in range(n)])

    # ------- input & rendering -------
    def draw_header(self, surface):
        font = pygame.font.SysFont(None, 28)
        btn_w = SCREEN_W // 3
        self.header_buttons = []

        for i, opt in enumerate(LAYOUT_OPTIONS):
            rect = pygame.Rect(i * btn_w, 0, btn_w, HEADER_HEIGHT)
            color = (80, 80, 200) if self.selected_layout == opt else (50, 50, 50)
            pygame.draw.rect(surface, color, rect)
            txt = font.render(opt, True, (255, 255, 255))
            surface.blit(txt, txt.get_rect(center=rect.center))
            self.header_buttons.append((rect, opt))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = event.pos
            if my <= HEADER_HEIGHT:
                for rect, opt in self.header_buttons:
                    if rect.collidepoint(mx, my):
                        self.selected_layout = opt
                        self.apply_layout()
                        return

            # pick a camera
            for i, cam in enumerate(self.cameras):
                x, y = cam.position
                w, h = cam.size
                if x <= mx <= x + w and y <= my <= y + h:
                    if event.button == 1:
                        # start dragging
                        self.dragging_index = i
                        self.offset = (mx - x, my - y)
                        self.drag_original_pos = cam.position[:]
                        self.drag_original_size = cam.size[:]
                    elif event.button == 3 and self.selected_layout == "Basic":
                        # toggle zoom
                        if self.zoomed_index == i:
                            self.zoomed_index = None
                            self.apply_layout()
                        else:
                            self.zoomed_index = i
                            cam.state = 'zoomed'
                            cam.position = [0, HEADER_HEIGHT]
                            zoom_h = int(SCREEN_H * 0.7)
                            cam.size = [SCREEN_W, zoom_h]

                            # others below
                            others = [j for j in range(len(self.cameras)) if j != i]
                            row_y = HEADER_HEIGHT + zoom_h
                            self._layout_row_compact(others, row_y, SCREEN_W, max(0, SCREEN_H - row_y))
                    break

        elif event.type == pygame.MOUSEBUTTONUP:
            if self.dragging_index is not None:
                dragged = self.cameras[self.dragging_index]
                drect = pygame.Rect(dragged.position[0], dragged.position[1], dragged.size[0], dragged.size[1])
                drect.inflate_ip(5, 5)

                swapped = False
                for i, cam in enumerate(self.cameras):
                    if i == self.dragging_index:
                        continue
                    crect = pygame.Rect(cam.position[0], cam.position[1], cam.size[0], cam.size[1])
                    crect.inflate_ip(5, 5)
                    if drect.colliderect(crect):
                        # swap positions (preserve visual sizes sensibly)
                        cam_pos, cam_size, cam_ar = cam.position[:], cam.size[:], cam.aspect_ratio
                        drag_pos, drag_size, drag_ar = self.drag_original_pos, self.drag_original_size, dragged.aspect_ratio

                        cam.position = drag_pos
                        dragged.position = cam_pos

                        cam.size = list(self._scale_size_to_fit(drag_size, cam_ar or 1.0))
                        dragged.size = list(self._scale_size_to_fit(cam_size, drag_ar or 1.0))
                        swapped = True
                        break

                if not swapped:
                    dragged.position = self.drag_original_pos
                    dragged.size = self.drag_original_size

                self.dragging_index = None
                self.drag_original_pos = None
                self.drag_original_size = None

        elif event.type == pygame.MOUSEMOTION and self.dragging_index is not None and self.zoomed_index is None:
            mx, my = event.pos
            if my <= HEADER_HEIGHT:
                my = HEADER_HEIGHT + 1
            cam = self.cameras[self.dragging_index]
            cam.position = [mx - self.offset[0], my - self.offset[1]]

    def draw(self, surface):
        for cam in self.cameras:
            cam.draw(surface)

    def shutdown(self):
        for cam in self.cameras:
            cam.stop()


def main():
    global SCREEN_W, SCREEN_H, screen
    manager = CameraManager(CAMERA_URLS)
    running = True
    try:
        while running:
            screen.fill((0, 0, 0))
            manager.draw_header(screen)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.VIDEORESIZE:
                    SCREEN_W, SCREEN_H = event.w, event.h
                    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.RESIZABLE)
                    manager.apply_layout()
                else:
                    manager.handle_event(event)

            manager.draw(screen)
            pygame.display.flip()
            clock.tick(FPS)
    except KeyboardInterrupt:
        pass
    finally:
        manager.shutdown()
        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    main()
