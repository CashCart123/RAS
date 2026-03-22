import sys
import json
import threading
import cv2
import pygame
import os
import time
import math

# --- FAILSAFE: Import ROS and Armik ---
try:
    import rclpy
    from rclpy.node import Node
    import armik
    from std_msgs.msg import Bool
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"[CRITICAL] ROS2 or armik.py not found. Arm simulation disabled. Reason: {e}")
    ROS_AVAILABLE = False

CONFIG_PATH = 'camera_gui_config.json'
LOGO_PATH = 'logo.png'
ROVER_NAME = "UNSW RAS"

BUTTON_SIZE = 32
PADDING = 10
SCALES = [1.0, 0.5, 0.25, 0.1]
SCALE_LABELS = ["100%", "50%", "25%", "10%"]

HEADER_HEIGHT = 75
FOOTER_HEIGHT = 40

EMAG_STATE = False
EMAG_LAST_UPDATE = 0.0


class CameraHandler:
    def __init__(self, index, url):
        self.index = index
        self.url = url
        self.active = True
        self.scale_idx = 0
        self.frame = None
        self.latest_raw_frame = None
        self.cap = None
        self.stop_event = threading.Event()
        self.flash_timer = 0

        self.rect = pygame.Rect(0, 0, 0, 0)
        self.btn_power_rect = pygame.Rect(0, 0, 0, 0)
        self.btn_res_rect = pygame.Rect(0, 0, 0, 0)
        self.btn_snap_rect = pygame.Rect(0, 0, 0, 0)

        self.start_thread()

    def start_thread(self):
        self.stop_event.clear()
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def _capture_loop(self):
        time.sleep(self.index * 3.0)
        source = int(self.url) if self.url.isdigit() else self.url
        is_rtsp = isinstance(source, str) and source.startswith("rtsp")

        while not self.stop_event.is_set():
            if not self.active:
                time.sleep(0.2)
                continue

            try:
                if self.cap is None or not self.cap.isOpened():
                    backend = cv2.CAP_V4L2 if not is_rtsp else cv2.CAP_ANY
                    self.cap = cv2.VideoCapture(source, backend)

                    if not is_rtsp:
                        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                        self.cap.set(cv2.CAP_PROP_FPS, 30)

                    time.sleep(1.0)

                if self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        self.latest_raw_frame = frame.copy()

                        if SCALES[self.scale_idx] != 1.0:
                            w = int(frame.shape[1] * SCALES[self.scale_idx])
                            h = int(frame.shape[0] * SCALES[self.scale_idx])
                            frame = cv2.resize(frame, (w, h))

                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                        frame = cv2.flip(frame, 1)
                        self.frame = pygame.surfarray.make_surface(frame)
                    else:
                        self.cap.release()
                        self.cap = None
                        self.frame = None
                        self.latest_raw_frame = None
                        time.sleep(2.0)
                else:
                    self.frame = None
                    self.latest_raw_frame = None
                    time.sleep(2.0)

            except Exception:
                if self.cap:
                    self.cap.release()
                self.cap = None
                self.frame = None
                self.latest_raw_frame = None
                time.sleep(2.0)

    def draw(self, surface, tile_rect, font):
        self.rect = tile_rect
        x, y, w, h = tile_rect

        pygame.draw.rect(surface, (15, 18, 22), tile_rect)

        if self.flash_timer > 0:
            pygame.draw.rect(surface, (255, 255, 255), tile_rect, 4)
            self.flash_timer -= 1
        else:
            pygame.draw.rect(surface, (40, 45, 55), tile_rect, 2)

        if self.active and self.frame:
            img_w, img_h = self.frame.get_size()
            aspect_ratio = img_w / img_h

            if w / h > aspect_ratio:
                new_h = h
                new_w = int(h * aspect_ratio)
            else:
                new_w = w
                new_h = int(w / aspect_ratio)

            offset_x = x + (w - new_w) // 2
            offset_y = y + (h - new_h) // 2

            scaled_frame = pygame.transform.scale(self.frame, (new_w, new_h))
            surface.blit(scaled_frame, (offset_x, offset_y))

            hud_txt = font.render(f"RAW STREAM: {img_w}x{img_h}", True, (0, 255, 100))
            surface.blit(hud_txt, (x + 10, y + h - 25))
        else:
            txt = font.render(f"CAMERA {self.index} OFFLINE", True, (100, 105, 115))
            txt_rect = txt.get_rect(center=(x + w // 2, y + h // 2))
            surface.blit(txt, txt_rect)

        self.btn_power_rect = pygame.Rect(x + w - BUTTON_SIZE - PADDING, y + PADDING, BUTTON_SIZE, BUTTON_SIZE)
        p_color = (46, 204, 113) if self.active else (231, 76, 60)
        pygame.draw.rect(surface, p_color, self.btn_power_rect, border_radius=6)

        self.btn_res_rect = pygame.Rect(x + w - (BUTTON_SIZE * 2) - (PADDING * 2), y + PADDING, BUTTON_SIZE, BUTTON_SIZE)
        pygame.draw.rect(surface, (52, 152, 219), self.btn_res_rect, border_radius=6)
        res_txt = font.render(SCALE_LABELS[self.scale_idx], True, (255, 255, 255))
        surface.blit(res_txt, res_txt.get_rect(center=self.btn_res_rect.center))

        self.btn_snap_rect = pygame.Rect(x + w - (BUTTON_SIZE * 3) - (PADDING * 3), y + PADDING, BUTTON_SIZE, BUTTON_SIZE)
        pygame.draw.rect(surface, (155, 89, 182), self.btn_snap_rect, border_radius=6)
        snap_txt = font.render("O", True, (255, 255, 255))
        surface.blit(snap_txt, snap_txt.get_rect(center=self.btn_snap_rect.center))


def get_smart_grid(count, sw, sh, start_y):
    if count == 0:
        return []

    if count <= 2:
        rows = 1
    elif count <= 6:
        rows = 2
    else:
        rows = 3

    rh = sh // rows
    rects = []
    base_per_row = count // rows
    extras = count % rows

    for r in range(rows):
        cams_in_row = base_per_row + (1 if r >= rows - extras else 0)
        rw = sw // cams_in_row
        for c in range(cams_in_row):
            rects.append(pygame.Rect(c * rw, start_y + (r * rh), rw, rh))

    return rects


def save_current_layout(handlers):
    out_data = {"cameras": [{"url": cam.url} for cam in handlers]}
    with open(CONFIG_PATH, 'w') as f:
        json.dump(out_data, f, indent=4)


class EmagStateSubscriber(Node):
    def __init__(self):
        super().__init__('camera_gui_emag_listener')
        self.subscription = self.create_subscription(
            Bool,
            '/emag_state',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global EMAG_STATE, EMAG_LAST_UPDATE
        EMAG_STATE = bool(msg.data)
        EMAG_LAST_UPDATE = time.time()


arm_node = None
emag_node = None

if ROS_AVAILABLE:
    try:
        rclpy.init()

        arm_node = armik.ArmPlot()
        emag_node = EmagStateSubscriber()

        arm_ros_thread = threading.Thread(target=rclpy.spin, args=(arm_node,), daemon=True)
        emag_ros_thread = threading.Thread(target=rclpy.spin, args=(emag_node,), daemon=True)

        arm_ros_thread.start()
        emag_ros_thread.start()

        print("[SUCCESS] Physical Arm simulation node initialized.")
        print("[SUCCESS] EMAG state subscriber initialized.")
    except Exception as e:
        print(f"[FAILSAFE] Error starting ROS node: {e}")
        arm_node = None
        emag_node = None

# --- Main App ---
pygame.init()
screen = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)
pygame.display.set_caption("Rover Mission Control")
clock = pygame.time.Clock()

ui_font = pygame.font.SysFont('Segoe UI', 14, bold=True)
header_font = pygame.font.SysFont('Segoe UI', 24, bold=True)
footer_font = pygame.font.SysFont('Consolas', 16)

logo_img = None
logo_w = 0
if os.path.exists(LOGO_PATH):
    try:
        raw_logo = pygame.image.load(LOGO_PATH).convert_alpha()
        logo_aspect = raw_logo.get_width() / raw_logo.get_height()
        target_h = HEADER_HEIGHT - 12
        logo_w = int(target_h * logo_aspect)
        logo_img = pygame.transform.smoothscale(raw_logo, (logo_w, target_h))
    except Exception:
        pass

if os.path.exists(CONFIG_PATH):
    with open(CONFIG_PATH, 'r') as f:
        urls = [c['url'] for c in json.load(f)['cameras']]
else:
    urls = [
        'rtsp:/ras:ras@192.168.1.247:554/stream0',
        'rtsp:/ras:ras@192.168.1.6:554/stream0',
        'rtsp:/ras:ras@192.168.1.250:554/stream0',
        # 'rtsp:/ras:ras@192.168.1.19:8554/stream0',
        'rtsp:/ras:ras@192.168.1.11:554/stream0',
        'rtsp:/ras:ras@192.168.1.22:8554/stream0'
    ]

camera_handlers = [CameraHandler(i, url) for i, url in enumerate(urls)]
fullscreen_cam = None

drag_start_pos = None
drag_cam_idx = None
is_dragging = False

show_arm_panel = False
panel_width_ratio = 0.20

btn_toggle_arm_rect = pygame.Rect(0, 0, 0, 0)
btn_init_rect = pygame.Rect(0, 0, 0, 0)
active_slider = None

running = True
while running:
    sw, sh = screen.get_size()
    screen.fill((10, 12, 15))
    mouse_pos = pygame.mouse.get_pos()

    view_y = HEADER_HEIGHT
    view_h = sh - HEADER_HEIGHT - FOOTER_HEIGHT

    if show_arm_panel and arm_node is not None:
        camera_area_width = int(sw * (1.0 - panel_width_ratio))
    else:
        camera_area_width = sw

    arm_panel_x = camera_area_width
    arm_panel_width = sw - camera_area_width

    if fullscreen_cam is not None:
        grid = [pygame.Rect(0, view_y, camera_area_width, view_h)]
        active_cams = [fullscreen_cam]
    else:
        grid = get_smart_grid(len(camera_handlers), camera_area_width, view_h, view_y)
        active_cams = camera_handlers

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            clicked_on_ui = False

            if btn_toggle_arm_rect.collidepoint(event.pos) and arm_node is not None:
                show_arm_panel = not show_arm_panel
                clicked_on_ui = True

            elif show_arm_panel and arm_node is not None:
                if btn_init_rect.collidepoint(event.pos):
                    arm_node.trigger_initialisation()
                    clicked_on_ui = True
                else:
                    plot_area_h = view_h - 150
                    ui_y = view_y + plot_area_h + 10

                    tr_w = arm_panel_width - 40
                    tr_x = arm_panel_x + 20

                    if pygame.Rect(tr_x - 10, ui_y + 100 - 15, tr_w + 20, 30).collidepoint(event.pos):
                        active_slider = "speed"
                        clicked_on_ui = True
                    elif pygame.Rect(tr_x - 10, ui_y + 140 - 15, tr_w + 20, 30).collidepoint(event.pos):
                        active_slider = "sens"
                        clicked_on_ui = True

            if not clicked_on_ui:
                for i, cam in enumerate(active_cams):
                    if cam.btn_power_rect.collidepoint(event.pos):
                        cam.active = not cam.active
                        clicked_on_ui = True
                        break
                    elif cam.btn_res_rect.collidepoint(event.pos):
                        cam.scale_idx = (cam.scale_idx + 1) % len(SCALES)
                        clicked_on_ui = True
                        break
                    elif cam.btn_snap_rect.collidepoint(event.pos):
                        if cam.latest_raw_frame is not None:
                            os.makedirs("captures", exist_ok=True)
                            ts = time.strftime("%Y%m%d_%H%M%S")
                            filename = f"captures/cam_{cam.index}_{ts}.jpg"
                            cv2.imwrite(filename, cam.latest_raw_frame)
                            cam.flash_timer = 5
                            print(f"[INFO] Snapshot saved: {filename}")
                        clicked_on_ui = True
                        break

            if not clicked_on_ui:
                for i, cam in enumerate(active_cams):
                    if cam.rect.collidepoint(event.pos):
                        drag_start_pos = event.pos
                        drag_cam_idx = i
                        is_dragging = False
                        break

        elif event.type == pygame.MOUSEMOTION:
            if active_slider and show_arm_panel and arm_node is not None:
                tr_w = arm_panel_width - 40
                tr_x = arm_panel_x + 20
                clamped_x = max(tr_x, min(event.pos[0], tr_x + tr_w))
                val = 0.1 + ((clamped_x - tr_x) / tr_w) * (3.0 - 0.1)

                if active_slider == "speed":
                    arm_node.ee_speed_scale = val
                elif active_slider == "sens":
                    arm_node.joint_sensitivity_scale = val

            elif drag_start_pos is not None and fullscreen_cam is None:
                dx = event.pos[0] - drag_start_pos[0]
                dy = event.pos[1] - drag_start_pos[1]
                if math.hypot(dx, dy) > 10:
                    is_dragging = True

        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            active_slider = None

            if is_dragging and drag_cam_idx is not None and fullscreen_cam is None:
                drop_target_idx = None
                for i, cam in enumerate(active_cams):
                    if cam.rect.collidepoint(event.pos):
                        drop_target_idx = i
                        break

                if drop_target_idx is not None and drop_target_idx != drag_cam_idx:
                    active_cams[drag_cam_idx], active_cams[drop_target_idx] = active_cams[drop_target_idx], active_cams[drag_cam_idx]
                    active_cams[drag_cam_idx].index = drag_cam_idx
                    active_cams[drop_target_idx].index = drop_target_idx
                    save_current_layout(active_cams)

            elif drag_cam_idx is not None:
                cam = active_cams[drag_cam_idx]
                if fullscreen_cam == cam:
                    fullscreen_cam = None
                else:
                    fullscreen_cam = cam

            drag_start_pos = None
            drag_cam_idx = None
            is_dragging = False

    for i, cam in enumerate(active_cams):
        if i < len(grid):
            cam.draw(screen, grid[i], ui_font)

    if show_arm_panel and arm_node is not None:
        pygame.draw.rect(screen, (26, 28, 35), (arm_panel_x, view_y, arm_panel_width, view_h))
        pygame.draw.line(screen, (50, 55, 65), (arm_panel_x, view_y), (arm_panel_x, sh - FOOTER_HEIGHT), 2)

        plot_area_h = view_h - 150

        try:
            with arm_node.buffer_lock:
                arm_frame_data = arm_node.latest_arm_frame
                tele_frame_data = arm_node.latest_telemetry_frame

            if arm_frame_data:
                raw_data, size = arm_frame_data
                arm_surf = pygame.image.frombuffer(raw_data, size, "RGBA")
                arm_surf = pygame.transform.smoothscale(arm_surf, (arm_panel_width - 20, (plot_area_h // 2) - 10))
                screen.blit(arm_surf, (arm_panel_x + 10, view_y + 10))

            if tele_frame_data:
                raw_data, size = tele_frame_data
                tele_surf = pygame.image.frombuffer(raw_data, size, "RGBA")
                tele_surf = pygame.transform.smoothscale(tele_surf, (arm_panel_width - 20, (plot_area_h // 2) - 10))
                screen.blit(tele_surf, (arm_panel_x + 10, view_y + (plot_area_h // 2) + 10))
        except Exception:
            pass

        ui_y = view_y + plot_area_h + 10

        # --- Init button ---
        btn_init_rect = pygame.Rect(arm_panel_x + 20, ui_y, arm_panel_width - 40, 30)
        btn_color = (46, 204, 113) if arm_node.initialising else (231, 76, 60)
        pygame.draw.rect(screen, btn_color, btn_init_rect, border_radius=4)
        init_txt = ui_font.render("RUN INITIALISATION", True, (255, 255, 255))
        screen.blit(init_txt, init_txt.get_rect(center=btn_init_rect.center))

        # --- EMAG indicator inside arm panel ---
        emag_y = ui_y + 40
        emag_stale = (time.time() - EMAG_LAST_UPDATE) > 1.0

        if emag_stale:
            emag_label = "EMAG: --"
            emag_color = (120, 120, 120)
        elif EMAG_STATE:
            emag_label = "EMAG: ON"
            emag_color = (46, 204, 113)
        else:
            emag_label = "EMAG: OFF"
            emag_color = (231, 76, 60)

        emag_rect = pygame.Rect(arm_panel_x + 20, emag_y, arm_panel_width - 40, 28)
        pygame.draw.rect(screen, emag_color, emag_rect, border_radius=6)

        emag_txt = ui_font.render(emag_label, True, (255, 255, 255))
        screen.blit(emag_txt, emag_txt.get_rect(center=emag_rect.center))

        def draw_slider(name, val, y_offset):
            tr_w = arm_panel_width - 40
            tr_x = arm_panel_x + 20
            pygame.draw.rect(screen, (60, 65, 75), (tr_x, ui_y + y_offset, tr_w, 4), border_radius=2)
            knob_x = tr_x + ((val - 0.1) / (3.0 - 0.1)) * tr_w
            pygame.draw.circle(screen, (52, 152, 219), (int(knob_x), ui_y + y_offset + 2), 8)
            lbl = ui_font.render(f"{name}: {val:.2f}", True, (200, 200, 200))
            screen.blit(lbl, (tr_x, ui_y + y_offset - 20))

        draw_slider("Speed", arm_node.ee_speed_scale, 100)
        draw_slider("Sens", arm_node.joint_sensitivity_scale, 140)

    pygame.draw.rect(screen, (138, 43, 226), (0, 0, sw, HEADER_HEIGHT))
    pygame.draw.line(screen, (255, 255, 255), (0, HEADER_HEIGHT - 1), (sw, HEADER_HEIGHT - 1), 3)

    title_txt = header_font.render("MARVIN GCS", True, (255, 255, 255))
    title_rect = title_txt.get_rect(center=(sw // 2, HEADER_HEIGHT // 2))

    if logo_img:
        screen.blit(logo_img, (20, 6))
        screen.blit(title_txt, title_rect)
    else:
        logo_txt = header_font.render("UNSW-RAS", True, (255, 255, 255))
        logo_y = (HEADER_HEIGHT - logo_txt.get_height()) // 2
        screen.blit(logo_txt, (20, logo_y))
        screen.blit(title_txt, title_rect)

    if arm_node is not None:
        btn_toggle_arm_rect = pygame.Rect(sw - 140, 20, 120, 35)
        btn_t_color = (46, 204, 113) if show_arm_panel else (255, 50, 50)
        pygame.draw.rect(screen, btn_t_color, btn_toggle_arm_rect, border_radius=6)
        tog_txt = ui_font.render("ARM SIM", True, (255, 255, 255))
        screen.blit(tog_txt, tog_txt.get_rect(center=btn_toggle_arm_rect.center))
    else:
        btn_toggle_arm_rect = pygame.Rect(0, 0, 0, 0)

    pygame.draw.rect(screen, (15, 18, 22), (0, sh - FOOTER_HEIGHT, sw, FOOTER_HEIGHT))
    pygame.draw.line(screen, (40, 45, 55), (0, sh - FOOTER_HEIGHT), (sw, sh - FOOTER_HEIGHT), 1)

    footer_txt = footer_font.render(f"STATUS: ONLINE  |  {ROVER_NAME}", True, (120, 130, 140))
    footer_rect = footer_txt.get_rect(center=(sw // 2, sh - (FOOTER_HEIGHT // 2)))
    screen.blit(footer_txt, footer_rect)

    if is_dragging and drag_cam_idx is not None:
        pygame.draw.circle(screen, (52, 152, 219), mouse_pos, 15)
        font_surf = ui_font.render("MOVING", True, (255, 255, 255))
        screen.blit(font_surf, (mouse_pos[0] + 20, mouse_pos[1] - 10))

    pygame.display.flip()
    clock.tick(30)

for cam in camera_handlers:
    cam.stop_event.set()
    if cam.cap:
        cam.cap.release()

if ROS_AVAILABLE:
    if arm_node is not None:
        arm_node.destroy_node()
    if emag_node is not None:
        emag_node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

pygame.quit()
sys.exit()