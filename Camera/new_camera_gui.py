import sys
import json
import threading
import cv2
import pygame
import os
import time
import math

CONFIG_PATH = 'camera_gui_config.json'
LOGO_PATH = 'logo.png'
ROVER_NAME = "UNSW RAS" 

BUTTON_SIZE = 32
PADDING = 10
SCALES = [1.0, 0.5, 0.25, 0.1]
SCALE_LABELS = ["100%", "50%", "25%", "10%"]

HEADER_HEIGHT = 75 
FOOTER_HEIGHT = 40

class CameraHandler:
    def __init__(self, index, url):
        self.index = index
        self.url = url
        self.active = True
        self.scale_idx = 0
        self.frame = None
        self.cap = None
        self.stop_event = threading.Event()
        
        self.rect = pygame.Rect(0, 0, 0, 0)
        self.btn_power_rect = pygame.Rect(0, 0, 0, 0)
        self.btn_res_rect = pygame.Rect(0, 0, 0, 0)

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
                        if SCALES[self.scale_idx] != 1.0:
                            w = int(frame.shape[1] * SCALES[self.scale_idx])
                            h = int(frame.shape[0] * SCALES[self.scale_idx])
                            frame = cv2.resize(frame, (w, h))

                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                        frame = cv2.flip(frame, 1)
                        self.frame = pygame.surfarray.make_surface(frame)
                    else:
                        print(f"[FAILSAFE] Camera {self.index} dropped frame. Reconnecting...")
                        self.cap.release()
                        self.frame = None
                        time.sleep(2.0)
                else:
                    self.frame = None
                    time.sleep(2.0)

            except Exception as e:
                print(f"[FAILSAFE] Error on Camera {self.index}: {e}")
                if self.cap: self.cap.release()
                self.frame = None
                time.sleep(2.0)

    def draw(self, surface, tile_rect, font):
        self.rect = tile_rect
        x, y, w, h = tile_rect
        
        pygame.draw.rect(surface, (15, 18, 22), tile_rect)
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
            txt_rect = txt.get_rect(center=(x + w//2, y + h//2))
            surface.blit(txt, txt_rect)

        # UI Buttons
        self.btn_power_rect = pygame.Rect(x + w - BUTTON_SIZE - PADDING, y + PADDING, BUTTON_SIZE, BUTTON_SIZE)
        p_color = (46, 204, 113) if self.active else (231, 76, 60)
        pygame.draw.rect(surface, p_color, self.btn_power_rect, border_radius=6)
        
        self.btn_res_rect = pygame.Rect(x + w - (BUTTON_SIZE*2) - (PADDING*2), y + PADDING, BUTTON_SIZE, BUTTON_SIZE)
        pygame.draw.rect(surface, (52, 152, 219), self.btn_res_rect, border_radius=6)
        
        res_txt = font.render(SCALE_LABELS[self.scale_idx], True, (255, 255, 255))
        res_txt_rect = res_txt.get_rect(center=self.btn_res_rect.center)
        surface.blit(res_txt, res_txt_rect)

def get_smart_grid(count, sw, sh, start_y):
    if count == 0: return []
    
    if count <= 2: rows = 1
    elif count <= 6: rows = 2
    else: rows = 3
    
    rh = sh // rows 
    rects = []

    base_per_row = count // rows
    extras = count % rows
    
    current_cam_idx = 0
    for r in range(rows):
        cams_in_row = base_per_row + (1 if r >= rows - extras else 0)
        
        rw = sw // cams_in_row
        
        for c in range(cams_in_row):
            rects.append(pygame.Rect(c * rw, start_y + (r * rh), rw, rh))
            current_cam_idx += 1
            
    return rects
def save_current_layout(handlers):
    out_data = {"cameras": []}
    for cam in handlers:
        out_data["cameras"].append({"url": cam.url})
    with open(CONFIG_PATH, 'w') as f:
        json.dump(out_data, f, indent=4)

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
    except Exception as e:
        print(f"Failed to load logo: {e}")

if os.path.exists(CONFIG_PATH):
    with open(CONFIG_PATH, 'r') as f:
        urls = [c['url'] for c in json.load(f)['cameras']]
else:
    urls = [
        'rtsp:/ras:ras@192.168.1.247:554/stream0',
        'rtsp:/ras:ras@192.168.1.6:554/stream0',
        'rtsp:/ras:ras@192.168.1.250:554/stream0',
        #'rtsp:/ras:ras@192.168.1.19:8554/stream0',
        'rtsp:/ras:ras@192.168.1.11:554/stream0',
        'rtsp:/ras:ras@192.168.1.22:8554/stream0',] # rtsp urls here

camera_handlers = [CameraHandler(i, url) for i, url in enumerate(urls)]
fullscreen_cam = None

drag_start_pos = None
drag_cam_idx = None
is_dragging = False

running = True
while running:
    sw, sh = screen.get_size()
    screen.fill((10, 12, 15))
    mouse_pos = pygame.mouse.get_pos()
    
    view_y = HEADER_HEIGHT
    view_h = sh - HEADER_HEIGHT - FOOTER_HEIGHT
    
    if fullscreen_cam is not None:
        grid = [pygame.Rect(0, view_y, sw, view_h)]
        active_cams = [fullscreen_cam]
    else:
        grid = get_smart_grid(len(camera_handlers), sw, view_h, view_y)
        active_cams = camera_handlers


    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            clicked_on_ui = False
            for i, cam in enumerate(active_cams):
                if cam.btn_power_rect.collidepoint(event.pos):
                    cam.active = not cam.active
                    clicked_on_ui = True
                    break
                elif cam.btn_res_rect.collidepoint(event.pos):
                    cam.scale_idx = (cam.scale_idx + 1) % len(SCALES)
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
            if drag_start_pos is not None and fullscreen_cam is None:
                dx, dy = event.pos[0] - drag_start_pos[0], event.pos[1] - drag_start_pos[1]
                if math.hypot(dx, dy) > 10:
                    is_dragging = True

        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
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
    pygame.draw.rect(screen, (255, 230, 0), (0, 0, sw, HEADER_HEIGHT)) # #ffe600 Yellow
    pygame.draw.line(screen, (138, 43, 226), (0, HEADER_HEIGHT-1), (sw, HEADER_HEIGHT-1), 3) # Violet Accent Line
    
    title_txt = header_font.render("MARVIN GCS", True, (20, 20, 20))
    text_y = (HEADER_HEIGHT - title_txt.get_height()) // 2
    
    if logo_img:
        screen.blit(logo_img, (20, 6)) 
        screen.blit(title_txt, (50 + logo_w + 20, text_y))
    else:
        logo_txt = header_font.render("[LOGO]", True, (40, 40, 40))
        screen.blit(logo_txt, (20, text_y))
        screen.blit(title_txt, (120, text_y))

    pygame.draw.rect(screen, (15, 18, 22), (0, sh - FOOTER_HEIGHT, sw, FOOTER_HEIGHT))
    pygame.draw.line(screen, (40, 45, 55), (0, sh - FOOTER_HEIGHT), (sw, sh - FOOTER_HEIGHT), 1)
    
    footer_txt = footer_font.render(f"STATUS: ONLINE  |  {ROVER_NAME}", True, (120, 130, 140))
    footer_rect = footer_txt.get_rect(center=(sw // 2, sh - (FOOTER_HEIGHT // 2)))
    screen.blit(footer_txt, footer_rect)

    if is_dragging and drag_cam_idx is not None:
        pygame.draw.circle(screen, (52, 152, 219, 150), mouse_pos, 15)
        font_surf = ui_font.render("MOVING", True, (255, 255, 255))
        screen.blit(font_surf, (mouse_pos[0] + 20, mouse_pos[1] - 10))

    pygame.display.flip()
    clock.tick(30)


for cam in camera_handlers:
    cam.stop_event.set()
    if cam.cap:
        cam.cap.release()
pygame.quit()
sys.exit()