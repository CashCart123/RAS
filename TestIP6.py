
'''
import sys
import pygame
import cv2
import imutils
import threading
import tkinter as tk

root = tk.Tk()

screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
camera_urls = [
    'rtsp:/ras:ras@192.168.1.13:8554/profile0',
    'rtsp:/ras:ras@192.168.1.14:8554/profile0',
    'rtsp:/ras:ras@192.168.1.15:8554/profile0',
    'rtsp:/ras:ras@192.168.1.18:8554/profile0',
    'rtsp:/ras:ras@192.168.1.19:8554/profile0',
    'rtsp:/ras:ras@192.168.1.22:8554/profile0'
]

pygame.init()

screen = pygame.display.set_mode((screen_width, screen_height))
clock = pygame.time.Clock()

capture_objects = [None] * len(camera_urls)  # Store capture objects globally
frames = [None] * len(camera_urls)

selected_camera = None

def capture_frames(url, camera_index):
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print(f"Failed to open camera: {url}")
        return

    # Store the capture object globally
    capture_objects[camera_index] = cap

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Camera {camera_index} disconnected.")
            break
        frames[camera_index] = frame


threads = []
for i, url in enumerate(camera_urls):
    thread = threading.Thread(target=capture_frames, args=(url, i))
    thread.daemon = True
    thread.start()
    threads.append(thread)

try:
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                if selected_camera is None:
                    grid_cols = 2
                    frame_width = screen_width // grid_cols
                    frame_height = screen_height // ((len(camera_urls) + grid_cols - 1) // grid_cols)
                    for i in range(len(camera_urls)):
                        row = i // grid_cols
                        col = i % grid_cols
                        x = col * frame_width
                        y = row * frame_height
                        if x <= mouse_x < x + frame_width and y <= mouse_y < y + frame_height:
                            selected_camera = i
                            break
                else:
                    selected_camera = None

        screen.fill((0, 0, 0))

        if selected_camera is not None:
            frame = frames[selected_camera]
            if frame is not None:
                frame = imutils.resize(frame, width=screen_width, height=screen_height)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = cv2.flip(frame, 1)
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                frame_surface = pygame.surfarray.make_surface(frame)
                screen.blit(frame_surface, (0, 0))
        else:
            grid_cols = 2
            grid_rows = (len(camera_urls) + grid_cols - 1) // grid_cols
            frame_width = screen_width // grid_cols
            frame_height = screen_height // grid_rows

            for i, frame in enumerate(frames):
                if frame is not None:
                    frame = imutils.resize(frame, width=frame_width, height=frame_height)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame = cv2.flip(frame, 1)
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    frame_surface = pygame.surfarray.make_surface(frame)

                    # Calculate grid position
                    row = i // grid_cols
                    col = i % grid_cols
                    x = col * frame_width
                    y = row * frame_height

                    screen.blit(frame_surface, (x, y))

        pygame.display.update()
        clock.tick(30)  # Adjust the frame rate as needed

except KeyboardInterrupt:
    pass

for cap in capture_objects:
    if cap is not None:
        cap.release()
pygame.quit()
sys.exit()
'''

import sys
import json
import threading
import cv2
import pygame
import imutils
import os
import time

CONFIG_PATH = 'camera_gui_config.json'

pygame.init()
screen_info = pygame.display.Info()
screen_width, screen_height = screen_info.current_w, screen_info.current_h
screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
clock = pygame.time.Clock()

# Load config or initialize default
if os.path.exists(CONFIG_PATH):
    with open(CONFIG_PATH, 'r') as f:
        config = json.load(f)
else:
    config = {
        "cameras": [
            {"url": "rtsp:/ras:ras@192.168.1.13:8554/profile0", "position": [0, 0], "size": [320, 240], "state": "normal"},
            {"url": "rtsp:/ras:ras@192.168.1.14:8554/profile0", "position": [320, 0], "size": [320, 240], "state": "normal"}
        ]
    }

frames = [None] * len(config['cameras'])
capture_objects = [None] * len(config['cameras'])
dragging_index = None
offset_x = offset_y = 0


# Threaded camera capture
def capture_frames(url, index):
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print(f"Failed to open {url}")
        return
    capture_objects[index] = cap

    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Camera {index} disconnected.")
            break
        frames[index] = frame


threads = []
for i, cam in enumerate(config['cameras']):
    t = threading.Thread(target=capture_frames, args=(cam['url'], i))
    t.daemon = True
    t.start()
    threads.append(t)


def save_config():
    with open(CONFIG_PATH, 'w') as f:
        json.dump(config, f, indent=2)


def draw_camera_feed(index):
    frame = frames[index]
    if frame is None:
        return

    cam = config['cameras'][index]
    x, y = cam['position']
    w, h = cam['size']

    frame = imutils.resize(frame, width=w)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_surface = pygame.surfarray.make_surface(frame)
    screen.blit(frame_surface, (x, y))


def handle_mouse(event):
    global dragging_index, offset_x, offset_y

    if event.type == pygame.MOUSEBUTTONDOWN:
        mouse_x, mouse_y = event.pos
        for i, cam in enumerate(config['cameras']):
            x, y = cam['position']
            w, h = cam['size']
            if x <= mouse_x <= x + w and y <= mouse_y <= y + h:
                if event.button == 1:  # Left click = drag
                    dragging_index = i
                    offset_x = mouse_x - x
                    offset_y = mouse_y - y
                elif event.button == 3:  # Right click = toggle size
                    state = cam['state']
                    if state == 'normal':
                        cam['size'] = [screen_width // 2, screen_height // 2]
                        cam['state'] = 'medium'
                    elif state == 'medium':
                        cam['size'] = [screen_width, screen_height]
                        cam['position'] = [0, 0]
                        cam['state'] = 'full'
                    else:
                        cam['size'] = [320, 240]
                        cam['state'] = 'normal'
                break

    elif event.type == pygame.MOUSEBUTTONUP:
        dragging_index = None

    elif event.type == pygame.MOUSEMOTION:
        if dragging_index is not None:
            mx, my = event.pos
            config['cameras'][dragging_index]['position'] = [mx - offset_x, my - offset_y]


# Main loop
try:
    running = True
    while running:
        screen.fill((0, 0, 0))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            else:
                handle_mouse(event)

        for i in range(len(config['cameras'])):
            draw_camera_feed(i)

        pygame.display.flip()
        clock.tick(30)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    save_config()
    for cap in capture_objects:
        if cap is not None:
            cap.release()
    pygame.quit()
    sys.exit()
    