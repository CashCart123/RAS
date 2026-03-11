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
    'rtsp:/ras:ras@192.168.1.247:554/stream0',
    'rtsp:/ras:ras@192.168.1.6:554/stream1',
    'rtsp:/ras:ras@192.168.1.250:554/stream0',
    'rtsp:/ras:ras@192.168.1.19:8554/stream1',
    'rtsp:/ras:ras@192.168.1.11:554/stream0',
    'rtsp:/ras:ras@192.168.1.22:8554/stream1',
    
    
    

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
                    grid_cols = 3
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
            grid_cols = 3
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
        clock.tick(10)  # Adjust the frame rate as needed

except KeyboardInterrupt:
    pass

for cap in capture_objects:
    if cap is not None:
        cap.release()
pygame.quit()
sys.exit()
