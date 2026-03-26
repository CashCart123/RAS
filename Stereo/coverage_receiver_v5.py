import cv2
import numpy as np
import socket
import pickle
import sys
import os
import subprocess
from datetime import datetime

# CONFIGURATION
GRID_SIZE_M = 30.0  
MAP_DISPLAY_PX = 600 
X_MIN, Z_MIN = -10.0, -23.0
PX_PER_M = MAP_DISPLAY_PX / GRID_SIZE_M 

class CoverageReceiver:
    def __init__(self, sender_ip, tcp_port=5006, udp_port=5005):
        self.sender_ip = sender_ip
        self.tcp_port = tcp_port
        self.udp_port = udp_port
        
        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(('', udp_port))
        self.udp_sock.setblocking(False)

        self.mapping_active = False
        self.process_complete = False
        self.status_log = ["System Ready."]
        
        self.mesh_name = "N/A"
        self.mesh_size = "N/A"
        self.img_name = "N/A"
        self.img_size = "N/A"
        
        vox_dim = int(GRID_SIZE_M / 0.10)
        self.voxel_grid = np.zeros((vox_dim, vox_dim), dtype=np.uint8)
        self.path_history = []
        self.start_pos = None
        self.telemetry = {'pos': [0,0,0], 'rot': [0,0,0], 'time': 0, 'nodes': 0}

    def connect(self):
        try:
            self.tcp_sock.connect((self.sender_ip, self.tcp_port))
            self.tcp_sock.setblocking(False)
            return True
        except: return False

    def to_px(self, wx, wz):
        px = int(((wx - X_MIN) / GRID_SIZE_M) * MAP_DISPLAY_PX)
        pz = int(((wz - Z_MIN) / GRID_SIZE_M) * MAP_DISPLAY_PX)
        return np.clip(px, 0, MAP_DISPLAY_PX-1), np.clip(pz, 0, MAP_DISPLAY_PX-1)

    def draw_ui(self):
        m_map = cv2.resize(self.voxel_grid * 255, (MAP_DISPLAY_PX, MAP_DISPLAY_PX), interpolation=cv2.INTER_NEAREST)
        m_map_c = cv2.applyColorMap(m_map, cv2.COLORMAP_JET)
        m_map_c[m_map == 0] = 30 

        for i in range(int(GRID_SIZE_M) + 1):
            pos = int(i * PX_PER_M)
            cv2.line(m_map_c, (pos, 0), (pos, MAP_DISPLAY_PX), (65, 65, 65), 1)
            cv2.line(m_map_c, (0, pos), (MAP_DISPLAY_PX, pos), (65, 65, 65), 1)

        for i in range(1, len(self.path_history)):
            cv2.line(m_map_c, self.path_history[i-1], self.path_history[i], (200, 200, 200), 1)

        if self.start_pos:
            sx, sz = self.to_px(self.start_pos[0], self.start_pos[2])
            cv2.circle(m_map_c, (sx, sz), 5, (255, 255, 255), -1) 

        cx, cz = self.to_px(self.telemetry['pos'][0], self.telemetry['pos'][2])
        cv2.circle(m_map_c, (cx, cz), 6, (203, 192, 255), -1) 

        # Corrected Arrow - Flipped Y (ZED vs OpenCV Y-axis)
        y_rad = np.radians(self.telemetry['rot'][1])
        ax = int(cx + 30 * np.sin(y_rad)) 
        az = int(cz + 30 * np.cos(y_rad)) # Changed - to + to flip vertical orientation
        cv2.arrowedLine(m_map_c, (cx, cz), (ax, az), (0, 255, 0), 2, tipLength=0.3)

        sidebar = np.zeros((MAP_DISPLAY_PX + 120, 500, 3), dtype=np.uint8)
        y_off = 35
        rel_x, rel_y, rel_z = (0, 0, 0)
        if self.start_pos:
            rel_x = self.telemetry['pos'][0] - self.start_pos[0]
            rel_y = self.telemetry['pos'][1] - self.start_pos[1]
            rel_z = self.telemetry['pos'][2] - self.start_pos[2]

        lines = [
            f"TIME: {int(self.telemetry['time'])}s | NODES: {self.telemetry['nodes']}",
            "-------------------------------------------",
            f"REL X: {rel_x:+.2f}m | REL Y: {rel_y:+.2f}m | REL Z: {rel_z:+.2f}m",
            "-------------------------------------------",
            "FILE REGISTRY:",
            f"MAP: {self.mesh_name} ({self.mesh_size})",
            f"IMG: {self.img_name} ({self.img_size})",
            "-------------------------------------------"
        ]
        for line in lines:
            cv2.putText(sidebar, line, (20, y_off), 1, 1.0, (255, 255, 255), 1)
            y_off += 30

        for i, msg in enumerate(self.status_log[-8:]):
            cv2.putText(sidebar, f"> {msg[:55]}", (15, 380 + i*22), 1, 0.7, (0, 200, 0), 1)

        ctrl_bar = np.zeros((120, MAP_DISPLAY_PX, 3), dtype=np.uint8)
        b_col = (0, 150, 0) if not self.mapping_active else (0, 0, 150)
        b_txt = "COMMENCE MAPPING" if not self.mapping_active else "SAVE & EXIT"
        if self.process_complete: b_col, b_txt = (150, 0, 150), "FINISH"
        
        cv2.rectangle(ctrl_bar, (150, 20), (450, 100), b_col, -1)
        cv2.putText(ctrl_bar, b_txt, (185 if "COMMENCE" in b_txt else 250, 65), 1, 1.2, (255, 255, 255), 2)

        return np.hstack((np.vstack((m_map_c, ctrl_bar)), sidebar))

    def auto_scp(self, filename):
        self.status_log.append("Initiating SCP...")
        # Fixed command string formatting
        scp_cmd = f"sshpass -p \"RASunsw\" scp unsw-mapping@{self.sender_ip}:~/Desktop/mapping/{filename} /home/elder3/Downloads/"
        try:
            subprocess.Popen(scp_cmd, shell=True)
            self.status_log.append("SCP sent to background.")
        except Exception as e:
            self.status_log.append(f"SCP Error: {str(e)}")

    def click_handler(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and 150 <= x <= 450 and 620 <= y <= 700:
            if not self.mapping_active and not self.process_complete:
                self.tcp_sock.sendall(b"START")
                self.mapping_active = True
            elif self.mapping_active:
                self.tcp_sock.sendall(b"EXIT")
                self.mapping_active = False
            elif self.process_complete: sys.exit()

    def run(self):
        cv2.namedWindow("Rover Control")
        cv2.setMouseCallback("Rover Control", self.click_handler)
        
        while True:
            try:
                data = self.tcp_sock.recv(1024).decode()
                if data:
                    if "FILE_INFO" in data:
                        _, name, size = data.split("|")
                        self.mesh_name, self.mesh_size = name, size
                        self.process_complete = True
                        
                        ts = datetime.now().strftime('%H%M_%d_%m')
                        self.img_name = f"ui_summary_{ts}.png"
                        cv2.imwrite(self.img_name, self.draw_ui())
                        self.img_size = f"{os.path.getsize(self.img_name) / (1024*1024):.2f} MB"
                        
                        self.auto_scp(name)
                    else:
                        self.status_log.append(data)
            except BlockingIOError: pass
            except: break

            try:
                while True: 
                    d, _ = self.udp_sock.recvfrom(65535)
                    p = pickle.loads(d)
                    if self.start_pos is None: self.start_pos = p['pos']
                    self.telemetry.update({'pos': p['pos'], 'rot': p['rot'], 'time': p['time']})
                    
                    # Update Nodes Count logic
                    for row, col in p['nodes']: self.voxel_grid[row, col] = 1
                    self.telemetry['nodes'] = int(np.sum(self.voxel_grid))
                    
                    px, pz = self.to_px(p['pos'][0], p['pos'][2])
                    if not self.path_history or self.path_history[-1] != (px, pz):
                        self.path_history.append((px, pz))
            except BlockingIOError: pass

            cv2.imshow("Rover Control", self.draw_ui())
            if cv2.waitKey(1) == ord('q'): break

if __name__ == "__main__":
    rec = CoverageReceiver("192.168.1.42")
    if rec.connect(): rec.run()
    else: print("Connection Failed.")
