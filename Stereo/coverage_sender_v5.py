import pyzed.sl as sl
import numpy as np
import cv2
import time
import socket
import pickle
import os
import sys
from datetime import datetime

class CoverageSender:
    def __init__(self, headless=True):
        self.udp_port = 5005
        self.tcp_port = 5006
        self.program_running = True 
        self.receiver_conns = {} 
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp_server.setblocking(False)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def get_local_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 1))
            IP = s.getsockname()[0]
        except: IP = '127.0.0.1'
        finally: s.close()
        return IP

    def send_tcp_status(self, message):
        for addr in list(self.receiver_conns.keys()):
            try:
                self.receiver_conns[addr].sendall(f"[STATUS] {message}".encode())
            except:
                del self.receiver_conns[addr]

    def run_server(self):
        try:
            self.tcp_server.bind(('', self.tcp_port))
            self.tcp_server.listen(5)
        except Exception as e:
            print(f"[ERROR] Bind failed: {e}")
            return
        
        print(f"[INFO] Sender Active: {self.get_local_ip()}:{self.tcp_port}")

        while self.program_running:
            try:
                conn, addr = self.tcp_server.accept()
                conn.setblocking(False)
                self.receiver_conns[addr] = conn
                print(f"[CONNECT] Receiver linked: {addr}")
            except: pass

            for addr in list(self.receiver_conns.keys()):
                try:
                    data = self.receiver_conns[addr].recv(1024)
                    if b"START" in data: 
                        self.start_mapping()
                        self.program_running = False 
                except: pass
            time.sleep(0.1)

    def start_mapping(self):
        zed = sl.Camera(); mesh = sl.Mesh(); mapping_exit = False
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        
        if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.send_tcp_status("ZED Open Failed.")
            return

        try:
            self.send_tcp_status("ZED Active. Mapping...")
            tracking_params = sl.PositionalTrackingParameters()
            tracking_params.set_gravity_as_origin = True
            zed.enable_positional_tracking(tracking_params)

            mapping_params = sl.SpatialMappingParameters()
            mapping_params.resolution_meter = 0.10
            mapping_params.map_type = sl.SPATIAL_MAP_TYPE.MESH
            zed.enable_spatial_mapping(mapping_params)

            vox_dim = int(30.0 / 0.10)
            voxel_grid = np.zeros((vox_dim, vox_dim), dtype=np.uint8)
            start_unix = time.time(); last_stream_time = 0
            
            runtime_params = sl.RuntimeParameters()
            point_cloud = sl.Mat(); camera_pose = sl.Pose()

            while not mapping_exit:
                for addr in list(self.receiver_conns.keys()):
                    try:
                        data = self.receiver_conns[addr].recv(1024)
                        if b"EXIT" in data or not data: mapping_exit = True
                    except: pass

                if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                    elapsed = time.time() - start_unix
                    zed.get_position(camera_pose, sl.REFERENCE_FRAME.WORLD)
                    cam_pos = camera_pose.get_translation(sl.Translation()).get()
                    cam_rot = np.degrees(camera_pose.get_euler_angles())
                    pose_matrix = camera_pose.pose_data(sl.Transform()).m

                    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ, sl.MEM.CPU, sl.Resolution(640, 360))
                    pts = point_cloud.get_data()[::4, ::4, :3].reshape(-1, 3) 
                    valid_pts = pts[np.isfinite(pts[:, 0])]
                    
                    new_nodes = []
                    if valid_pts.size > 0:
                        dist_sq = np.sum(valid_pts**2, axis=1)
                        if (in_range_pts := valid_pts[dist_sq <= 25.0]).size > 0:
                            world_pts = np.c_[in_range_pts, np.ones(in_range_pts.shape[0])] @ pose_matrix.T 
                            mask = (world_pts[:,0] >= -10.0) & (world_pts[:,0] <= 20.0) & \
                                   (world_pts[:,2] >= -23.0) & (world_pts[:,2] <= 7.0)
                            if (final_pts := world_pts[mask]).size > 0:
                                c = ((final_pts[:, 0] - (-10.0)) / 0.10).astype(int)
                                r = ((final_pts[:, 2] - (-23.0)) / 0.10).astype(int)
                                for rr, cc in zip(np.clip(r, 0, vox_dim-1), np.clip(c, 0, vox_dim-1)):
                                    if voxel_grid[rr, cc] == 0:
                                        voxel_grid[rr, cc] = 1
                                        new_nodes.append((int(rr), int(cc)))

                    if (time.time() - last_stream_time > 0.05):
                        packet = pickle.dumps({
                            'pos': cam_pos.tolist(), 'rot': cam_rot.tolist(), 
                            'time': elapsed, 'nodes': new_nodes
                        })
                        for addr in list(self.receiver_conns.keys()):
                            try: self.udp_sock.sendto(packet, (addr[0], self.udp_port))
                            except: pass
                        last_stream_time = time.time()

            self.send_tcp_status("Finalizing PLY Mesh...")
            zed.extract_whole_spatial_map(mesh)
            mesh.filter(sl.MeshFilterParameters())
            ts = datetime.now().strftime('%H%M_%d_%m')
            m_name = f"terrain_{ts}.ply"
            m_path = os.path.expanduser(f"~/Desktop/mapping/{m_name}")
            os.makedirs(os.path.dirname(m_path), exist_ok=True)
            
            if mesh.save(m_path, sl.MESH_FILE_FORMAT.PLY):
                f_size = os.path.getsize(m_path) / (1024*1024)
                self.send_tcp_status(f"FILE_INFO|{m_name}|{f_size:.2f} MB")
        
        finally:
            # Shutdown order prevents Argus Client Socket errors
            zed.disable_spatial_mapping()
            zed.disable_positional_tracking()
            zed.close()

if __name__ == "__main__":
    sender = CoverageSender()
    sender.run_server()
