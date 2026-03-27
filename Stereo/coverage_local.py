import pyzed.sl as sl
import numpy as np
import cv2
import time
import os
import sys
from datetime import datetime

# CONFIGURATION
GRID_SIZE_M = 30.0 # Competition Pitch Size
VOXEL_RES_M = 0.10  # 10cm Resolution
MAP_DISPLAY_SIZE = 640 


### Code in case the start point (origin) is not centre of the pitch

# Start offset from center: (-5, 8)
OFFSET_X, OFFSET_Z = 5.0, -8.0
X_MIN, X_MAX = OFFSET_X - (GRID_SIZE_M / 2), OFFSET_X + (GRID_SIZE_M / 2)
Z_MIN, Z_MAX = OFFSET_Z - (GRID_SIZE_M / 2), OFFSET_Z + (GRID_SIZE_M / 2)

request_exit = False
path_history = [] # To store (px, pz) tuples

def on_mouse_click(event, x, y, flags, param):
    global request_exit
    if event == cv2.EVENT_LBUTTONDOWN:
        if x >= 640 and 640 <= y <= 720:
            request_exit = True

def main():
    global request_exit
    zed = sl.Camera()
    mesh = sl.Mesh()
    
    try:
        # Hardware init
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL 
        
        if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print("ZED2i Open Failed.")
            return

        # Tracking init (Magnetometer Disabled)
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.set_gravity_as_origin = True 
        zed.enable_positional_tracking(tracking_params)

        # Spatial Mapping Setup
        mapping_params = sl.SpatialMappingParameters()
        mapping_params.resolution_meter = VOXEL_RES_M 
        mapping_params.range_meter = 5.0          
        mapping_params.map_type = sl.SPATIAL_MAP_TYPE.MESH
        zed.enable_spatial_mapping(mapping_params)

        # Coverage Grid
        vox_dim = int(GRID_SIZE_M / VOXEL_RES_M)
        voxel_grid = np.zeros((vox_dim, vox_dim), dtype=np.uint8)
        
        start_time_dt = datetime.now()
        start_unix = time.time()

        cv2.namedWindow("Mission Dashboard")
        cv2.setMouseCallback("Mission Dashboard", on_mouse_click)

        runtime_params = sl.RuntimeParameters()
        left_image = sl.Mat()
        point_cloud = sl.Mat()
        camera_pose = sl.Pose()

        while not request_exit:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                elapsed_total = time.time() - start_unix
                
                # Retrieve Pose & Orientation
                zed.get_position(camera_pose, sl.REFERENCE_FRAME.WORLD)
                cam_pos = camera_pose.get_translation(sl.Translation()).get()
                cam_rot = np.degrees(camera_pose.get_euler_angles()) 
                pose_matrix = camera_pose.pose_data(sl.Transform()).m
                
                dist_from_start = np.sqrt(cam_pos[0]**2 + cam_pos[1]**2 + cam_pos[2]**2)

                # Mapping Logic
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ, sl.MEM.CPU, sl.Resolution(640, 360))
                pts = point_cloud.get_data()[::2, ::2, :3].reshape(-1, 3)
                valid_pts = pts[np.isfinite(pts[:, 0])]
                
                if valid_pts.size > 0:
                    dist_sq = np.sum(valid_pts**2, axis=1)
                    in_range_pts = valid_pts[dist_sq <= 25.0]
                    
                    if in_range_pts.size > 0:
                        world_pts = np.c_[in_range_pts, np.ones(in_range_pts.shape[0])] @ pose_matrix.T 
                        mask = (world_pts[:,0] >= X_MIN) & (world_pts[:,0] <= X_MAX) & \
                               (world_pts[:,2] >= Z_MIN) & (world_pts[:,2] <= Z_MAX)
                        final_pts = world_pts[mask]
                        
                        if final_pts.size > 0:
                            c = ((final_pts[:, 0] - X_MIN) / VOXEL_RES_M).astype(int)
                            r = ((final_pts[:, 2] - Z_MIN) / VOXEL_RES_M).astype(int)
                            voxel_grid[np.clip(r, 0, vox_dim-1), np.clip(c, 0, vox_dim-1)] = 1

                # UI RENDERING 
                zed.retrieve_image(left_image, sl.VIEW.LEFT)
                feed = cv2.resize(left_image.get_data()[:,:,:3], (640, 360))
                
                mini_map = cv2.resize(voxel_grid * 255, (640, 360), interpolation=cv2.INTER_NEAREST)
                mini_map_col = cv2.applyColorMap(mini_map, cv2.COLORMAP_JET)
                mini_map_col[mini_map == 0] = 35 

                def to_px(wx, wz):
                    px = int(((wx - X_MIN) / GRID_SIZE_M) * 640)
                    pz = int(((wz - Z_MIN) / GRID_SIZE_M) * 360)
                    return px, pz

                # Start Point (Red Cross)
                sx, sz = to_px(0, 0)
                cv2.drawMarker(mini_map_col, (sx, sz), (0, 0, 255), cv2.MARKER_CROSS, 15, 2)
                
                # Path History Logic
                cx, cz = to_px(cam_pos[0], cam_pos[2])
                path_history.append((cx, cz))
                if len(path_history) > 1:
                    # Draw path as a series of connected lines
                    for i in range(1, len(path_history)):
                        cv2.line(mini_map_col, path_history[i-1], path_history[i], (200, 200, 200), 1)

                # Camera Directional Arrow (Direction Correction)
                yaw_rad = np.radians(cam_rot[1]) 
                arrow_len = 25
                ex = int(cx - arrow_len * np.sin(yaw_rad))
                ez = int(cz - arrow_len * np.cos(yaw_rad))
                
                cv2.arrowedLine(mini_map_col, (cx, cz), (ex, ez), (255, 255, 255), 3, tipLength=0.3)
                cv2.circle(mini_map_col, (cx, cz), 5, (0, 255, 0), -1)
                
                # Sidebar Telemetry
                sidebar = np.zeros((720, 350, 3), dtype=np.uint8)
                put = lambda txt, y, col=(255,255,255), sz=0.85: cv2.putText(sidebar, txt, (20, y), 1, sz, col, 1)
                
                mins, secs = divmod(int(elapsed_total), 60)
                put(f"TIME: {mins:02d}:{secs:02d}", 40, (0, 255, 255), 1.3)
                put(f"NODES: {np.sum(voxel_grid)}", 75, (0, 255, 0), 1.1)
                put(f"MAG STATUS: {not (tracking_params.set_gravity_as_origin)}", 105, (0, 150, 255), 0.9)
                put(f"DIST START: {dist_from_start:.2f}m", 135, (255, 200, 0), 1.1)
                
                put("START RELATIVE (X,Y,Z):", 185, (255, 255, 0), 0.95)
                put(f" X: {cam_pos[0]:+.2f}m", 215)
                put(f" Y: {cam_pos[1]:+.2f}m", 245)
                put(f" Z: {cam_pos[2]:+.2f}m", 275)

                put("ORIENTATION (DEG):", 325, (255, 255, 0), 0.95)
                put(f" YAW:   {cam_rot[1]:+.1f}", 355)
                put(f" PITCH: {cam_rot[2]:+.1f}", 385)
                put(f" ROLL:  {cam_rot[0]:+.1f}", 415)
                
                cv2.rectangle(sidebar, (20, 640), (330, 700), (0, 0, 180), -1)
                cv2.putText(sidebar, "SAVE & EXIT", (85, 680), 1, 1.4, (255, 255, 255), 2)

                visuals = np.vstack((feed, mini_map_col))
                cv2.imshow("Mission Dashboard", np.hstack((visuals, sidebar)))

            if cv2.waitKey(1) & 0xFF == ord('q'): break

        # Final Shutdown
        print("\n[INFO] Saving High-Resolution Mesh...")
        zed.extract_whole_spatial_map(mesh)
        filter_params = sl.MeshFilterParameters()
        filter_params.set(sl.MESH_FILTER.LOW)
        mesh.filter(filter_params)
        fname = f"terrain_map_{start_time_dt.strftime('%H%M_%d_%m')}.ply"
        mesh.save(fname)
        print(f"FILE SAVED: {fname}")

    except Exception as e:
        print(f"CRITICAL ERROR: {e}")
    finally:
        if zed.is_opened():
            zed.disable_spatial_mapping()
            zed.close()
        cv2.destroyAllWindows()
        sys.exit(0)

if __name__ == "__main__":
    main()
