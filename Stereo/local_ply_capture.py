import pyzed.sl as sl
import cv2
import time

def main():
    zed = sl.Camera()

    # Init for local camera
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720   # Resolution: HD1080
    init_params.camera_fps = 60                            # Target 60 FPS
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] Could not open ZED camera")
        exit(1)

    # Enable positional tracking
    tracking_params = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(tracking_params)

    # Enable spatial mapping
    # Spatial mapping
    mapping_params = sl.SpatialMappingParameters()
    mapping_params.resolution_meter = 0.01   # 1 cm detail
    mapping_params.range_meter = 5.0         # map up to 5 meters away
    mapping_params.map_type = sl.SPATIAL_MAP_TYPE.MESH
    mapping_params.save_texture = False
    
    zed.enable_spatial_mapping(mapping_params)


    zed.enable_spatial_mapping(mapping_params)

    runtime_params = sl.RuntimeParameters()
    mesh = sl.Mesh()
    image = sl.Mat()

    print("[INFO] Running spatial mapping for 2 minutes at 60 FPS...")
    start_time = time.time()
    duration = 60  # 2 minutes in seconds

    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Retrieve and show left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            cv2.imshow("ZED Camera Feed", image.get_data())

            # Update mesh asynchronously
            if zed.get_spatial_map_request_status_async() != sl.ERROR_CODE.SUCCESS:
                zed.request_spatial_map_async()
            if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_spatial_map_async(mesh)

        # Quit window with Q
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Stop after 2 minutes
        if time.time() - start_time > duration:
            break

    # Save final map
    print("[INFO] Extracting final spatial map...")
    zed.extract_whole_spatial_map(mesh)
    mesh.filter(sl.MeshFilterParameters())
    filename = "mesh_output.ply"
    mesh.save(filename, sl.MESH_FILE_FORMAT.PLY)
    print(f"[SUCCESS] Mesh saved as {filename}")

    # Cleanup
    cv2.destroyAllWindows()
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()

if __name__ == "__main__":
    main()
