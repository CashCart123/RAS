import pyzed.sl as sl

def main():
    zed = sl.Camera()

    # Init camera params (no depth, sender is lightweight)
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"[ERROR] Camera Open: {err}. Exiting.")
        exit(1)

    # Setup streaming
    stream_params = sl.StreamingParameters()
    stream_params.codec = sl.STREAMING_CODEC.H265  
    stream_params.bitrate = 8000                   
    stream_params.port = 30000                     # default port

    err = zed.enable_streaming(stream_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"[ERROR] Enable streaming: {err}. Exiting.")
        zed.close()
        exit(1)

    print("[INFO] Streaming ZED2i feed on 192.168.1.5:30000")
    print("[INFO] Press Ctrl+C to stop.")

    try:
        while True:
            pass  # sender is just broadcasting
    except KeyboardInterrupt:
        print("\n[INFO] Stopping stream...")

    zed.disable_streaming()
    zed.close()
    print("[INFO] ZED sender closed.")

if __name__ == "__main__":
    main()
