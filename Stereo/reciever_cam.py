import pyzed.sl as sl
import cv2

def main():
    # Init ZED in streaming input mode
    init_params = sl.InitParameters()
    init_params.set_from_stream("192.168.1.5", 30000)
    init_params.sdk_verbose = 1

    zed = sl.Camera()
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] Cannot connect to ZED stream.")
        exit(1)

    runtime_params = sl.RuntimeParameters()
    image = sl.Mat()

    print("[INFO] Streaming camera feed. Press 'q' to quit.")

    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)  # Just show left camera
            cv2.imshow("ZED Stream - Left View", image.get_data())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
