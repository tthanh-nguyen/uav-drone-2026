
import time

import cv2

def open_camera(camera_index=2, width=640, height=480, fps=60):
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
            raise RuntimeError("Error: Could not open webcam.")
    # fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    # cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    
    return cap


if __name__ == "__main__":
    cam = open_camera() # 0= right cam # 4 leftcam
    counter = 0
    while True:
        ret, frame = cam.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break
        
        cv2.imshow('Webcam', frame)

        
        # 1. Gọi waitKey ĐÚNG 1 LẦN và lưu vào biến 'key'
        key = cv2.waitKey(1) & 0xFF

        # 2. So sánh biến 'key' với các phím bạn muốn
        if key == ord('s'):
            cv2.imwrite(f"data/{counter:06d}.jpg", frame)
            counter += 1
            print(f"Captured frame {counter:06d}")
            time.sleep(0.1)

        if key == ord('q'):
            break

    
    cam.release()
    cv2.destroyAllWindows()