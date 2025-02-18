import cv2

def list_cameras():
    print("Scanning for available cameras...")
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"✅ Camera detected at index: {i}")
            cap.release()
        else:
            print(f"❌ No camera at index: {i}")

#list_cameras()
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ Failed to open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame")
        break

    cv2.imshow("Camera Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()