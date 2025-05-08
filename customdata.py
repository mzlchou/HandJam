import cv2
import os

SAVE_DIR = "C:/Users/micha/Desktop/customdata"
IMG_SIZE = 64
count = {str(i): 0 for i in range(10)} #track how many images per class

#folders
for digit in count.keys():
    os.makedirs(os.path.join(SAVE_DIR, digit), exist_ok=True)

cap = cv2.VideoCapture(0)
print("Press number keys (0–9) to save labeled images.")
print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    #center and 64x64
    h, w = frame.shape[:2]
    min_edge = min(h, w)
    roi = frame[h//2 - min_edge//2:h//2 + min_edge//2,
                w//2 - min_edge//2:w//2 + min_edge//2]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, (IMG_SIZE, IMG_SIZE))

    #show live + input preview
    preview = cv2.resize(resized, (IMG_SIZE*8, IMG_SIZE*8), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("Webcam Feed", frame)
    cv2.imshow("Captured (64x64)", preview)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
    elif chr(key) in count.keys():
        label = chr(key)
        img_path = os.path.join(SAVE_DIR, label, f"{count[label]}.png")
        cv2.imwrite(img_path, resized)
        count[label] += 1
        print(f"Saved {img_path}")

cap.release()
cv2.destroyAllWindows()
