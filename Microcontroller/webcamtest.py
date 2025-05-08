import cv2
import numpy as np
import tensorflow as tf
import serial
import time

# loading model
interpreter = tf.lite.Interpreter(model_path="C:/Users/micha/Downloads/sign_model_uint8_3.tflite")
interpreter.allocate_tensors()

#Model input/output config
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
IMG_SIZE = input_details[0]['shape'][1]
CLASS_NAMES = [str(i) for i in range(10)]  #digits 0–9

#Serial config
ser = serial.Serial('COM8', 9600)
time.sleep(2)  #give STM32 time to receive

#inference timing + confidence threshold
last_send_time = 0
send_interval = .5  #seconds
CONFIDENCE_THRESHOLD = 0.50

#webcam setup
cap = cv2.VideoCapture(0)
print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    #center-crop to square
    h, w = frame.shape[:2]
    min_edge = min(h, w)
    roi = frame[h//2 - min_edge//2 : h//2 + min_edge//2,
                w//2 - min_edge//2 : w//2 + min_edge//2]

    #preprocessing grayscale + resize to model input
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, (IMG_SIZE, IMG_SIZE))
    input_img = resized.reshape(1, IMG_SIZE, IMG_SIZE, 1).astype(np.uint8)

    #run inference
    interpreter.set_tensor(input_details[0]['index'], input_img)
    interpreter.invoke()
    output = interpreter.get_tensor(output_details[0]['index'])

    #get prediction and confidence
    pred_class = int(np.argmax(output))
    confidence = float(np.max(output)) / 255.0

    #send to STM32 only if high confidence and 1s has passed
    current_time = time.time()
    if confidence >= CONFIDENCE_THRESHOLD and (current_time - last_send_time >= send_interval):
        ser.write(f"{pred_class}\n".encode())
        ser.flush()
        last_send_time = current_time
        print(f"Sent {pred_class} to STM32 (confidence: {confidence:.2f})")

    #displaying result on webcam
    label = f"{CLASS_NAMES[pred_class]} ({confidence:.2f})"
    cv2.putText(frame, label, (10, 35), cv2.FONT_HERSHEY_SIMPLEX,
                1.2, (0, 255, 0) if confidence >= CONFIDENCE_THRESHOLD else (0, 0, 255), 2)

    cv2.imshow("Live Inference", frame)

    #model input
    zoomed = cv2.resize(resized, (IMG_SIZE*8, IMG_SIZE*8), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("Model Input (64x64)", zoomed)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#clean
cap.release()
cv2.destroyAllWindows()
ser.close()
