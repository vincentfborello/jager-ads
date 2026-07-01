import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam")
    exit()

print("Webcam opened successfully. Press Q to quit.")

bg_subtractor = cv2.createBackgroundSubtractorMOG2(
    history=800,
    varThreshold=50,
    detectShadows=False
)

kernel = np.ones((5, 5), np.uint8)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    fg_mask = bg_subtractor.apply(frame)
    fg_mask = cv2.erode(fg_mask, kernel, iterations=1)
    fg_mask = cv2.dilate(fg_mask, kernel, iterations=2)

    # Find contours — traces the outlines of all white blobs in the mask
    # RETR_EXTERNAL means only find outer contours, ignore holes inside blobs
    # CHAIN_APPROX_SIMPLE compresses contour points to save memory
    contours, _ = cv2.findContours(
        fg_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    # If no contours found, skip this frame
    if not contours:
        cv2.imshow("ADS Vision", frame)
        cv2.imshow("Cleaned Mask", fg_mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    # Find the single largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)

    # Ignore it if it's too small — still just noise
    if cv2.contourArea(largest_contour) < 1500:
        cv2.imshow("ADS Vision", frame)
        cv2.imshow("Cleaned Mask", fg_mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    # Draw one clean bounding box around just the largest contour
    x, y, w, h = cv2.boundingRect(largest_contour)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Center point of the detected object
    center_x = x + w // 2
    center_y = y + h // 2

    # Red dot at center
    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    print(f"Target at: ({center_x}, {center_y}) | Area: {cv2.contourArea(largest_contour):.0f}px")

    cv2.imshow("ADS Vision", frame)
    cv2.imshow("Cleaned Mask", fg_mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()