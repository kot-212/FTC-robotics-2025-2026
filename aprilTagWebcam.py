import cv2
from pupil_apriltags import Detector
import math

# -------------------------
# REAL WIDTH OF YOUR TAG
REAL_TAG_WIDTH_CM = 5.0
# -------------------------

# Open webcam
cap = cv2.VideoCapture(0)
detector = Detector(families="tag36h11")

# Center tolerance
tolerance = 50

# We will estimate focal length automatically on first detection
focal_length_px = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)

    height, width = gray.shape
    center_x, center_y = width // 2, height // 2

    direction = "no tag"
    display_angle = ""
    display_distance = ""

    for tag in tags:
        corners = tag.corners
        tag_x, tag_y = int(tag.center[0]), int(tag.center[1])

        # Draw tag outline
        c = corners.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(c[i]), tuple(c[(i + 1) % 4]), (0, 255, 0), 2)

        # Draw centers
        cv2.circle(frame, (tag_x, tag_y), 5, (0, 0, 255), -1)
        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

        # Compute rotation angle (top-left -> top-right)
        dx = corners[1][0] - corners[0][0]
        dy = corners[1][1] - corners[0][1]
        angle = (math.degrees(math.atan2(dy, dx)) + 360) % 360
        display_angle = f"Angle: {int(angle)}°"

        # Direction to center
        if tag_x < center_x - tolerance:
            direction = "left"
        elif tag_x > center_x + tolerance:
            direction = "right"
        elif tag_y < center_y - tolerance:
            direction = "up"
        elif tag_y > center_y + tolerance:
            direction = "down"
        else:
            direction = "centered"

        # --- DEPTH ESTIMATION ---

        # Pixel width of the tag = distance between top-left and top-right corners
        pixel_width = math.dist(corners[0], corners[1])

        # Auto-calc focal length when tag first appears
        if focal_length_px is None and pixel_width > 0:
            # Assume first detection is about 30 cm away (good starting estimate)
            assumed_distance_cm = 30
            focal_length_px = (pixel_width * assumed_distance_cm) / REAL_TAG_WIDTH_CM

        # Calculate distance if focal length known
        if focal_length_px is not None:
            distance_cm = (REAL_TAG_WIDTH_CM * focal_length_px) / pixel_width
            display_distance = f"Distance: {distance_cm:.1f} cm"

        # Overlay text
        cv2.putText(frame,
                    f"{direction}  |  {display_angle}  |  {display_distance}",
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2)

    cv2.imshow("AprilTag Angle + Depth", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
