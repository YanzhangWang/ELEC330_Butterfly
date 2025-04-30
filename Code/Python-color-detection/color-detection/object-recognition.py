import cv2
import numpy as np
import argparse

# Create argument parser to take image path from command line
parser = argparse.ArgumentParser()
parser.add_argument('-i', '--image', required=True, help="Image Path")
args = vars(parser.parse_args())
img_path = args['image']

# Read the image
img = cv2.imread(img_path)
img = cv2.resize(img, (800, 600)) if img is not None else None

if img is None:
    print("Error: Could not read image. Please check the path.")
    exit()

# Create a copy for drawing
output = img.copy()
height, width = img.shape[:2]

# Convert to HSV color space for better color detection
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Define color ranges in HSV format (you can adjust these)
color_ranges = {
    "red": (np.array([0, 100, 100]), np.array([10, 255, 255])),
    "blue": (np.array([100, 100, 100]), np.array([140, 255, 255])),
    "green": (np.array([40, 100, 100]), np.array([80, 255, 255])),
    "yellow": (np.array([20, 100, 100]), np.array([35, 255, 255])),
    "orange": (np.array([10, 100, 100]), np.array([20, 255, 255])),
    "purple": (np.array([140, 50, 50]), np.array([170, 255, 255])),
}

# Detect and label colors
for color_name, (lower, upper) in color_ranges.items():
    # Create mask and refine
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # Find contours
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Process contours of sufficient size
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:  # Minimum area to consider (adjust as needed)
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Draw rectangle
            cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Add text label
            cv2.putText(output, color_name, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            print(f"Detected {color_name} object at position ({x}, {y})")

# Define a mouse callback function to get colors at clicked position (optional)
def get_color(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Get BGR color at clicked point
        b, g, r = img[y, x]
        # Convert to HSV
        pixel = np.uint8([[[b, g, r]]])
        hsv_pixel = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)
        h, s, v = hsv_pixel[0][0]
        print(f"Clicked at position ({x}, {y})")
        print(f"BGR color: ({r}, {g}, {b})")
        print(f"HSV color: ({h}, {s}, {v})")

# Show results
cv2.namedWindow('Color Detection')
cv2.setMouseCallback('Color Detection', get_color)

while True:
    cv2.imshow('Color Detection', output)
    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()