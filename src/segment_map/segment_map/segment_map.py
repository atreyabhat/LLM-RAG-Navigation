import cv2
import numpy as np
import yaml

# Paths to the map files
map_image_path = "/home/vishwas/Navigation-LLM/LLM-Navigation/maps/first.pgm"  # Replace with your map image path
map_yaml_path = "/home/vishwas/Navigation-LLM/LLM-Navigation/maps/first.yaml"  # Replace with your map YAML path

# Load the map image (PGM)
map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)

# print size of the map
print(map_image.shape)
exit(0)

# Load map resolution and origin from the YAML file
with open(map_yaml_path, "r") as yaml_file:
    map_metadata = yaml.safe_load(yaml_file)
resolution = map_metadata["resolution"]
origin = map_metadata["origin"]  # [x, y, theta]

print("Map and metadata loaded successfully.")

# Preprocess the map
# Threshold the map to binary (free space = 1, obstacles = 0)
_, binary_map = cv2.threshold(map_image, 254, 255, cv2.THRESH_BINARY)

# Invert the binary map: Free space = 255 (white), Obstacles = 0 (black)
binary_map = cv2.bitwise_not(binary_map)

# Perform room segmentation using connected components
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_map, connectivity=8)

print(f"Number of rooms found: {num_labels - 1}")  # Excluding the background

# Compute Cartesian coordinates for each room's center
room_centers = []

for i, (cx, cy) in enumerate(centroids[1:], start=1):  # Skip the background (label 0)
    # Convert pixel coordinates to Cartesian coordinates
    x = origin[0] + cx * resolution
    y = origin[1] + (binary_map.shape[0] - cy) * resolution  # Invert Y-axis
    room_centers.append((x, y))
    print(f"Room {i}: Center (Cartesian): ({x:.2f}, {y:.2f})")

# Visualize the segmented map and room centers
# Map with labeled rooms
colored_labels = cv2.applyColorMap((labels * 30).astype(np.uint8), cv2.COLORMAP_JET)

# Draw centroids
for i, (cx, cy) in enumerate(centroids[1:], start=1):  # Skip the background
    cv2.circle(colored_labels, (int(cx), int(cy)), 5, (255, 255, 255), -1)
    cv2.putText(colored_labels, f"R{i}", (int(cx), int(cy) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

# Show the result
cv2.imshow("Segmented Map", colored_labels)
cv2.waitKey(0)
cv2.destroyAllWindows()
