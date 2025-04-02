# This code does the following:
# - Initializes a 6x2 inch strip at a resolution of 100 pixels per inch, resulting in a 600x200 pixel image.
# - Generates four ARuco markers using a specific dictionary.
# - Calculates the positions for these markers to be equally spaced on the strip.
# - Draws each ARuco marker onto the strip at the calculated positions.
# - Displays the strip with ARuco tags in a window.
# - Saves the strip as 'aruco_strip.png' in the current directory.
# Adjust the `strip_length_in_pixels`, `strip_height_in_pixels`, and `marker_side_length` values
# Initial version by Chatgpt4

import cv2
import numpy as np

ppi = 75                       # pixels per inch

# Initialize dimensions
strip_length_in_pixels = ppi * 9    # 6-inch strip
strip_height_in_pixels = ppi * 2    # 2-inch height
marker_side_length = ppi # Pixel length of the square ARuco markers

# Create a white strip
strip = 255 * np.ones((strip_height_in_pixels, strip_length_in_pixels), dtype=np.uint8)

# Define ARuco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Calculate spacing for 4 markers to be equally distributed
space_between_markers = (strip_length_in_pixels - 4 * marker_side_length) / 5

# Generate and place 4 ARuco tags
for i in range(4):
    # Generate ARuco tag
    tag_id = i  # Use i as the tag ID for differentiation
    tag_img = cv2.aruco.generateImageMarker(aruco_dict, tag_id, marker_side_length)
    
    # Calculate tag's top-left corner position
    x_position = int((i + 1) * space_between_markers + i * marker_side_length)
    y_position = (strip_height_in_pixels - marker_side_length) // 2
    
    # Place Aruco tag on the strip
    strip[y_position:y_position+marker_side_length, x_position:x_position+marker_side_length] = tag_img

# Display the strip
cv2.imshow('sodacan_strip', strip)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save the strip to a file
cv2.imwrite('sodacan_strip.png', strip)
