import os
from PIL import Image
import re

# Folder where PNGs are stored
folder_path = '../post-process/figures/animation/'

# Regex to capture the numeric part of the filename
def extract_number(filename):
    match = re.search(r'_(\d+)\.png', filename)
    return int(match.group(1)) if match else -1

# Get all PNG files, extract their numeric suffix, and sort by the number
png_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.png')], key=extract_number)

# List to store images
images = []

# Load all images into the list
for file_name in png_files:
    file_path = os.path.join(folder_path, file_name)
    img = Image.open(file_path)
    images.append(img)

# Save the images as an animated GIF
output_path = '../post-process/figures/animation.gif'

# Use the first image as the base and append the rest of the frames to create the animation
images[0].save(output_path, save_all=True, append_images=images[1:], duration=50, loop=0)

print(f"Animation saved as {output_path}")
