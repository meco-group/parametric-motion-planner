import os
from PIL import Image
import re

STEP_SIZE = 1
DT = 0.01

def make_animation(folder_path, appendix):
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
    output_path = f'../post-process/figures/animation_{appendix}.gif'

    # Use the first image as the base and append the rest of the frames to create the animation
    images[0].save(output_path, save_all=True, append_images=images[1:], duration=STEP_SIZE*DT*1000*5, loop=0)

    print(f"Animation saved as {output_path}")

def make_all_animations(folder_path):
    subfolders = ['traj_frames', 'vel_frames', 'accel_frames']
    animation_appendix = ['traj', 'vel', 'accel']

    for subfolder in subfolders:
        make_animation(os.path.join(folder_path, subfolder), animation_appendix[subfolders.index(subfolder)])

# Folder where PNGs are stored
folder_path = '../post-process/figures/animation/'
make_all_animations(folder_path)
