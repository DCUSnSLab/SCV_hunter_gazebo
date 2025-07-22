#!/usr/bin/env python3
import numpy as np
from PIL import Image

# Create empty 400x400 map (white = free space)
map_array = np.ones((400, 400), dtype=np.uint8) * 254  # 254 = free space in grayscale

# Convert to PIL Image and save as PGM
img = Image.fromarray(map_array, mode='L')
img.save('empty_world.pgm')

print("Empty map created: empty_world.pgm")