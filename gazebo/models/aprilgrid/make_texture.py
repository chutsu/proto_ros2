#!/usr/bin/env python3
import cv2
import numpy as np


grid = cv2.imread("aprilgrid.png")
h, w, c = grid.shape

texture_shape = np.array([h * 4, w * 3, 3])
texture = np.zeros(texture_shape)

top = 255 * np.ones((h, w, 3))
bottom = 255 * np.ones((h, w, 3))
left = 255 * np.ones((h, w, 3))
right = 255 * np.ones((h, w, 3))

texture[0:h, w:w*2, :] = top
texture[h:h*2, 0:w, :] = left
texture[h:h*2, w:w*2, :] = grid
texture[h:h*2, w*2:w*3, :] = right
texture[h*2:h*3, w:w*2, :] = bottom
texture[h*3:, w:w*2, :] = bottom

# scale_percent = 10 # percent of original size
# width = int(texture.shape[1] * scale_percent / 100)
# height = int(texture.shape[0] * scale_percent / 100)
# dim = (width, height)
# viz = cv2.resize(texture, dim)
# cv2.imshow("texture", viz)
# cv2.waitKey(0)

cv2.imwrite("./texture-test.jpg", texture)
