import random

import cv2
import bm3d
import numpy as np

image = cv2.imread('bread.jpg')


def get_line(p1, p2):
    m = (p2[1] - p1[1]) / (p2[0] - p1[0])
    c = p1[1] - m * p1[0]
    return m, c


denoised_image = cv2.bilateralFilter(image, 5, 100, 100)
gray = cv2.cvtColor(denoised_image, cv2.COLOR_RGB2GRAY)
ret, thresh = cv2.threshold(gray, 170, 255, 1)
edges = cv2.Canny(thresh, 0, 255)
dilated = cv2.dilate(edges, (10, 10), iterations=10)
eroded = cv2.erode(dilated, (3, 3), iterations=4)

contours, hierarchy = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contour = max(contours, key=cv2.contourArea)

hull_list = []

hull = cv2.convexHull(contour)
hull_list.append(hull)

rect = cv2.minAreaRect(contour)
box = cv2.boxPoints(rect)
box = np.intp(box)

drawing = np.zeros((edges.shape[0], edges.shape[1], 3), dtype=np.uint8)
cv2.drawContours(image, [box], 0, (0, 0, 255), 2)

cv2.drawContours(image, hull_list, 0, (0, 0, 255), 2)


m, c = get_line(box[1], box[2])

print(f"Line Solution is y = {m}x + {c}")

point1 = (box[1] + box[2]) * 1 / 2
point2 = (box[2] + box[3]) * 1 / 2
print(point1)
point1 = (np.rint(point1)).astype(int)
point2 = (np.rint(point2)).astype(int)
image = cv2.circle(image, point1, 5, (255,0,0), -1)
image = cv2.circle(image, point2, 5, (255,0,0), -1)
cv2.imshow('bread3', image)

print(box)
# (this is necessary to avoid Python kernel form crashing)
cv2.waitKey(0)

# closing all open windows
cv2.destroyAllWindows()
