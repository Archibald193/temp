import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('APK.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# FAST
fast = cv2.FastFeatureDetector_create(threshold=50)
kp_fast = fast.detect(gray, None)

# Harris
gray_float = np.float32(gray)
harris = cv2.cornerHarris(gray_float, 2, 3, 0.04)
harris = cv2.dilate(harris, None)
corners_harris = harris > 0.01 * harris.max()

# Визуализация
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Harris на оригинале
img_harris = img_rgb.copy()
img_harris[corners_harris] = [255, 0, 0]

# FAST на оригинале
img_fast = cv2.drawKeypoints(img_rgb, kp_fast, None, color=(0, 255, 0))

plt.figure(figsize=(15, 5))
plt.subplot(131); plt.imshow(img_rgb); plt.title('Оригинал'); plt.axis('off')
plt.subplot(132); plt.imshow(img_harris); plt.title(f'Harris ({np.sum(corners_harris)} углов)'); plt.axis('off')
plt.subplot(133); plt.imshow(img_fast); plt.title(f'FAST ({len(kp_fast)} углов)'); plt.axis('off')
plt.tight_layout()
plt.show()
