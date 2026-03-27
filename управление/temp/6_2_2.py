import cv2
import matplotlib.pyplot as plt

img = cv2.imread('APK.png', cv2.IMREAD_GRAYSCALE)

# Создаем детектор с разными параметрами
fast_default = cv2.FastFeatureDetector_create(threshold=50, nonmaxSuppression=True)
fast_low = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
fast_high = cv2.FastFeatureDetector_create(threshold=100, nonmaxSuppression=True)
fast_no_nms = cv2.FastFeatureDetector_create(threshold=50, nonmaxSuppression=False)

# Детектируем углы
kp_default = fast_default.detect(img, None)
kp_low = fast_low.detect(img, None)
kp_high = fast_high.detect(img, None)
kp_no_nms = fast_no_nms.detect(img, None)

# Визуализация
fig, axes = plt.subplots(2, 2, figsize=(12, 10))

img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

axes[0,0].imshow(cv2.drawKeypoints(img_rgb, kp_default, None, color=(255,0,0)))
axes[0,0].set_title(f'FAST порог=50 (NMS) — {len(kp_default)} углов')
axes[0,0].axis('off')

axes[0,1].imshow(cv2.drawKeypoints(img_rgb, kp_low, None, color=(255,0,0)))
axes[0,1].set_title(f'FAST порог=20 — {len(kp_low)} углов (много шума)')
axes[0,1].axis('off')

axes[1,0].imshow(cv2.drawKeypoints(img_rgb, kp_high, None, color=(255,0,0)))
axes[1,0].set_title(f'FAST порог=100 — {len(kp_high)} углов (только сильные)')
axes[1,0].axis('off')

axes[1,1].imshow(cv2.drawKeypoints(img_rgb, kp_no_nms, None, color=(255,0,0)))
axes[1,1].set_title(f'FAST без NMS — {len(kp_no_nms)} углов (кластеры)')
axes[1,1].axis('off')

plt.tight_layout()
plt.show()
