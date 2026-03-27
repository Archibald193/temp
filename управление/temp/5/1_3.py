import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

# Загрузка изображения
path = "APK.png"
if not os.path.exists(path):
    img = np.zeros((300,300), dtype=np.uint8)
    cv2.rectangle(img, (80,80), (150,150), 255, -1)
    cv2.circle(img, (220,150), 40, 255, -1)
else:
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    img = cv2.resize(img, (300,300))

# Функция применения фильтра
def apply_filter(img, mask):
    f = np.fft.fft2(img)
    fshift = np.fft.fftshift(f) * mask
    return np.abs(np.fft.ifft2(np.fft.ifftshift(fshift))).astype(np.uint8)

# Создание масок
rows, cols = img.shape
crow, ccol = rows//2, cols//2
Y, X = np.ogrid[:rows, :cols]
dist = np.sqrt((X - ccol)**2 + (Y - crow)**2)

# Исследование радиусов
radii = [10, 30, 50, 70, 90]
plt.figure(figsize=(15, 10))

for i, r in enumerate(radii):
    # Маски
    ideal = (dist <= r).astype(float)
    gauss = np.exp(-(dist**2)/(2*r**2))
    butter = 1/(1 + (dist/r)**4)
    
    # Применение
    img_i = apply_filter(img, ideal)
    img_g = apply_filter(img, gauss)
    img_b = apply_filter(img, butter)
    
    plt.subplot(3, 4, i*1+1)
    plt.imshow(gauss, cmap='gray')
    plt.title(f'Гаусс r={r}')
    plt.axis('off')
    plt.subplot(3, 4, i*1+6)
    plt.imshow(img_g, cmap='gray')
    plt.axis('off')

plt.tight_layout()
plt.show()
