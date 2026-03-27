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
    
    # Визуализация
    plt.subplot(5, 4, i*4+1)
    plt.imshow(ideal, cmap='gray')
    plt.title(f'Идеал r={r}')
    plt.axis('off')
    
    plt.subplot(5, 4, i*4+2)
    plt.imshow(img_i, cmap='gray')
    plt.axis('off')
    
    plt.subplot(5, 4, i*4+3)
    plt.imshow(gauss, cmap='gray')
    plt.title(f'Гаусс r={r}')
    plt.axis('off')
    
    plt.subplot(5, 4, i*4+4)
    plt.imshow(img_g, cmap='gray')
    plt.axis('off')

plt.tight_layout()
plt.show()

# Сравнение при r=40
r = 40
ideal = (dist <= r).astype(float)
gauss = np.exp(-(dist**2)/(2*r**2))
butter = 1/(1 + (dist/r)**4)

plt.figure(figsize=(12, 8))
titles = ['Идеальный', 'Гаусс', 'Баттерворт', 'Оригинал']
for i, mask in enumerate([ideal, gauss, butter]):
    plt.subplot(3, 4, i+1)
    plt.imshow(mask, cmap='gray')
    plt.title(titles[i])
    plt.axis('off')
    
    plt.subplot(3, 4, i+5)
    plt.imshow(apply_filter(img, mask), cmap='gray')
    plt.axis('off')

plt.subplot(3, 4, 4) 
plt.imshow(img, cmap='gray')
plt.title('Оригинал')
plt.axis('off')
plt.subplot(3, 4, 9)
diff = cv2.absdiff(img, img_i)
plt.imshow(diff, cmap='hot')
plt.title('Разность (идеал)')
plt.axis('off')
plt.subplot(3, 4, 10)
diff = cv2.absdiff(img, img_g)
plt.imshow(diff, cmap='hot')
plt.title('Разность (гаус)')
plt.axis('off')
plt.subplot(3, 4, 11)
diff = cv2.absdiff(img, img_b)
plt.imshow(diff, cmap='hot')
plt.title('Разность (баттер)')
plt.axis('off')
plt.tight_layout()
plt.show()
