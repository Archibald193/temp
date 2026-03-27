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

# Параметры
r = int(input("Радиус (10-30): ") or "20")
k = float(input("Коэффициент усиления (0.5-2): ") or "1.0")

# Создание масок
rows, cols = img.shape
crow, ccol = rows//2, cols//2
Y, X = np.ogrid[:rows, :cols]
dist = np.sqrt((X-ccol)**2 + (Y-crow)**2)

# Фильтры
ideal = (dist > r).astype(float)
gauss = 1 - np.exp(-(dist**2)/(2*r**2))
sharpen = 1 + k * gauss

# Применение
def apply_f(img, mask):
    f = np.fft.fft2(img)
    fshift = np.fft.fftshift(f) * mask
    return np.abs(np.fft.ifft2(np.fft.ifftshift(fshift))).astype(np.uint8)

img_i = apply_f(img, ideal)
img_g = apply_f(img, gauss)
img_s = apply_f(img, sharpen)

# Визуализация
plt.figure(figsize=(15,8))
titles = ['Идеальный', 'Гаусс', 'Усиление', 'Оригинал']
images = [img_i, img_g, img_s, img]

for i in range(4):
    plt.subplot(2,4,i+1)
    plt.imshow([ideal,gauss,sharpen,img][i], cmap='gray')
    plt.title(f'Маска {titles[i]}' if i<3 else titles[i])
    plt.axis('off')
    
    plt.subplot(2,4,i+5)
    plt.imshow(images[i], cmap='gray')
    plt.title(f'Результат {titles[i]}' if i<3 else titles[i])
    plt.axis('off')
plt.tight_layout()
plt.show()

# Исследование радиусов
plt.figure(figsize=(15,4))
for i, rad in enumerate([5,15,25,35]):
    g = 1 - np.exp(-(dist**2)/(2*rad**2))
    res = apply_f(img, g)
    plt.subplot(1,4,i+1)
    plt.imshow(res, cmap='gray')
    plt.title(f'Гаусс r={rad}')
    plt.axis('off')
plt.show()

# Исследование коэффициента k
plt.figure(figsize=(15,4))
for i, k_val in enumerate([0.3,0.7,1.3,1.8]):
    s = 1 + k_val * gauss
    res = apply_f(img, s)
    plt.subplot(1,4,i+1)
    plt.imshow(res, cmap='gray')
    plt.title(f'k={k_val}, r={r}')
    plt.axis('off')
plt.show()

# Сравнение фильтров при разных радиусах
radii_test = [10, 20, 30]
plt.figure(figsize=(15, 10))

for i, rad in enumerate(radii_test):
    ideal_t = (dist > rad).astype(float)
    gauss_t = 1 - np.exp(-(dist**2)/(2*rad**2))
    
    img_ideal_t = apply_f(img, ideal_t)
    img_gauss_t = apply_f(img, gauss_t)
    
    plt.subplot(3, 3, i*3+1)
    plt.imshow(img_ideal_t, cmap='gray')
    plt.title(f'Идеальный r={rad}')
    plt.axis('off')
    
    plt.subplot(3, 3, i*3+2)
    plt.imshow(img_gauss_t, cmap='gray')
    plt.title(f'Гаусс r={rad}')
    plt.axis('off')
    
    plt.subplot(3, 3, i*3+3)
    diff = cv2.absdiff(img_ideal_t, img_gauss_t)
    plt.imshow(diff, cmap='hot')
    plt.title(f'Разница r={rad}')
    plt.axis('off')

plt.tight_layout()
plt.show()

# Полосовой фильтр (выделяет средние частоты)
r_low = 100
r_high = 130
low_mask = np.exp(-(dist**2)/(2*r_low**2))  # низкие частоты
high_mask = 1 - np.exp(-(dist**2)/(2*r_high**2))  # высокие частоты
band_mask = (1 - low_mask) * high_mask  # средние частоты

img_band = apply_f(img, band_mask)
img_band_norm = cv2.normalize(img_band, None, 0, 255, cv2.NORM_MINMAX)
plt.figure(figsize=(12,4))
plt.subplot(131); plt.imshow(band_mask, cmap='gray'); plt.title('Полосовая маска'); plt.axis('off')
plt.subplot(132); plt.imshow(img_band_norm, cmap='gray'); plt.title('Выделенные средние частоты'); plt.axis('off')
plt.subplot(133); plt.imshow(img, cmap='gray'); plt.title('Оригинал'); plt.axis('off')
plt.show()
