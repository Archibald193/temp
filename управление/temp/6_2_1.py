import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

# ==== ВСТАВЬТЕ СЮДА ПУТЬ К ВАШЕМУ ФАЙЛУ ====
file_path = r"APK.png"  # измените на свой путь

# Загрузка изображения
if not os.path.exists(file_path):  # исправлено: file_path вместо path
    print("Файл не найден. Создаю тестовое изображение.")
    img = np.ones((400, 600, 3), dtype=np.uint8) * 240
    cv2.rectangle(img, (150, 150), (250, 250), (0, 0, 0), -1)
    cv2.rectangle(img, (300, 100), (400, 200), (0, 0, 0), -1)
    cv2.circle(img, (500, 300), 50, (0, 0, 0), -1)
else:
    img = cv2.imread(file_path)  # исправлено: file_path вместо path

# Конвертация в grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray_float = np.float32(gray)

# Параметры Харриса
block_size = 2      # размер окна
ksize = 3           # размер ядра Собеля
k = 0.04            # параметр Харриса

# Детектор Харриса
harris = cv2.cornerHarris(gray_float, block_size, ksize, k)
harris = cv2.dilate(harris, None)  # увеличиваем точки для видимости

# Порог для отбора углов
threshold = 0.1
corners = harris > threshold * harris.max()

# Результат
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_with_corners = img_rgb.copy()
img_with_corners[corners] = [255, 0, 0]  # красные точки

# Отображение
plt.figure(figsize=(12, 5))
plt.subplot(121); plt.imshow(img_rgb); plt.title('Оригинал'); plt.axis('off')
plt.subplot(122); plt.imshow(img_with_corners); plt.title(f'Углы Харриса (найдено: {np.sum(corners)})'); plt.axis('off')
plt.tight_layout()
plt.show()
