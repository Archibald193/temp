import cv2
import numpy as np
import matplotlib.pyplot as plt

# Загрузка изображения
img = cv2.imread('n_k.png')  # замените на ваш файл
if img is None:
    # Если файла нет, создаем тестовое изображение
    img = np.ones((400, 600, 3), dtype=np.uint8) * 240
    # Рисуем дома
    cv2.rectangle(img, (100, 200), (250, 350), (180, 180, 180), -1)
    cv2.rectangle(img, (120, 220), (150, 280), (100, 100, 200), -1)
    cv2.rectangle(img, (180, 220), (230, 280), (100, 100, 200), -1)
    cv2.rectangle(img, (300, 150), (450, 350), (160, 160, 160), -1)
    cv2.rectangle(img, (350, 200), (400, 280), (200, 200, 100), -1)
    # Рисуем машины
    cv2.rectangle(img, (150, 320), (250, 350), (200, 0, 0), -1)
    cv2.circle(img, (170, 350), 15, (50, 50, 50), -1)
    cv2.circle(img, (230, 350), 15, (50, 50, 50), -1)
    cv2.rectangle(img, (350, 320), (420, 350), (0, 0, 200), -1)
    cv2.circle(img, (370, 350), 15, (50, 50, 50), -1)
    cv2.circle(img, (400, 350), 15, (50, 50, 50), -1)

# Конвертация в RGB для matplotlib
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Детектор Канни
edges = cv2.Canny(img_gray, 30, 90)

# Наложение контуров на оригинал (красным)
overlay = img_rgb.copy()
overlay[edges > 0] = [255, 0, 0]

# Создание слайда с тремя изображениями
fig, axes = plt.subplots(1, 3, figsize=(15, 5))

# Обычное фото
axes[0].imshow(img_rgb)
axes[0].set_title('Обычное фото', fontsize=14)
axes[0].axis('off')

# Результат Канни
axes[1].imshow(edges, cmap='gray')
axes[1].set_title('Детектор Канни', fontsize=14)
axes[1].axis('off')

# Наложение контуров
axes[2].imshow(overlay)
axes[2].set_title('Контуры поверх фото', fontsize=14)
axes[2].axis('off')

plt.tight_layout()
plt.show()
