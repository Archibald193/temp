import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

# ==== ВСТАВЬТЕ СЮДА ПУТЬ К ВАШЕМУ ФАЙЛУ ====
file_path = r"n_k.png"  # измените на свой путь

# Загрузка изображения
if not os.path.exists(file_path):  # исправлено: file_path вместо path
    print("Файл не найден. Создаю тестовое изображение.")
    img = np.ones((400, 600, 3), dtype=np.uint8) * 240
    cv2.rectangle(img, (150, 150), (250, 250), (0, 0, 0), -1)
    cv2.rectangle(img, (300, 100), (400, 200), (0, 0, 0), -1)
    cv2.circle(img, (500, 300), 50, (0, 0, 0), -1)
else:
    img = cv2.imread(file_path)  # исправлено: file_path вместо path

# Конвертация в оттенки серого
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Создание ORB детектора
orb = cv2.ORB_create(nfeatures=5000)

# Поиск ключевых точек и вычисление дескрипторов
kp, des = orb.detectAndCompute(gray, None)

# Визуализация ключевых точек
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_kp = cv2.drawKeypoints(img_rgb, kp, None, color=(255, 0, 0), flags=0)

# Вывод информации
print(f"Найдено ключевых точек: {len(kp)}")
print(f"Размер дескриптора: {des.shape if des is not None else 'N/A'}")
if des is not None:
    print(f"Тип: бинарный (первые 64 бита: {des[0][:8]})")
    print(f"Размер в байтах: {des.nbytes}")

# Отображение
plt.figure(figsize=(12, 5))
plt.subplot(121); plt.imshow(img_rgb); plt.title('Оригинал'); plt.axis('off')
plt.subplot(122); plt.imshow(img_kp); plt.title(f'ORB (найдено {len(kp)} точек)'); plt.axis('off')
plt.tight_layout()
plt.show()
