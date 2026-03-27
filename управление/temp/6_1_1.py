import cv2
import numpy as np
import matplotlib.pyplot as plt

# ==== ВСТАВЬТЕ СЮДА ПУТЬ К ВАШЕМУ ФАЙЛУ ====
file_path = r"APK.png"  # измените на свой путь

# Загрузка изображения
img = cv2.imread(file_path)

if img is None:
    print("Ошибка: файл не найден. Проверьте путь.")
    # Создаем тестовое изображение
    img = np.ones((400, 600, 3), dtype=np.uint8) * 240
    cv2.rectangle(img, (150, 200), (350, 380), (180, 180, 180), -1)
    cv2.rectangle(img, (400, 250), (500, 350), (200, 0, 0), -1)
else:
    print(f"Изображение загружено: {file_path}")

# Дальше обработка...
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
sobel = np.sqrt(sobelx**2 + sobely**2)
sobel = np.uint8(np.clip(sobel, 0, 255))

# Отображение
plt.figure(figsize=(12, 4))
plt.subplot(131); plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)); plt.title('Оригинал'); plt.axis('off')
plt.subplot(132); plt.imshow(sobelx, cmap='RdBu', vmin=-150, vmax=150); plt.title('Собель X'); plt.axis('off')
plt.subplot(133); plt.imshow(sobel, cmap='gray'); plt.title('Модуль градиента'); plt.axis('off')
plt.tight_layout()
plt.show()
