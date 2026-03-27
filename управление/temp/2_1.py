import cv2
import numpy as np
import matplotlib.pyplot as plt

# Загружаем изображение (можно заменить на своё)
# Создаём простое тестовое изображение, если файла нет
def create_test_image():
    img = np.zeros((200, 200), dtype=np.uint8)
    img[50:150, 50:150] = 200
    img = cv2.GaussianBlur(img, (15,15), 0)
    return img

# Загружаем или создаём изображение
try:
    image = cv2.imread('APK.png', cv2.IMREAD_GRAYSCALE)
    if image is None:
        image = create_test_image()
except:
    image = create_test_image()

# Добавляем гауссовский шум
noise = np.random.normal(0, 25, image.shape).astype(np.float32)
noisy_gauss = image.astype(np.float32) + noise
noisy_gauss = np.clip(noisy_gauss, 0, 255).astype(np.uint8)

# Показываем результат
plt.figure(figsize=(10,5))
plt.subplot(121); plt.imshow(image, cmap='gray'); plt.title('Оригинал'); plt.axis('off')
plt.subplot(122); plt.imshow(noisy_gauss, cmap='gray'); plt.title('Гауссовский шум'); plt.axis('off')
plt.show()
# Копируем изображение
noisy_sp = image.copy()

# Случайные индексы для шума
h, w = image.shape
num_salt = int(0.02 * h * w)  # 2% соли
num_pepper = int(0.02 * h * w)  # 2% перца

# Добавляем соль (белые точки)
coords = [np.random.randint(0, h-1, num_salt), np.random.randint(0, w-1, num_salt)]
noisy_sp[coords[0], coords[1]] = 255

# Добавляем перец (чёрные точки)
coords = [np.random.randint(0, h-1, num_pepper), np.random.randint(0, w-1, num_pepper)]
noisy_sp[coords[0], coords[1]] = 0

# Показываем результат
plt.figure(figsize=(10,5))
plt.subplot(121); plt.imshow(image, cmap='gray'); plt.title('Оригинал'); plt.axis('off')
plt.subplot(122); plt.imshow(noisy_sp, cmap='gray'); plt.title('Соль-перец'); plt.axis('off')
plt.show()
# Мультипликативный шум
noise = 1 + np.random.normal(0, 0.1, image.shape)
noisy_speckle = image.astype(np.float32) * noise.astype(np.float32)
noisy_speckle = np.clip(noisy_speckle, 0, 255).astype(np.uint8)

# Показываем результат
plt.figure(figsize=(10,5))
plt.subplot(121); plt.imshow(image, cmap='gray'); plt.title('Оригинал'); plt.axis('off')
plt.subplot(122); plt.imshow(noisy_speckle, cmap='gray'); plt.title('Спекл-шум'); plt.axis('off')
plt.show()
# Масштабируем для пуассоновского распределения
scaled = image.astype(np.float32) * 0.1
noisy_poisson = np.random.poisson(scaled).astype(np.float32) * 10
noisy_poisson = np.clip(noisy_poisson, 0, 255).astype(np.uint8)

# Показываем результат
plt.figure(figsize=(10,5))
plt.subplot(121); plt.imshow(image, cmap='gray'); plt.title('Оригинал'); plt.axis('off')
plt.subplot(122); plt.imshow(noisy_poisson, cmap='gray'); plt.title('Пуассоновский шум'); plt.axis('off')
plt.show()
