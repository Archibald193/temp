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

# ============================================
# 1. ГАУССОВСКИЙ ШУМ + ГАУССОВСКИЙ ФИЛЬТР
# ============================================
print("1. Обработка гауссовского шума...")

# Добавляем гауссовский шум
noise = np.random.normal(0, 25, image.shape).astype(np.float32)
noisy_gauss = image.astype(np.float32) + noise
noisy_gauss = np.clip(noisy_gauss, 0, 255).astype(np.uint8)

# Применяем гауссовский фильтр
filtered_gauss = cv2.medianBlur(noisy_gauss, 5)

# Показываем результат
plt.figure(figsize=(15, 5))
plt.subplot(131); plt.imshow(image, cmap='gray'); plt.title('Оригинал'); plt.axis('off')
plt.subplot(132); plt.imshow(noisy_gauss, cmap='gray'); plt.title('Гауссовский шум'); plt.axis('off')
plt.subplot(133); plt.imshow(filtered_gauss, cmap='gray'); plt.title('После медианного фильтра'); plt.axis('off')
plt.suptitle('Гауссовский шум → Медианный фильтр')
plt.show()

# ============================================
# 2. СОЛЬ-ПЕРЕЦ + ГАУССОВСКИЙ ФИЛЬТР
# ============================================
print("2. Обработка импульсного шума (соль-перец)...")

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

# Применяем гауссовский фильтр
filtered_sp = cv2.medianBlur(noisy_sp, 5)

# Показываем результат
plt.figure(figsize=(15, 5))
plt.subplot(131); plt.imshow(image, cmap='gray'); plt.title('Оригинал'); plt.axis('off')
plt.subplot(132); plt.imshow(noisy_sp, cmap='gray'); plt.title('Соль-перец'); plt.axis('off')
plt.subplot(133); plt.imshow(filtered_sp, cmap='gray'); plt.title('После медианного фильтра'); plt.axis('off')
plt.suptitle('Импульсный шум (соль-перец) → Медианный фильтр')
plt.show()

# ============================================
# 3. СПЕКЛ-ШУМ + ГАУССОВСКИЙ ФИЛЬТР
# ============================================
print("3. Обработка спекл-шума...")

# Мультипликативный шум
noise = 1 + np.random.normal(0, 0.1, image.shape)
noisy_speckle = image.astype(np.float32) * noise.astype(np.float32)
noisy_speckle = np.clip(noisy_speckle, 0, 255).astype(np.uint8)

# Применяем гауссовский фильтр
filtered_speckle = cv2.medianBlur(noisy_speckle, 5)

# Показываем результат
plt.figure(figsize=(15, 5))
plt.subplot(131); plt.imshow(image, cmap='gray'); plt.title('Оригинал'); plt.axis('off')
plt.subplot(132); plt.imshow(noisy_speckle, cmap='gray'); plt.title('Спекл-шум'); plt.axis('off')
plt.subplot(133); plt.imshow(filtered_speckle, cmap='gray'); plt.title('После медианного фильтра'); plt.axis('off')
plt.suptitle('Спекл-шум → Медианный фильтр')
plt.show()

# ============================================
# 4. ПУАССОНОВСКИЙ ШУМ + ГАУССОВСКИЙ ФИЛЬТР
# ============================================
print("4. Обработка пуассоновского шума...")

# Масштабируем для пуассоновского распределения
scaled = image.astype(np.float32) * 0.1
noisy_poisson = np.random.poisson(scaled).astype(np.float32) * 10
noisy_poisson = np.clip(noisy_poisson, 0, 255).astype(np.uint8)

# Применяем гауссовский фильтр
filtered_poisson = cv2.medianBlur(noisy_poisson, 5)

# Показываем результат
plt.figure(figsize=(15, 5))
plt.subplot(131); plt.imshow(image, cmap='gray'); plt.title('Оригинал'); plt.axis('off')
plt.subplot(132); plt.imshow(noisy_poisson, cmap='gray'); plt.title('Пуассоновский шум'); plt.axis('off')
plt.subplot(133); plt.imshow(filtered_poisson, cmap='gray'); plt.title('После медианного фильтра'); plt.axis('off')
plt.suptitle('Пуассоновский шум → Медианный фильтр')
plt.show()

# ============================================
# 5. ОЦЕНКА КАЧЕСТВА (PSNR)
# ============================================
print("\n" + "="*50)
print("ОЦЕНКА КАЧЕСТВА ФИЛЬТРАЦИИ (PSNR)")
print("="*50)

def calculate_psnr(original, processed):
    """Вычисление PSNR"""
    mse = np.mean((original.astype(np.float32) - processed.astype(np.float32))**2)
    if mse == 0:
        return float('inf')
    psnr = 10 * np.log10(255**2 / mse)
    return psnr

# Гауссовский шум
psnr_noisy_gauss = calculate_psnr(image, noisy_gauss)
psnr_filtered_gauss = calculate_psnr(image, filtered_gauss)
print(f"\nГауссовский шум:")
print(f"  PSNR зашумленного: {psnr_noisy_gauss:.2f} dB")
print(f"  PSNR после фильтра: {psnr_filtered_gauss:.2f} dB")
print(f"  Улучшение: {psnr_filtered_gauss - psnr_noisy_gauss:.2f} dB")

# Соль-перец
psnr_noisy_sp = calculate_psnr(image, noisy_sp)
psnr_filtered_sp = calculate_psnr(image, filtered_sp)
print(f"\nСоль-перец:")
print(f"  PSNR зашумленного: {psnr_noisy_sp:.2f} dB")
print(f"  PSNR после фильтра: {psnr_filtered_sp:.2f} dB")
print(f"  Улучшение: {psnr_filtered_sp - psnr_noisy_sp:.2f} dB")

# Спекл-шум
psnr_noisy_speckle = calculate_psnr(image, noisy_speckle)
psnr_filtered_speckle = calculate_psnr(image, filtered_speckle)
print(f"\nСпекл-шум:")
print(f"  PSNR зашумленного: {psnr_noisy_speckle:.2f} dB")
print(f"  PSNR после фильтра: {psnr_filtered_speckle:.2f} dB")
print(f"  Улучшение: {psnr_filtered_speckle - psnr_noisy_speckle:.2f} dB")

# Пуассоновский шум
psnr_noisy_poisson = calculate_psnr(image, noisy_poisson)
psnr_filtered_poisson = calculate_psnr(image, filtered_poisson)
print(f"\nПуассоновский шум:")
print(f"  PSNR зашумленного: {psnr_noisy_poisson:.2f} dB")
print(f"  PSNR после фильтра: {psnr_filtered_poisson:.2f} dB")
print(f"  Улучшение: {psnr_filtered_poisson - psnr_noisy_poisson:.2f} dB")
