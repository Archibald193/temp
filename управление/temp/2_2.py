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

def compare_noise_levels(image, noise_type='gaussian'):
    """
    Сравнение разных уровней одного типа шума
    """
    plt.figure(figsize=(15, 10))
    
    # Оригинал
    plt.subplot(2, 3, 1)
    plt.imshow(image, cmap='gray')
    plt.title('Оригинал')
    plt.axis('off')
    
    # Разные уровни шума
    levels = [10, 25, 50, 75, 100]
    
    for i, level in enumerate(levels, 2):
        if noise_type == 'gaussian':
            noise = np.random.normal(0, level, image.shape)
            noisy = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
            title = f'σ={level}'
        
        elif noise_type == 'salt_pepper':
            noisy = image.copy()
            prob = level / 1000  # 0.01, 0.025, 0.05, 0.075, 0.1
            mask = np.random.random(image.shape)
            noisy[mask < prob/2] = 255
            noisy[mask > 1 - prob/2] = 0
            title = f'{prob*100:.1f}%'
        
        elif noise_type == 'speckle':
            noise = 1 + np.random.normal(0, level/100, image.shape)
            noisy = np.clip(image.astype(np.float32) * noise, 0, 255).astype(np.uint8)
            title = f'intensity={level/100}'
        
        plt.subplot(2, 3, i)
        plt.imshow(noisy, cmap='gray')
        plt.title(title)
        plt.axis('off')
    
    plt.suptitle(f'Тип шума: {noise_type}')
    plt.tight_layout()
    plt.show()

# Примеры использования
compare_noise_levels(image, 'gaussian')
compare_noise_levels(image, 'salt_pepper')
compare_noise_levels(image, 'speckle')
