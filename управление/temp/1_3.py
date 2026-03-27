import cv2
import matplotlib.pyplot as plt
import numpy as np
# Загрузка изображения
img = cv2.imread('n_k.png', 0)
# 1. Глобальная эквализация
global_eq = cv2.equalizeHist(img)
# 2. АДАПТИВНАЯ ЭКВАЛИЗАЦИЯ (AHE)
def adaptive_equalization(img, tile_size=(50, 50)):
    """
    Адаптивная эквализация гистограммы
    Изображение делится на блоки, каждый блок эквализируется отдельно
    """
    h, w = img.shape
    tile_h, tile_w = tile_size
    result = np.zeros_like(img)
    
    # Проходим по всем блокам
    for i in range(0, h, tile_h):
        for j in range(0, w, tile_w):
            # Определяем границы блока
            i_end = min(i + tile_h, h)
            j_end = min(j + tile_w, w)         
            # Извлекаем блок и применяем эквализацию
            tile = img[i:i_end, j:j_end]
            tile_eq = cv2.equalizeHist(tile)           
            # Сохраняем результат
            result[i:i_end, j:j_end] = tile_eq
    
    return result

ahe_result = adaptive_equalization(img, tile_size=(50, 50))
# 3. CLAHE (Contrast Limited Adaptive Histogram Equalization)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
clahe_result = clahe.apply(img)
# Сравнение всех методов
titles = ['Оригинал', 'Глобальная', 'AHE', 'CLAHE']
images = [img, global_eq, ahe_result, clahe_result]
# Отображение в сетке 2×2
plt.figure(figsize=(12, 10))

for i in range(4):
    plt.subplot(2, 2, i+1)
    plt.imshow(images[i], cmap='gray')
    plt.title(titles[i])
    plt.axis('off')
plt.tight_layout()
plt.show()
# Дополнительно: гистограммы для сравнения
plt.figure(figsize=(12, 8))

for i in range(4):
    plt.subplot(2, 2, i+1)
    plt.hist(images[i].ravel(), 256, [0, 256])
    plt.title(f'Гистограмма: {titles[i]}')
    plt.xlabel('Яркость')
    plt.ylabel('Количество пикселей')

plt.tight_layout()
plt.show()
# Статистика
print("\n" + "="*50)
print("СТАТИСТИКА ИЗОБРАЖЕНИЙ")
print("="*50)

for i in range(4):
    print(f"\n{titles[i]}:")
    print(f"  Среднее: {np.mean(images[i]):.1f}")
    print(f"  Стд.откл.: {np.std(images[i]):.1f}")
    print(f"  Мин/Макс: {images[i].min()}/{images[i].max()}")
