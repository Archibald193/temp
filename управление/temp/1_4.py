import cv2
import numpy as np
import matplotlib.pyplot as plt

def histogram_matching(source, target):
    """
    Приведение гистограммы source к гистограмме target
    """
    # Вычисляем гистограммы
    hist_source = cv2.calcHist([source], [0], None, [256], [0,256]).ravel()
    hist_target = cv2.calcHist([target], [0], None, [256], [0,256]).ravel()
    
    # Нормализуем в PDF (ПРВ)
    pdf_source = hist_source / hist_source.sum()
    pdf_target = hist_target / hist_target.sum()
    
    # Вычисляем CDF
    cdf_source = pdf_source.cumsum()
    cdf_target = pdf_target.cumsum()
    
    # Создаем таблицу преобразования
    mapping = np.zeros(256, dtype=np.uint8)
    for i in range(256):
        # Находим значение j, где cdf_target[j] ≈ cdf_source[i]
        diff = np.abs(cdf_target - cdf_source[i])
        mapping[i] = np.argmin(diff)
    
    # Применяем преобразование
    result = mapping[source]
    
    return result

# Загрузка изображений
source = cv2.imread('n_k.png', 0)
target = cv2.imread('saturn.png', 0)

# Применяем приведение гистограммы
matched = histogram_matching(source, target)

# Визуализация
plt.figure(figsize=(15, 10))
plt.subplot(2,3,1), plt.imshow(source, cmap='gray'), plt.title('Исходное')
plt.subplot(2,3,2), plt.imshow(target, cmap='gray'), plt.title('Целевое')
plt.subplot(2,3,3), plt.imshow(matched, cmap='gray'), plt.title('Результат')

plt.subplot(2,3,4), plt.hist(source.ravel(), 256), plt.title('Гистограмма исходного')
plt.subplot(2,3,5), plt.hist(target.ravel(), 256), plt.title('Гистограмма целевого')
plt.subplot(2,3,6), plt.hist(matched.ravel(), 256), plt.title('Гистограмма результата')
plt.show()
