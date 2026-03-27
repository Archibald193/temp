import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.feature import graycomatrix, graycoprops

# Загрузка изображения
img = cv2.imread('APK.png', cv2.IMREAD_GRAYSCALE)
if img is None:
    # Создаем тестовые текстуры
    img = np.zeros((400, 600), dtype=np.uint8)
    # Гладкая
    img[0:200, 0:200] = 128
    # Полосатая горизонтальная
    for i in range(0, 200, 20):
        img[0:200, 200:400, i:i+5] = 255
    # Полосатая вертикальная
    for i in range(200, 400, 20):
        img[0:200, 400:600, :, i:i+5] = 255
    # Мелкозернистая
    img[200:400, 0:200] = np.random.normal(128, 40, (200, 200)).astype(np.uint8)
    # Крупнозернистая
    for i in range(200, 400, 30):
        for j in range(200, 400, 30):
            img[i:i+15, j:j+15] = np.random.randint(100, 200)
    # Клетчатая
    for i in range(200, 400, 20):
        for j in range(400, 600, 20):
            img[i:i+10, j:j+10] = 200

# Разделение на регионы
regions = [
    ("Гладкая", img[0:100, 0:100]),
    ("Горизонтальные полосы", img[0:100, 200:300]),
    ("Вертикальные полосы", img[0:100, 400:500]),
    ("Мелкозернистая", img[250:350, 50:150]),
    ("Крупнозернистая", img[250:350, 250:350]),
    ("Клетчатая", img[250:350, 450:550])
]

print("GLCM ПРИЗНАКИ ТЕКСТУРЫ")
print("-" * 70)

plt.figure(figsize=(15, 10))

for i, (name, region) in enumerate(regions):
    # Дискретизация до 8 уровней
    region_8 = (region / 32).astype(np.uint8)
    
    # Вычисление GLCM для разных направлений
    glcm_h = graycomatrix(region_8, [1], [0], levels=8, symmetric=True, normed=True)
    glcm_v = graycomatrix(region_8, [1], [np.pi/2], levels=8, symmetric=True, normed=True)
    glcm_d1 = graycomatrix(region_8, [1], [np.pi/4], levels=8, symmetric=True, normed=True)
    glcm_d2 = graycomatrix(region_8, [1], [3*np.pi/4], levels=8, symmetric=True, normed=True)
    
    # Признаки Харалика
    contrast_h = graycoprops(glcm_h, 'contrast')[0, 0]
    contrast_v = graycoprops(glcm_v, 'contrast')[0, 0]
    
    dissimilarity_h = graycoprops(glcm_h, 'dissimilarity')[0, 0]
    homogeneity_h = graycoprops(glcm_h, 'homogeneity')[0, 0]
    energy_h = graycoprops(glcm_h, 'energy')[0, 0]
    correlation_h = graycoprops(glcm_h, 'correlation')[0, 0]
    asm_h = graycoprops(glcm_h, 'ASM')[0, 0]
    
    # Определение направления текстуры
    if contrast_h > contrast_v * 1.5:
        orientation = "Вертикальная"
    elif contrast_v > contrast_h * 1.5:
        orientation = "Горизонтальная"
    else:
        orientation = "Равномерная/Изотропная"
    
    print(f"\n{name.upper()}:")
    print(f"  Контраст (гор/верт): {contrast_h:.3f} / {contrast_v:.3f}")
    print(f"  Ориентация: {orientation}")
    print(f"  Однородность: {homogeneity_h:.3f}")
    print(f"  Энергия: {energy_h:.3f}")
    print(f"  Корреляция: {correlation_h:.3f}")
    
    # Визуализация
    plt.subplot(2, 3, i+1)
    plt.imshow(region, cmap='gray')
    plt.title(f'{name}\nОриентация: {orientation}')
    plt.axis('off')

plt.tight_layout()
plt.show()
