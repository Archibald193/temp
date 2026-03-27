import cv2
import numpy as np
import matplotlib.pyplot as plt

# Загрузка изображения
img = cv2.imread('n_k.png', cv2.IMREAD_GRAYSCALE)
if img is None:
    # Создаем тестовые текстуры
    img = np.zeros((400, 600), dtype=np.uint8)
    # Гладкая текстура
    img[0:200, 0:300] = 128
    # Шумная текстура
    noise = np.random.normal(128, 30, (200, 300)).astype(np.uint8)
    img[0:200, 300:600] = noise
    # Полосатая текстура
    for i in range(200, 400, 10):
        img[200:400, 0:300, i:i+5] = 255
    # Клетчатая текстура
    for i in range(200, 400, 20):
        for j in range(300, 600, 20):
            img[i:i+10, j:j+10] = 200

# Разделение на регионы для анализа
h, w = img.shape
regions = [
    ("Гладкая", img[0:100, 0:100]),
    ("Шумная", img[0:100, 300:400]),
    ("Полосатая", img[200:300, 0:100]),
    ("Клетчатая", img[200:300, 300:400])
]

print("СТАТИСТИЧЕСКИЕ ПРИЗНАКИ ТЕКСТУРЫ")
print("-" * 50)

plt.figure(figsize=(12, 8))

for i, (name, region) in enumerate(regions):
    # Статистические признаки первого порядка
    mean = np.mean(region)
    std = np.std(region)
    variance = np.var(region)
    min_val = np.min(region)
    max_val = np.max(region)
    
    # Гистограмма
    hist = cv2.calcHist([region], [0], None, [256], [0, 256])
    hist = hist / hist.sum()  # нормализация
    
    # Энтропия
    entropy = -np.sum(hist[hist > 0] * np.log2(hist[hist > 0]))
    
    # Энергия (однородность)
    energy = np.sum(hist**2)
    
    # Асимметрия (skewness)
    skewness = np.mean(((region - mean)/std)**3) if std > 0 else 0
    
    # Эксцесс (kurtosis)
    kurtosis = np.mean(((region - mean)/std)**4) if std > 0 else 0
    
    print(f"\n{name.upper()} ТЕКСТУРА:")
    print(f"  Среднее: {mean:.1f}")
    print(f"  Стд. отклонение: {std:.1f}")
    print(f"  Энтропия: {entropy:.3f}")
    print(f"  Энергия: {energy:.3f}")
    print(f"  Асимметрия: {skewness:.3f}")
    print(f"  Эксцесс: {kurtosis:.3f}")
    
    # Визуализация
    plt.subplot(2, 4, i*2+1)
    plt.imshow(region, cmap='gray')
    plt.title(f'{name}')
    plt.axis('off')
    
    plt.subplot(2, 4, i*2+2)
    plt.plot(hist)
    plt.title(f'Гистограмма {name}')
    plt.xlim([0, 255])

plt.tight_layout()
plt.show()
