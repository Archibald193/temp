import cv2
import numpy as np
import matplotlib.pyplot as plt

def create_gabor_filter(ksize=31, sigma=4.0, theta=0, lambd=10.0, gamma=0.5, psi=0):
    """Создание фильтра Габора"""
    kernel = cv2.getGaborKernel((ksize, ksize), sigma, theta, lambd, gamma, psi, ktype=cv2.CV_32F)
    return kernel

def apply_gabor_filter(img, kernel):
    """Применение фильтра Габора"""
    filtered = cv2.filter2D(img, cv2.CV_8UC3, kernel)
    return filtered

# Загрузка изображения
img = cv2.imread('n_k.png', cv2.IMREAD_GRAYSCALE)
if img is None:
    # Создаем тестовые текстуры
    img = np.zeros((400, 600), dtype=np.uint8)
    
    # Гладкая текстура
    img[0:200, 0:200] = 128
    
    # Шумная текстура
    img[0:200, 200:400] = np.random.normal(128, 40, (200, 200)).astype(np.uint8)
    
    # Полосатая текстура (горизонтальная)
    for i in range(0, 200, 10):
        img[0:200, 400:600, i:i+3] = 255
    
    # Полосатая текстура (вертикальная)
    for i in range(200, 400, 10):
        img[200:400, 0:200, :, i:i+3] = 255
    
    # Клетчатая текстура
    for i in range(200, 400, 20):
        for j in range(200, 400, 20):
            img[i:i+10, j:j+10] = 200
    
    # Текстурный градиент
    for i in range(200, 400):
        for j in range(400, 600):
            img[i, j] = (i + j) % 255

# Параметры фильтров Габора
sigmas = [3, 5, 7]  # масштабы
thetas = [0, np.pi/4, np.pi/2, 3*np.pi/4]  # ориентации (0°, 45°, 90°, 135°)
lambd = 10  # длина волны
gamma = 0.5  # эллиптичность
ksi = 31  # размер ядра

print("ПРИЗНАКИ ТЕКСТУР НА ОСНОВЕ ФИЛЬТРОВ ГАБОРА")
print("=" * 60)

# Применение фильтров с разными ориентациями
results = []
filter_names = []

plt.figure(figsize=(15, 10))

# Оригинал
plt.subplot(3, 5, 1)
plt.imshow(img, cmap='gray')
plt.title('Оригинал')
plt.axis('off')

# Применяем фильтры с разными углами
idx = 2
for i, theta in enumerate(thetas):
    kernel = create_gabor_filter(ksize=ksi, sigma=5, theta=theta, lambd=lambd, gamma=gamma)
    filtered = apply_gabor_filter(img, kernel)
    
    # Статистики фильтрованного изображения
    mean = np.mean(filtered)
    std = np.std(filtered)
    energy = np.sum(filtered**2) / filtered.size
    
    print(f"\nОриентация {theta*180/np.pi:.0f}°:")
    print(f"  Среднее: {mean:.2f}")
    print(f"  Стд. отклонение: {std:.2f}")
    print(f"  Энергия: {energy:.2f}")
    
    plt.subplot(3, 5, idx)
    plt.imshow(filtered, cmap='gray')
    plt.title(f'θ={theta*180/np.pi:.0f}°')
    plt.axis('off')
    idx += 1

# Применяем фильтры с разными масштабами
for i, sigma in enumerate(sigmas):
    kernel = create_gabor_filter(ksize=ksi, sigma=sigma, theta=np.pi/4, lambd=lambd, gamma=gamma)
    filtered = apply_gabor_filter(img, kernel)
    
    mean = np.mean(filtered)
    std = np.std(filtered)
    energy = np.sum(filtered**2) / filtered.size
    
    print(f"\nМасштаб σ={sigma}:")
    print(f"  Среднее: {mean:.2f}")
    print(f"  Стд. отклонение: {std:.2f}")
    print(f"  Энергия: {energy:.2f}")
    
    plt.subplot(3, 5, idx)
    plt.imshow(filtered, cmap='gray')
    plt.title(f'σ={sigma}')
    plt.axis('off')
    idx += 1

plt.tight_layout()
plt.show()

# Анализ текстурных регионов
print("\n" + "="*60)
print("АНАЛИЗ ТЕКСТУРНЫХ РЕГИОНОВ")
print("="*60)

# Выделим несколько регионов для анализа
regions = [
    ("Гладкая", img[50:150, 50:150]),
    ("Шумная", img[50:150, 250:350]),
    ("Горизонтальная", img[50:150, 450:550]),
    ("Вертикальная", img[250:350, 50:150]),
    ("Клетчатая", img[250:350, 250:350]),
    ("Градиент", img[250:350, 450:550])
]

# Для каждого региона вычислим отклики на фильтры Габора
for name, region in regions:
    print(f"\n{name.upper()}:")
    
    features = []
    for theta in thetas:
        kernel = create_gabor_filter(ksize=ksi, sigma=5, theta=theta, lambd=lambd, gamma=gamma)
        filtered = apply_gabor_filter(region, kernel)
        
        # Энергия отклика
        energy = np.sum(filtered**2) / filtered.size
        features.append(energy)
        print(f"  Энергия при {theta*180/np.pi:.0f}°: {energy:.3f}")
    
    # Определение доминирующей ориентации
    dominant_theta = thetas[np.argmax(features)]
    print(f"  Доминирующая ориентация: {dominant_theta*180/np.pi:.0f}°")

# Визуализация банка фильтров Габора
plt.figure(figsize=(12, 8))
plt.suptitle("Банк фильтров Габора", fontsize=14)

# Показываем фильтры для разных ориентаций и масштабов
for i, sigma in enumerate(sigmas):
    for j, theta in enumerate(thetas):
        kernel = create_gabor_filter(ksize=ksi, sigma=sigma, theta=theta, lambd=lambd, gamma=gamma)
        
        plt.subplot(len(sigmas), len(thetas), i*len(thetas) + j + 1)
        plt.imshow(kernel, cmap='gray')
        plt.title(f'σ={sigma}, θ={theta*180/np.pi:.0f}°')
        plt.axis('off')

plt.tight_layout()
plt.show()

# Функция для извлечения вектора признаков на основе фильтров Габора
def extract_gabor_features(img, sigmas=[3, 5, 7], thetas=[0, np.pi/4, np.pi/2, 3*np.pi/4]):
    """Извлечение признаков Габора из изображения"""
    features = []
    
    for sigma in sigmas:
        for theta in thetas:
            kernel = create_gabor_filter(ksize=31, sigma=sigma, theta=theta, lambd=10, gamma=0.5)
            filtered = apply_gabor_filter(img, kernel)
            
            # Статистики отклика
            mean = np.mean(filtered)
            std = np.std(filtered)
            energy = np.sum(filtered**2) / filtered.size
            
            features.extend([mean, std, energy])
    
    return np.array(features)

# Пример извлечения признаков для разных регионов
print("\n" + "="*60)
print("ВЕКТОР ПРИЗНАКОВ ГАБОРА ДЛЯ РАЗНЫХ ТЕКСТУР")
print("="*60)

for name, region in regions[:3]:  # первые 3 региона
    features = extract_gabor_features(region)
    print(f"\n{name.upper()} - первые 6 признаков из {len(features)}:")
    print(f"  {features[:6]}")

