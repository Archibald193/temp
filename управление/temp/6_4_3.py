import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.special import comb

def zernike_moment(img, n, m, radius=20):
    """Вычисление момента Зернике порядка n и повторения m"""
    if m > n or (n - m) % 2 != 0:
        return 0
    
    # Создание сетки координат
    y, x = np.ogrid[-radius:radius+1, -radius:radius+1]
    rho = np.sqrt(x**2 + y**2) / radius
    theta = np.arctan2(y, x)
    
    # Маска круга
    mask = rho <= 1
    
    # Радиальный полином Зернике
    R = 0
    for k in range((n - m) // 2 + 1):
        R += ((-1)**k * comb(n - k, k) * comb(n - 2*k, (n - m)//2 - k) * 
              rho**(n - 2*k))
    
    # Базисная функция Зернике
    V = R * np.exp(1j * m * theta)
    
    # Вычисление момента
    roi = cv2.resize(img, (2*radius+1, 2*radius+1))
    moment = np.sum(roi * V * mask)
    
    # Нормализация
    moment = moment * (n + 1) / np.pi
    
    return abs(moment)

# Загрузка изображения
img = cv2.imread('n_k.png')
if img is None:
    # Создаем тестовое изображение
    img = np.ones((400, 600, 3), dtype=np.uint8) * 255
    cv2.rectangle(img, (50, 50), (150, 150), (0, 0, 0), 2)    # квадрат
    cv2.circle(img, (250, 100), 50, (0, 0, 0), 2)             # круг
    cv2.rectangle(img, (350, 50), (500, 150), (0, 0, 0), 2)   # прямоугольник
    pts = np.array([[100, 250], [200, 250], [150, 350]], np.int32)
    cv2.polylines(img, [pts], True, (0, 0, 0), 2)              # треугольник

# Конвертация в gray и бинаризация
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
_, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Поиск контуров
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

img_result = img.copy()

print("МОМЕНТЫ ЗЕРНИКЕ (первые 4 порядка):")
print("-" * 50)

# Анализ каждого контура
for i, contour in enumerate(contours):
    if cv2.contourArea(contour) < 100:
        continue
    
    # Создаем маску для контура
    mask = np.zeros(gray.shape, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    
    # Вычисляем моменты Зернике разных порядков
    z00 = zernike_moment(mask, 0, 0)  # порядок 0 (площадь)
    z11 = zernike_moment(mask, 1, 1)  # порядок 1 (центр масс)
    z20 = zernike_moment(mask, 2, 0)  # порядок 2,0 (вытянутость)
    z22 = zernike_moment(mask, 2, 2)  # порядок 2,2 (ориентация)
    z31 = zernike_moment(mask, 3, 1)  # порядок 3 (асимметрия)
    
    # Определение формы по моментам (упрощенная эвристика)
    if z20 < 0.01:  # маленькая вытянутость
        shape = "Круг"
    elif z22 > 0.5:  # сильная квадратичная компонента
        x, y, w, h = cv2.boundingRect(contour)
        if 0.9 <= w/h <= 1.1:
            shape = "Квадрат"
        else:
            shape = "Прямоугольник"
    else:
        shape = "Многоугольник"
    
    # Рисуем результат
    x, y, w, h = cv2.boundingRect(contour)
    cv2.drawContours(img_result, [contour], -1, (255, 0, 0), 2)
    cv2.putText(img_result, shape, (x, y-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    # Вывод моментов
    print(f"\nКонтур {i+1} - {shape}:")
    print(f"  Z20 (вытянутость): {z20:.4f}")
    print(f"  Z22 (квадратичность): {z22:.4f}")
    print(f"  Z31 (асимметрия): {z31:.4f}")

# Отображение
plt.figure(figsize=(12, 5))
plt.subplot(121); plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)); plt.title('Оригинал')
plt.subplot(122); plt.imshow(cv2.cvtColor(img_result, cv2.COLOR_BGR2RGB)); plt.title('Распознавание по Зернике')
plt.show()
