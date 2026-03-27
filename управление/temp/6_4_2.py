import cv2
import numpy as np
import matplotlib.pyplot as plt

# Загрузка изображения
img = cv2.imread('APK.png')
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

print("МОМЕНТЫ HU (логарифмированные значения):")
print("-" * 50)

# Анализ каждого контура
for i, contour in enumerate(contours):
    if cv2.contourArea(contour) < 100:
        continue
    
    # Вычисление моментов Hu
    moments = cv2.moments(contour)
    hu = cv2.HuMoments(moments).flatten()
    
    # Логарифмирование для лучшего сравнения
    hu_log = -np.sign(hu) * np.log10(np.abs(hu) + 1e-10)
    
    # Определение формы по моментам Hu (эвристика)
    if hu_log[0] < -2.0:  # Круг имеет маленькое значение первого момента
        shape = "Круг"
    elif hu_log[1] < -3.0:  # Квадрат/прямоугольник
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
    
    # Вывод моментов Hu
    print(f"\nКонтур {i+1} - {shape}:")
    print(f"  Hu1: {hu_log[0]:.3f}")
    print(f"  Hu2: {hu_log[1]:.3f}")
    print(f"  Hu3: {hu_log[2]:.3f}")

# Отображение
plt.figure(figsize=(12, 5))
plt.subplot(121); plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)); plt.title('Оригинал')
plt.subplot(122); plt.imshow(cv2.cvtColor(img_result, cv2.COLOR_BGR2RGB)); plt.title('Распознавание по Hu')
plt.show()
