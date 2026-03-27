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

# Анализ каждого контура
for contour in contours:
    if cv2.contourArea(contour) < 100:
        continue
    
    # Основные признаки формы
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    
    # Аппроксимация контура (определяем количество углов)
    epsilon = 0.04 * perimeter
    approx = cv2.approxPolyDP(contour, epsilon, True)
    corners = len(approx)
    
    # Соотношение сторон bounding box
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = w / h if h > 0 else 0
    
    # Компактность (периметр^2 / площадь)
    compactness = perimeter**2 / (4 * np.pi * area) if area > 0 else 0
    
    # Определяем форму
    if corners == 3:
        shape = "Треугольник"
    elif corners == 4:
        if 0.9 <= aspect_ratio <= 1.1:
            shape = "Квадрат"
        else:
            shape = "Прямоугольник"
    elif corners > 5 and compactness < 1.5:
        shape = "Круг"
    else:
        shape = "Многоугольник"
    
    # Рисуем результат
    cv2.drawContours(img_result, [contour], -1, (255, 0, 0), 2)
    cv2.putText(img_result, shape, (x, y-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    # Вывод признаков
    print(f"\n{shape}: углов={corners}, площадь={area:.0f}, "
          f"соотношение={aspect_ratio:.2f}, компактность={compactness:.2f}")

# Отображение
plt.figure(figsize=(10, 5))
plt.subplot(121); plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)); plt.title('Оригинал')
plt.subplot(122); plt.imshow(cv2.cvtColor(img_result, cv2.COLOR_BGR2RGB)); plt.title('Распознанные формы')
plt.show()
