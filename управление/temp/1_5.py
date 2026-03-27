import cv2
import matplotlib.pyplot as plt
import numpy as np

img_color = cv2.imread('APK.png')
img_color = cv2.cvtColor(img_color, cv2.COLOR_BGR2RGB)
# для корректного отображения

# Вариант 1: Эквализация каждого канала отдельно
r, g, b = cv2.split(img_color)
r_eq = cv2.equalizeHist(r)
g_eq = cv2.equalizeHist(g)
b_eq = cv2.equalizeHist(b)
color_per_channel = cv2.merge([r_eq, g_eq, b_eq])

# Вариант 2: Конвертация в HSV и эквализация только канала яркости
img_hsv = cv2.cvtColor(img_color, cv2.COLOR_RGB2HSV)
h, s, v = cv2.split(img_hsv)
v_eq = cv2.equalizeHist(v)
hsv_eq = cv2.merge([h, s, v_eq])
color_hsv = cv2.cvtColor(hsv_eq, cv2.COLOR_HSV2RGB)

# Вариант 3: CLAHE в цветовом пространстве LAB
img_lab = cv2.cvtColor(img_color, cv2.COLOR_RGB2LAB)
l, a, b_lab = cv2.split(img_lab)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
l_eq = clahe.apply(l)
lab_eq = cv2.merge([l_eq, a, b_lab])
color_lab = cv2.cvtColor(lab_eq, cv2.COLOR_LAB2RGB)

# Отображение
plt.figure(figsize=(15, 10))

images = [img_color, color_per_channel, color_hsv, color_lab]
titles = ['Оригинал', 'По каналам RGB', 'HSV (V канал)', 'LAB (L канал)']

for i in range(4):
    plt.subplot(2, 4, i+1)
    plt.imshow(images[i])
    plt.title(titles[i])
    plt.axis('off')
    
    # Гистограмма яркости (преобразуем в градации серого для сравнения)
    gray = cv2.cvtColor(cv2.cvtColor(images[i], cv2.COLOR_RGB2BGR), cv2.COLOR_BGR2GRAY)
    plt.subplot(2, 4, i+5)
    plt.hist(gray.ravel(), 256, [0, 256])
    plt.title(f'Гистограмма {titles[i]}')

plt.tight_layout()
plt.show()

