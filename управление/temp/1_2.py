import cv2
import matplotlib.pyplot as plt
img = cv2.imread('n_k.png', 0)
# 1. Глобальная
global_eq = cv2.equalizeHist(img)
# 2. CLAHE
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
clahe_result = clahe.apply(img)
# Сравнение
titles = ['Оригинал', 'Глобальная', 'CLAHE']
images = [img, global_eq, clahe_result]
for i in range(3):
    plt.subplot(1,3,i+1)
    plt.imshow(images[i], cmap='gray')
    plt.title(titles[i])
plt.show()
