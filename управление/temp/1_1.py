import cv2
import matplotlib.pyplot as plt
# Загрузка изображения
img = cv2.imread('n_k.png', 0)
# Эквализация гистограммы
equalized = cv2.equalizeHist(img)
# Сравнение
plt.subplot(2,2,1), plt.imshow(img, cmap='gray'), plt.title('До')
plt.subplot(2,2,2), plt.hist(img.ravel(), 256), plt.title('Гистограмма до')
plt.subplot(2,2,3), plt.imshow(equalized, cmap='gray'), plt.title('После') 
plt.subplot(2,2,4), plt.hist(equalized.ravel(), 256), plt.title('Гистограмма после')
plt.show()
