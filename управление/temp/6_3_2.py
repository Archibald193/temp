import cv2
import numpy as np
import matplotlib.pyplot as plt

# Загрузка изображений
img1 = cv2.imread('6_3_1.jpg')
img2 = cv2.imread('6_3_2.jpg')

if img1 is None or img2 is None:
    print("Ошибка: не удалось загрузить изображения")
    exit()

# Конвертация в оттенки серого
gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

# ORB детектор
orb = cv2.ORB_create(nfeatures=2000)
kp1, des1 = orb.detectAndCompute(gray1, None)
kp2, des2 = orb.detectAndCompute(gray2, None)

# FLANN параметры для ORB
FLANN_INDEX_LSH = 6
index_params = dict(algorithm=FLANN_INDEX_LSH,
                    table_number=6,
                    key_size=12,
                    multi_probe_level=1)
search_params = dict(checks=50)

# FLANN матчер
flann = cv2.FlannBasedMatcher(index_params, search_params)

# Поиск совпадений (k=2 для проверки отношения)
matches = flann.knnMatch(des1, des2, k=2)

# Отбор хороших совпадений по критерию Lowe's ratio
good_matches = []
for match_pair in matches:
    if len(match_pair) == 2:
        m, n = match_pair
        if m.distance < 0.75 * n.distance:  # порог отношения
            good_matches.append(m)

# Сортировка по расстоянию
good_matches = sorted(good_matches, key=lambda x: x.distance)

print(f"Найдено хороших совпадений: {len(good_matches)}")

# Визуализация
img_matches = cv2.drawMatches(img1, kp1, img2, kp2, good_matches[:100], None,
                              flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

plt.figure(figsize=(15, 8))
plt.imshow(cv2.cvtColor(img_matches, cv2.COLOR_BGR2RGB))
plt.title(f'ORB + FLANN совпадения (Lowe\'s ratio test) - показано {min(100, len(good_matches))} из {len(good_matches)}')
plt.axis('off')
plt.show()

# Если достаточно совпадений, можно найти гомографию
if len(good_matches) > 10:
    # Извлечение координат точек
    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    
    # Нахождение гомографии с RANSAC
    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    
    inliers = mask.ravel().tolist().count(1)
    print(f"Найдено гомографией inliers: {inliers} из {len(good_matches)}")
    
    # Визуализация inliers
    inlier_matches = [good_matches[i] for i in range(len(good_matches)) if mask[i]]
    
    img_inliers = cv2.drawMatches(img1, kp1, img2, kp2, inlier_matches, None,
                                 flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    
    plt.figure(figsize=(15, 8))
    plt.imshow(cv2.cvtColor(img_inliers, cv2.COLOR_BGR2RGB))
    plt.title(f'Inliers после RANSAC ({len(inlier_matches)} совпадений)')
    plt.axis('off')
    plt.show()
