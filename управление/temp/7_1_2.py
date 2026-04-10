import numpy as np

def sigmoid(x):
    """Функция активации"""
    return 1 / (1 + np.exp(-x))

class BananaFreshnessNetwork:
    '''
    Нейронная сеть для предсказания свежести банана:
    - 2 входа (цвет, мягкость)
    - скрытый слой с 3 нейронами
    - 1 выход (свежесть от 0 до 1)
    '''
    def __init__(self, input_size=2):
        # Инициализируем веса случайными числами
        # Для скрытого слоя (3 нейрона, каждый с 2 входами)
        self.w1 = np.random.randn(10, input_size)  # веса для скрытого слоя
        self.b1 = np.random.randn(10)      # пороги для скрытого слоя
        
        # Для выходного слоя (1 нейрон с 3 входами)
        self.w2 = np.random.randn(1, 10)
        self.b2 = np.random.randn(1)
    
    def feedforward(self, x):
        """Прямой проход через сеть"""
        # Скрытый слой
        h = sigmoid(np.dot(self.w1, x) + self.b1)
        
        # Выходной слой
        out = sigmoid(np.dot(self.w2, h) + self.b2)
        return out[0]
    
    def train(self, X, y, epochs=1000, lr=0.1):
        """
        Обучение сети методом обратного распространения
        X - входные данные
        y - целевые значения
        epochs - количество эпох обучения
        lr - скорость обучения (learning rate)
        """
        for epoch in range(epochs):
            total_loss = 0
            
            for i in range(len(X)):
                # Прямой проход
                x = X[i]
                target = y[i]
                
                # Скрытый слой
                h = sigmoid(np.dot(self.w1, x) + self.b1)
                
                # Выходной слой
                out = sigmoid(np.dot(self.w2, h) + self.b2)[0]
                
                # Считаем ошибку (MSE)
                error = out - target
                total_loss += error ** 2
                
                # Обратное распространение (выходной слой)
                # Производная сигмоиды: out * (1 - out)
                d_out = error * out * (1 - out)
                
                # Обновляем веса выходного слоя
                # w2 += -lr * d_out * h (градиент)
                self.w2 -= lr * d_out * h.reshape(1, -1)
                self.b2 -= lr * d_out
                
                # Обратное распространение (скрытый слой)
                # Градиент для скрытого слоя
                d_h = (d_out * self.w2).reshape(-1) * h * (1 - h)
                
                # Обновляем веса скрытого слоя
                self.w1 -= lr * np.outer(d_h, x)
                self.b1 -= lr * d_h
            
            # Выводим ошибку каждые 100 эпох
            if epoch % 100 == 0:
                avg_loss = total_loss / len(X)
                print(f"Эпоха {epoch}, Ошибка: {avg_loss:.6f}")

# 1. Подготовка данных
X_banana_3 = np.array([
    [0.1, 0.1, 0.0], [0.3, 0.2, 0.1], [0.5, 0.3, 0.2],
    [0.6, 0.4, 0.4], [0.7, 0.6, 0.6], [0.8, 0.7, 0.8], [0.9, 0.9, 1.0]
])
y_banana = np.array([0.9, 0.8, 0.7, 0.6, 0.4, 0.3, 0.1])

# 2. Создаем сеть
network = BananaFreshnessNetwork(input_size=3)

# 3. Обучаем
print("Обучение началось...")
network.train(X_banana_3, y_banana, epochs=1000, lr=0.5)

# 4. Тестируем на обучающих данных (проверка)
print("\nПроверка на обучающих данных:")
for i in range(len(X_banana_3)):
    pred = network.feedforward(X_banana_3[i])
    print(f"Банан {i+1}: реальная свежесть={y_banana[i]:.1f}, предсказание={pred:.3f}")

# 5. Тестируем на новых данных
X_apple = np.array([
    [0.9, 0.9, 0.0], # Идеальное красное, твердое
    [0.8, 0.8, 0.1], # Хорошее
    [0.6, 0.7, 0.2], # Среднее
    [0.5, 0.5, 0.4], # Начинает вянуть
    [0.4, 0.4, 0.6], # Помятое
    [0.3, 0.3, 0.8], # Сильные пятна
    [0.2, 0.2, 0.9], # Порченое
    [0.1, 0.1, 1.0]  # Гнилое
])
y_apple = np.array([0.95, 0.85, 0.7, 0.5, 0.35, 0.2, 0.1, 0.05])
apple_net = BananaFreshnessNetwork(input_size=3)
apple_net.train(X_apple, y_apple, epochs=1500, lr=0.3)
print("\nРезультаты тестирования яблок")
for i in range(len(X_apple)):
    pred = apple_net.feedforward(X_apple[i])
    print(f"Яблоко {i+1}: реальная={y_apple[i]:.2f}, предсказание={pred:.3f}")
