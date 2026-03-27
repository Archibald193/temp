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
    def __init__(self):
        # Инициализируем веса случайными числами
        # Для скрытого слоя (3 нейрона, каждый с 2 входами)
        self.w1 = np.random.randn(10, 2)  # веса для скрытого слоя
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
X = np.array([
    [0.1, 0.1],  # зеленый, твердый
    [0.3, 0.2],  # желто-зеленый
    [0.5, 0.3],  # желтый
    [0.6, 0.4],  # желтый, мягче
    [0.7, 0.6],  # желтый, мягкий
    [0.8, 0.7],  # коричневый
    [0.9, 0.9]   # черный, мягкий
])

y = np.array([0.9, 0.8, 0.7, 0.6, 0.4, 0.3, 0.1])  # свежесть

# 2. Создаем сеть
network = BananaFreshnessNetwork()

# 3. Обучаем
print("Обучение началось...")
network.train(X, y, epochs=1000, lr=0.8)

# 4. Тестируем на обучающих данных (проверка)
print("\nПроверка на обучающих данных:")
for i in range(len(X)):
    pred = network.feedforward(X[i])
    print(f"Банан {i+1}: реальная свежесть={y[i]:.1f}, предсказание={pred:.3f}")

# 5. Тестируем на новых данных
test_data = np.array([
    [0.2, 0.15],  # зеленый, твердый - должен быть ~0.85
    [0.55, 0.35], # желтый, средний - должен быть ~0.65
    [0.85, 0.8]   # коричневый, мягкий - должен быть ~0.2
])

print("\nРезультаты тестирования новых бананов:")
for i, x in enumerate(test_data):
    pred = network.feedforward(x)
    print(f"Банан {i+1}: цвет={x[0]:.2f}, мягкость={x[1]:.2f} → предсказанная свежесть={pred:.3f}")
