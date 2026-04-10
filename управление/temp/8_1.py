import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, transforms, models
from torch.utils.data import DataLoader, Subset
import matplotlib.pyplot as plt
import numpy as np
import time


# 1. ПРОВЕРКА И ПОДКЛЮЧЕНИЕ GPU
print("Проверка GPU...")
print(f"CUDA доступен: {torch.cuda.is_available()}")


if torch.cuda.is_available():
    print(f"✅ GPU подключен: {torch.cuda.get_device_name(0)}")
    device = torch.device('cuda')
else:
    print("⚠️ GPU не подключен. Выполните: Среда выполнения → Сменить тип → T4 GPU")
    print("Продолжаем на CPU (будет медленно)...")
    device = torch.device('cpu')


print(f"Используется устройство: {device}\n")

# Преобразования для изображений
transform = transforms.Compose([
    transforms.Resize(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                         std=[0.229, 0.224, 0.225])
])


# Загрузка CIFAR-10 (10 классов: самолеты, машины, птицы, кошки и т.д.)
train_dataset = datasets.CIFAR10(root='./data', train=True, download=True, transform=transform)
test_dataset = datasets.CIFAR10(root='./data', train=False, download=True, transform=transform)


# Используем подмножество для ускорения (2000 тренировочных, 500 тестовых)
train_subset = Subset(train_dataset, range(2000))
test_subset = Subset(test_dataset, range(500))


# Создаем загрузчики
train_loader = DataLoader(train_subset, batch_size=32, shuffle=True, num_workers=2)
test_loader = DataLoader(test_subset, batch_size=32, shuffle=False, num_workers=2)


print(f"Тренировочных образцов: {len(train_subset)}")
print(f"Тестовых образцов: {len(test_subset)}")
print(f"Классы: {train_dataset.classes}")

def evaluate_model(model, test_loader, device):
    """Вычисляет точность модели на тестовых данных"""
    model.eval()
    correct = 0
    total = 0
    
    with torch.no_grad():
        for images, labels in test_loader:
            images, labels = images.to(device), labels.to(device)
            outputs = model(images)
            _, predicted = torch.max(outputs, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
    
    accuracy = 100 * correct / total
    return accuracy


def train_one_epoch(model, train_loader, criterion, optimizer, device):
    """Обучает модель одну эпоху"""
    model.train()
    running_loss = 0.0
    
    for images, labels in train_loader:
        images, labels = images.to(device), labels.to(device)
        
        optimizer.zero_grad()
        outputs = model(images)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        
        running_loss += loss.item()
    
    return running_loss / len(train_loader)

# 1. Список моделей ResNet разной глубины
resnet_variants = [
    ('ResNet18', models.resnet18(pretrained=True)),
    ('ResNet34', models.resnet34(pretrained=True)),
    ('ResNet50', models.resnet50(pretrained=True))
]

# 2. Словарь для хранения результатов анализа глубины
resnet_results = {
    'model': [],
    'accuracy': [],
    'time_per_epoch': [],
    'num_params': []
}

# 3. Цикл обучения и оценки
for name, model in resnet_variants:
    print(f"\nАнализ глубины: {name}")
    
    # Адаптация последнего слоя под 10 классов CIFAR-10
    num_ftrs = model.fc.in_features
    model.fc = nn.Linear(num_ftrs, 10)
    model = model.to(device)
    
    # Параметры обучения
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    
    # Обучение (2 эпохи для сравнения)
    start_time = time.time()
    for epoch in range(2):
        train_one_epoch(model, train_loader, criterion, optimizer, device)
    
    avg_time = (time.time() - start_time) / 2
    acc = evaluate_model(model, test_loader, device)
    num_params = sum(p.numel() for p in model.parameters())
    
    resnet_results['model'].append(name)
    resnet_results['accuracy'].append(acc)
    resnet_results['time_per_epoch'].append(avg_time)
    resnet_results['num_params'].append(num_params)
    
    print(f"Результат {name}: Точность = {acc:.2f}%, Параметров = {num_params/1e6:.2f}M")
