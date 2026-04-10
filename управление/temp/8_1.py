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

# Словарь для хранения результатов
results = {
    'model': [],
    'accuracy': [],
    'time_per_epoch': [],
    'num_params': []
}

# Список моделей для сравнения
models_to_test = [
    ('AlexNet', models.alexnet(pretrained=True)),
    ('VGG16', models.vgg16(pretrained=True)),
    ('GoogLeNet', models.googlenet(pretrained=True)),
    ('ResNet18', models.resnet18(pretrained=True)),
    #('ResNet34', models.resnet34(pretrained=True)),
    #('ResNet50', models.resnet50(pretrained=True))
]

trained_models = {}
for name, model in models_to_test:
    print(f"\n{'='*40}")
    print(f"Обучение модели: {name}")
    print('='*40)
    
    # Изменяем последний слой для CIFAR-10 (10 классов)
    if name == 'AlexNet':
        model.classifier[6] = nn.Linear(4096, 10)
    elif name == 'VGG16':
        model.classifier[6] = nn.Linear(4096, 10)
    elif name == 'GoogLeNet':
        model.fc = nn.Linear(1024, 10)
    elif name == 'ResNet18':
        model.fc = nn.Linear(512, 10)
    
    model = model.to(device)
    
    # Подсчет параметров
    num_params = sum(p.numel() for p in model.parameters())
    results['model'].append(name)
    results['num_params'].append(num_params)
    print(f"Количество параметров: {num_params:,}")
    
    # Настройка обучения
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.0001)
    
    # Обучение 2 эпох (для ускорения)
    epoch_times = []
    
    for epoch in range(5):
        start_time = time.time()
        loss = train_one_epoch(model, train_loader, criterion, optimizer, device)
        epoch_time = time.time() - start_time
        epoch_times.append(epoch_time)
        
        acc = evaluate_model(model, test_loader, device)
        print(f"Эпоха {epoch+1}: Loss = {loss:.4f}, Accuracy = {acc:.2f}%, Время = {epoch_time:.2f}с")
    
    avg_time = sum(epoch_times) / len(epoch_times)
    final_acc = evaluate_model(model, test_loader, device)
    
    results['time_per_epoch'].append(avg_time)
    results['accuracy'].append(final_acc)
    
    print(f"\nРезультат для {name}:")
    print(f"  Точность: {final_acc:.2f}%")
    print(f"  Среднее время эпохи: {avg_time:.2f}с")
    trained_models[name] = model
    # Этап 5. Визуализация результатов
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))

    # График точности
    axes[0].bar(results['model'], results['accuracy'], color=['blue', 'green', 'orange', 'red'])
    axes[0].set_title('Точность моделей')
    axes[0].set_ylabel('Accuracy (%)')
    axes[0].set_ylim(0, 100)
    for i, v in enumerate(results['accuracy']):
        axes[0].text(i, v + 1, f'{v:.1f}%', ha='center')

    # График времени обучения
    axes[1].bar(results['model'], results['time_per_epoch'], color=['blue', 'green', 'orange', 'red'])
    axes[1].set_title('Время обучения (сек/эпоха)')
    axes[1].set_ylabel('Seconds')

    # График количества параметров
    axes[2].bar(results['model'], [p/1e6 for p in results['num_params']], color=['blue', 'green', 'orange', 'red'])
    axes[2].set_title('Количество параметров')
    axes[2].set_ylabel('Millions')

    plt.tight_layout()
    plt.savefig('comparison_results.png')
    plt.show()

    # Вывод итоговой таблицы
    print("\n" + "="*60)
    print("ИТОГОВАЯ ТАБЛИЦА СРАВНЕНИЯ")
    print("="*60)
    print(f"{'Модель':<12} {'Точность (%)':<12} {'Время/эпоха (с)':<15} {'Параметры (млн)':<15}")
    print("-" * 60)
    for i in range(len(results['model'])):
        print(f"{results['model'][i]:<12} {results['accuracy'][i]:<12.2f} {results['time_per_epoch'][i]:<15.2f} {results['num_params'][i]/1e6:<15.2f}")
    from google.colab import files
    import torch
    import torchvision.transforms as transforms
    from PIL import Image
    import matplotlib.pyplot as plt

    # Загрузка изображения с компьютера
    uploaded = files.upload()

    # Получаем имя загруженного файла
    image_path = list(uploaded.keys())[0]
    print(f"Загружено: {image_path}")

    transform = transforms.Compose([
        transforms.Resize(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                            std=[0.229, 0.224, 0.225])
    ])

    # Загрузка и преобразование изображения
    image = Image.open(image_path).convert('RGB')
    input_tensor = transform(image).unsqueeze(0)  # добавляем batch dimension
    input_tensor = input_tensor.to(device)

    print(f"Форма тензора: {input_tensor.shape}")  # [1, 3, 224, 224]

    # Показываем изображение
    plt.imshow(image)
    plt.axis('off')
    plt.title("Загруженное изображение")
    plt.show()

    model.eval()

    with torch.no_grad():
        outputs = model(input_tensor)
        probabilities = torch.softmax(outputs, dim=1)
        predicted_class = torch.argmax(outputs, dim=1).item()

    print(f"Предсказанный класс: {predicted_class}")
    print(f"Вероятность: {probabilities[0][predicted_class]:.4f}")

    # Классы CIFAR-10
    cifar10_classes = ['airplane', 'automobile', 'bird', 'cat', 'deer', 
                    'dog', 'frog', 'horse', 'ship', 'truck']

    print(f"\nРезультат: {cifar10_classes[predicted_class]}")
    for name, model in trained_models.items():
    model.eval()
    with torch.no_grad():
        outputs = model(input_tensor)
        probabilities = torch.softmax(outputs, dim=1)
        predicted_class= torch.argmax(outputs, dim=1).item()
        confidence = probabilities[0][predicted_class].item()*100
        print(f"Model {name.upper()}:")
        print(f"answer: {cifar10_classes[predicted_class].upper()} (Уверенность: {confidence:.1f}%)")
        print("Alter:")
        for i, class_name in enumerate(cifar10_classes):
        prob = probabilities[0][i].item()*100
        if prob > 5.0 and i != predicted_class:  # выводим только значимые
            print(f"  {class_name}: {prob:.1f}%")

    # Визуализация вероятностей
    plt.figure(figsize=(10, 4))
    plt.bar(cifar10_classes, probabilities[0].cpu().numpy())
    plt.title(f'Предсказание: {cifar10_classes[predicted_class]}')
    plt.ylabel('Вероятность')
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.show()
