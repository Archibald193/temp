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