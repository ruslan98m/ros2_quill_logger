#!/bin/bash

# Скрипт для автоматической настройки Quill логгера как дефолтного ROS2 логгера

set -e

echo "=== Настройка Quill Logger как дефолтного ROS2 логгера ==="
echo ""

# Цвета для вывода
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

# Проверяем, что мы в ROS2 workspace
if [ ! -f "src/quill_logger/package.xml" ]; then
    echo "Ошибка: Скрипт должен быть запущен из корня ROS2 workspace"
    echo "Убедитесь, что quill_logger находится в src/quill_logger/"
    exit 1
fi

print_info "Собираем пакет quill_logger..."
colcon build --packages-select quill_logger

print_info "Источним workspace..."
source install/setup.bash

print_success "Quill Logger готов к использованию!"
echo ""
echo "=== Способы использования ==="
echo ""
echo "1. Автоматическая замена в коде:"
echo "   Добавьте в начало main():"
echo "   ```cpp"
echo "   auto logger = quill_logger::QuillLogger::getInstance();"
echo "   logger->initialize();"
echo "   ```"
echo ""
echo "2. Глобальная инициализация через launch:"
echo "   ros2 launch quill_logger global_quill_logger.launch.py"
echo ""
echo "3. Ручная инициализация:"
echo "   ros2 run quill_logger quill_logger_init_node"
echo ""
echo "4. Тестирование:"
echo "   ros2 run quill_logger quill_logger_test_node"
echo ""
echo "=== Конфигурация ==="
echo ""
echo "Уровни логирования: DEBUG, INFO, WARN, ERROR, FATAL"
echo "Файл лога по умолчанию: /tmp/ros2_quill_global.log"
echo ""
echo "=== Примеры ==="
echo ""
echo "Запуск с кастомным уровнем логирования:"
echo "ros2 launch quill_logger global_quill_logger.launch.py log_level:=DEBUG"
echo ""
echo "Запуск с кастомным файлом лога:"
echo "ros2 launch quill_logger global_quill_logger.launch.py log_file:=/path/to/custom.log"
echo ""
print_success "Настройка завершена!" 