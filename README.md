# AS2 VLM prototype

Технический план: [TECH_PLAN_AS2_VLM.md](TECH_PLAN_AS2_VLM.md).

Реализация: проверка структуры плана, ограниченный parser русских приказов и backend локальной VLM Qwen2.5-VL 3B через Ollama. По умолчанию используется parser. Все результаты остаются dry-run; обращения к SDK отсутствуют. Настройка модели: [docs/vlm_setup.md](docs/vlm_setup.md).

Запуск из корня проекта с Python 3.10+:

```powershell
python -m as2_vlm "Повернись к двери и подойди к ней"
python -m unittest discover -s tests -v
```

На текущем Windows-ПК можно использовать bundled Python через `run.ps1`:

```powershell
.\run.ps1 --backend ollama "Повернись к двери и подойди к ней"
.\run.ps1 --backend ollama --image C:\frames\room.jpg "Дойди до белой стены"
```

При необходимости путь к Python задаётся переменной `AS2_VLM_PYTHON`. Модель должна быть запущена по инструкции выше.

CLI возвращает `PREVIEW_ONLY` и `executed: false`. Неподдерживаемая инструкция отклоняется с exit code 2. Поддерживаются примеры из технического плана; свободные перефразировки требуют будущего VLM backend.

Система координат будущего контроллера: x вперёд, y влево, положительный yaw влево. Адаптер SDK должен проверить и преобразовать эти направления для AS2.

Сведения об оборудовании: [docs/hardware_inventory.md](docs/hardware_inventory.md). Текущий статус: [docs/progress.md](docs/progress.md).

Проверка локальной среды без обращения к роботу:

```powershell
python -m as2_vlm.preflight
```

Код 1 означает незавершённые проверки (на Windows это ожидаемо). Параметр `--sdk-root` принимает путь к checkout SDK. Сборка и использование отдельного read-only AS2 probe описаны в [docs/sdk_notes.md](docs/sdk_notes.md).
