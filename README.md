# **Проект "TO-DO"**  

## 🛠 **Установка и настройка**  

### **Необходимые компоненты**  
- **Docker** ([Инструкция по установке](https://docs.docker.com/engine/install/))  

### **🚀 Быстрый старт с Docker**  

```bash
git clone git@github.com:Robosoft-MIK-2025/maneuver.git
cd maneuver
docker compose up --build <TO-DO>
```

---

## 🛠 **Руководство по разработке**  

### **1. Локальная разработка (без Docker)**  
*TO-DO*  

### **2. Разработка в Docker**  

Скачайте репозиторий и перейдидте в его корень
```bash
git clone git@github.com:Robosoft-MIK-2025/maneuver.git
cd maneuver
```

Запустите контейнер:  
```bash
docker compose up --build terminal
```

Подключитесь к контейнеру из других терминалов:  
```bash
docker compose exec terminal bash
```

### **3. Сборка Docker**  

После внесения изменений в Dockerfile необходимо собрать новый образ и загрузить его в облако (опционально).  

Сборка образа:  
```bash
docker build -t image_name -f Dockerfile .
```

Тег образа:  
```bash
docker tag image_name fabook/mik:common
```

Загрузка образа в Docker Hub:  
```bash
docker push fabook/mik:common
```

> [!IMPORTANT]  
> Для обеспечения воспроизводимости необходимо поддерживать актуальную версию Docker-образа в Docker Hub.  
> Однако не обязательно выполнять эту команду при каждой сборке, только когда вы уверены в своих изменениях

> [!TIP]  
> Тег может быть любым, но по умолчанию `docker compose` использует тег `latest`.  

---

## **5. Правила работы с Git и коммитами**  

### 🔹 **Основные теги (типы коммитов)**  
| Тег         | Описание                                                                 |
|-------------|--------------------------------------------------------------------------|
| **feat**    | Новая функциия. Пример: `feat: добавлена аутентификация`        |
| **fix**     | Исправление ошибки. Пример: `fix: исправлен краш при null-вводе`        |
| **docs**    | Изменения в документации. Пример: `docs: обновлен README.md`            |
| **style**   | Изменения форматирования (пробелы, запятые). Пример: `style: форматирование по PEP8` |
| **refactor**| Рефакторинг кода (без изменения функционала). Пример: `refactor: оптимизация функции X` |
| **perf**    | Улучшение производительности. Пример: `perf: уменьшено время загрузки` |
| **test**    | Изменения, связанные с тестами. Пример: `test: добавлено покрытие API` |
| **chore**   | Технические задачи (зависимости, конфиги). Пример: `chore: обновление webpack` |
| **ci**      | Изменения CI/CD (GitHub Actions, GitLab CI). Пример: `ci: добавлен деплой на staging` |
| **build**   | Изменения в системе сборки. Пример: `build: добавлен Dockerfile`       |
| **revert**  | Отмена предыдущего коммита. Пример: `revert: отмена коммита 123abc`    |

### 🔹 **Дополнительные правила**  
1. **Сообщение** должно быть четким и лаконичным.  
   - ❌ Плохо: `fix: баг`  
   - ✅ Хорошо: `fix: исправлена ошибка отправки формы`  

2. **Тело коммита** (опционально) — подробное описание изменений.  
   ```  
   fix: устранена утечка памяти в модуле X  

   Утечка возникала из-за незакрытых соединений с БД при долгих сессиях.  
   Добавлен `cleanup()` для корректного освобождения ресурсов.  
   ```  

3. **Футер** (опционально) — ссылки на задачи, критические изменения.  
   ```  
   feat: добавлена поддержка WebSocket  

   BREAKING CHANGE: Устаревший API `/chat` больше не поддерживается.  
   Closes #123  
   ```  

---

## 📌 **Как загрузить изменения в отдельную ветку**  

```bash
# 1. Создать новую ветку  
git checkout -b <имя_ветки>  
```

# 2. Проверить текущую ветку  
```bash
git branch  
```

# 3. Добавить изменения  
```bash
git add .  
```

# 4. Создать коммит  
```bash
git commit -m "<тип>: <описание>"  
```

# 5. Загрузить ветку на GitHub  
```bash
git push -u origin <имя_ветки>  
```

### Installing the Docker image with ROS2 and PX4 preinstalled
An instruction by George



**WARNING: This is a work in progress. Run for your own peril!**

#### 1. Installing the image

Before installing,
- Ensure you have [Docker](https://docs.docker.com/engine/install/) installed (`docker -v` should output docker version or show an error if docker is not installed)
- Ensure you're using the correct git branch: `EgorBranch`. Files in other branches may differ. (The code below uses the correct branch)
- Use the Ethernet connection, it's 1000MiB capable and is WAY faster than WiFi (in Dorm 5 at least)
- (optional and potentially dangerous) Run `docker system prune -a -f` remove all user files in docker to ensure that no previous attempts/containers are interfering with the install. **WARNING: THIS WILL ERASE ALL DOCKER CONTAINERS AND FILES YOU HAVE ON YOUR LOCAL MACHINE**


Then run:
```
git clone -b EgorBranch https://github.com/Robosoft-MIK-2025/maneuver.git
cd maneuver
docker compose up terminal
```

Code explanation:
- `git clone -b EgorBranch https://github.com/Robosoft-MIK-2025/maneuver.git` downloads latest files from MIK github repo from `EgorBranch` branch
- `cd maneuver` switch to the `maneuver` folder
- `docker compose up terminal` runs the container (also downloads all the dependencies on the first run)

#### 2. Starting PX4 sim
1. Ensure that the docker container is running (a terminal running `docker compose up terminal` is open)
2. Open two more terminal windows and connect them to the docker container using `docker compose exec terminal bash`
3. In one terminal, start the agent with settings for connecting to the uXRCE-DDS client running on the simulator: `MicroXRCEAgent udp4 -p 8888`
4. In the other terminal, start the simulator:  `cd /home/mobile/PX4-Autopilot && make px4_sitl gz_x500`