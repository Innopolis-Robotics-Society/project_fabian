# **Project "TO-DO"**

## 🛠 **Installation and Setup**

### **Required Components**

* **Docker** ([Installation Guide](https://docs.docker.com/engine/install/))

### **🚀 Quick Start with Docker**

```bash
git clone git@github.com:Innopolis-Robotics-Society/project_fabian.git
cd project_fabian
docker compose up --build <TO-DO>
```

---

## 🛠 **Development Guide**

### **1. Local Development (without Docker)**

*TO-DO*

### **2. Development in Docker**

Clone the repository and go to its root directory:

```bash
git clone git@github.com:Innopolis-Robotics-Society/project_fabian.git
cd project_fabian
```

Run the container:

```bash
docker compose up --build terminal
```

Connect to the container from other terminals:

```bash
docker compose exec terminal bash
```

### **3. Docker Build**

After making changes to the Dockerfile, you need to build a new image and optionally upload it to the cloud.

Build the image:

```bash
docker build -t image_name -f Dockerfile .
```

Tag the image:

```bash
docker tag image_name fabook/fabian:common
```

Push the image to Docker Hub:

```bash
docker push fabook/fabian:common
```

> \[!IMPORTANT]
> To ensure reproducibility, it is necessary to maintain the Docker image up to date in Docker Hub.
> However, it is not required to run this command every time — only when you are confident in your changes.

> \[!TIP]
> The tag can be anything, but by default, `docker compose` uses the `latest` tag.

---

## **5. Git and Commit Rules**

### 🔹 **Main Tags (Commit Types)**

| Tag          | Description                                                                                  |
| ------------ | -------------------------------------------------------------------------------------------- |
| **feat**     | New feature. Example: `feat: added authentication`                                           |
| **fix**      | Bug fix. Example: `fix: fixed crash on null input`                                           |
| **docs**     | Documentation changes. Example: `docs: updated README.md`                                    |
| **style**    | Formatting changes (spaces, commas). Example: `style: formatted to PEP8`                     |
| **refactor** | Code refactoring (without changing functionality). Example: `refactor: optimized function X` |
| **perf**     | Performance improvement. Example: `perf: reduced load time`                                  |
| **test**     | Changes related to tests. Example: `test: added API coverage`                                |
| **chore**    | Technical tasks (dependencies, configs). Example: `chore: updated webpack`                   |
| **ci**       | CI/CD changes (GitHub Actions, GitLab CI). Example: `ci: added staging deploy`               |
| **build**    | Build system changes. Example: `build: added Dockerfile`                                     |
| **revert**   | Revert a previous commit. Example: `revert: reverted commit 123abc`                          |

### 🔹 **Additional Rules**

1. **Commit message** should be clear and concise.

   * ❌ Bad: `fix: bug`
   * ✅ Good: `fix: fixed form submission error`

2. **Commit body** (optional) — detailed description of changes.

   ```
   fix: fixed memory leak in module X  

   The leak occurred due to unclosed DB connections during long sessions.  
   Added `cleanup()` to properly release resources.  
   ```

3. **Footer** (optional) — links to issues, breaking changes.

   ```
   feat: added WebSocket support  

   BREAKING CHANGE: Deprecated `/chat` API is no longer supported.  
   Closes #123  
   ```

---

## 📌 **How to Push Changes to a Separate Branch**

```bash
# 1. Create a new branch  
git checkout -b <branch_name>  
```

# 2. Check the current branch

```bash
git branch  
```

# 3. Add changes

```bash
git add .  
```

# 4. Create a commit

```bash
git commit -m "<type>: <description>"  
```

# 5. Push the branch to GitHub

```bash
git push -u origin <branch_name>  
```
