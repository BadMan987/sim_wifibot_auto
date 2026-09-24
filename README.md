# 🤖 Espace de travail ROS 2 de navigation autonome et de patrouille Wifibot

Ceci est un projet de simulation de navigation autonome et de patrouille pour un robot Wifibot développé sous **ROS 2**. Le projet intègre la simulation physique Gazebo, la localisation et la cartographie RTAB-Map, la planification de chemin globale A* personnalisée, le suivi de chemin Pure Pursuit, ainsi qu'un mécanisme de prévention des collisions et de protection de la sécurité basé sur la caméra de profondeur ZED 2i.

---

## 📂 Structure de l'espace de travail (`src/`)

Le projet contient les paquets fonctionnels principaux suivants :

* **`wifibot_bringup`** : Intégration des fichiers de configuration et de lancement du système.
* **`wifibot_control`** : Configuration liée au contrôle du robot.
* **`wifibot_description`** : Fichiers de modèle URDF/Xacro pour le châssis du robot Wifibot Lab V4 et la caméra de profondeur ZED 2i.
* **`wifibot_gazebo`** : Environnement de simulation Gazebo, modèles de mondes et plugin de conduite différentielle.
* **`wifibot_navigation`** : Données de la carte de navigation (incluant la carte statique `map.yaml` / `.pgm`).
* **`wifibot_patrol`** : Module central des algorithmes de patrouille et de navigation autonome (incluant la planification A*, le suivi Pure Pursuit, l'obtention de la pose et le gardien de sécurité).
* **`wifibot_slam`** : Fichiers de configuration pour la cartographie et la localisation RTAB-Map SLAM.
* **`wifibot_tasks`** : Nœuds d'exécution de tâches complexes.

---

## ✨ Caractéristiques principales

1. **Simulation multi-roues différentielles et de capteurs** : Construction d'un modèle Wifibot à quatre roues différentielles précis via Xacro, équipé d'une caméra de profondeur RVB-D ZED 2i haute précision et d'une IMU virtuelle.
2. **Planification de chemin globale A* efficace (`astar.py`)** :
   * Prend en charge la spécification manuelle des points d'arrivée et de l'angle d'orientation (Goal & Heading) via une interface graphique OpenCV.
   * Prend en charge la replanification dynamique automatique en mode Headless (sans interface), combinée à un bouclier de protection de départ pour éviter les blocages.
3. **Suivi de chemin fluide Pure Pursuit (`path_follower.py`)** :
   * Mise en œuvre d'une distance de regard en avant (Lookahead Distance) adaptative et d'une limitation de vitesse basée sur la courbure pour garantir des virages fluides.
   * Comprend un contrôle en boucle fermée à trois phases : alignement initial, suivi de chemin et alignement précis de l'orientation finale (Goal Yaw Alignment).
4. **Gardien de sécurité intelligent et évitement d'obstacles dynamique (`safety_guard.py`)** :
   * Abonnement en temps réel à la carte de profondeur ZED 2i pour détecter la distance des obstacles à l'avant via une zone d'intérêt (ROI).
   * Déclenchement automatique d'un arrêt d'urgence et appel du script A* pour une **replanification dynamique en ligne (Dynamic Re-planning)** lorsqu'un obstacle bloque la voie trop longtemps.

---

## 🗄️ Téléchargement de la base de données de la carte (`rtabmap.db`)

Étant donné que le fichier de base de données `rtabmap.db` est volumineux (~2 Go), il n'est pas inclus directement dans ce dépôt Git. Si vous souhaitez l'utiliser pour la localisation, veuillez le télécharger via le lien ci-dessous et le placer dans le dossier `src/wifibot_navigation/maps/indoor/` :

* **Téléchargement Google Drive** : [Cliquez ici pour télécharger le fichier rtabmap.db](https://drive.google.com/file/d/10x52tCdV76HGq51t29f5NSDp7LCilcUf/view?usp=sharing)

---

## 🛠️ Installation de l'environnement et du simulateur Gazebo

Avant d'exécuter ce projet, assurez-vous que votre système dispose de **ROS 2 Humble** et des dépendances du simulateur Gazebo.

### 1. Installer Gazebo et les packages de pont ROS 2
Si vous utilisez Ubuntu 22.04 avec ROS 2 Humble, exécutez les commandes suivantes :
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-bridge

```

### 2. Compiler l'espace de travail

Après avoir cloné le dépôt dans votre répertoire `wifibot_ws/src`, compilez-le :

```bash
cd ~/wifibot_ws
colcon build --symlink-install
source install/setup.bash

```

---

## 🗺️ Processus de re-cartographie (Optionnel : si `rtabmap.db` est absent ou l'environnement change)

Si vous n'avez pas téléchargé le fichier `rtabmap.db` ou si vous avez modifié la carte de simulation Gazebo, vous pouvez recréer la carte en suivant ces étapes :

### 1. Lancer l'environnement de simulation Gazebo

```bash
source /opt/ros/humble/setup.bash
source ~/wifibot_ws/install/setup.bash
ros2 launch wifibot_gazebo simulation.launch.py

```

### 2. Lancer le nœud de cartographie RTAB-Map

Ouvrez un deuxième terminal et exécutez la commande de cartographie (sans spécifier d'ancienne base de données, pour en créer une nouvelle) :

```bash
source /opt/ros/humble/setup.bash
source ~/wifibot_ws/install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/camera/zed2i/image_raw \
    depth_topic:=/camera/zed2i/depth/image_raw \
    camera_info_topic:=/camera/zed2i/camera_info \
    frame_id:=base_link \
    odom_topic:=/odom \
    subscribe_odom:=true \
    visual_odometry:=false \
    approx_sync:=true \
    use_sim_time:=true

```

### 3. Contrôler le robot pour mapper et sauvegarder

* Utilisez un nœud de téléopération au clavier pour déplacer le robot partout dans l'environnement de simulation afin de couvrir toutes les zones nécessaires.
* Une fois la cartographie terminée, RTAB-Map enregistre par défaut la base de données dans le répertoire personnel `~/.ros/rtabmap.db`.
* Copiez-la et renommez-la dans le chemin correspondant du projet :
```bash
cp ~/.ros/rtabmap.db ~/wifibot_ws/src/wifibot_navigation/maps/indoor/rtabmap.db

```



---

## 🚀 Étapes détaillées d'exécution et de lancement

Veuillez ouvrir quatre terminaux indépendants et exécuter respectivement les commandes suivantes pour lancer la simulation, la localisation, la planification de chemin et le gardien de sécurité :

### 1. Lancer l'environnement de simulation Gazebo

```bash
source /opt/ros/humble/setup.bash
source ~/wifibot_ws/install/setup.bash
ros2 launch wifibot_gazebo simulation.launch.py

```

### 2. Lancer le nœud de localisation RTAB-Map (chargement de `rtabmap.db` pour la localisation pure)

```bash
source /opt/ros/humble/setup.bash
source ~/wifibot_ws/install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
    database_path:=/home/yz0000/wifibot_ws/src/wifibot_navigation/maps/indoor/rtabmap.db \
    rgb_topic:=/camera/zed2i/image_raw \
    depth_topic:=/camera/zed2i/depth/image_raw \
    camera_info_topic:=/camera/zed2i/camera_info \
    frame_id:=base_link \
    odom_topic:=/odom \
    subscribe_odom:=true \
    visual_odometry:=false \
    approx_sync:=true \
    use_sim_time:=true \
    rtabmap_args:="\
--Mem/IncrementalMemory false \
--Mem/InitWMWithAllNodes true \
--RGBD/StartAtOrigin true \
--Reg/Force3DoF true \
--Optimizer/GravitySigma 0"

```

### 3. Exécuter la planification de chemin A* (spécification du point cible et de l'orientation)

```bash
python3 ~/wifibot_ws/src/wifibot_patrol/wifibot_patrol/astar.py

```

### 4. Lancer le gardien de sécurité et le suivi de chemin (contrôle global du système)

```bash
python3 ~/wifibot_ws/src/wifibot_patrol/wifibot_patrol/safety_guard.py










# 🤖 Wifibot ROS 2 自主导航与巡逻工作空间

这是一个基于 **ROS 2** 开发的 Wifibot 机器人自主导航与巡逻仿真项目。项目集成了 Gazebo 物理仿真、RTAB-Map 定位建图、自定义 A* 全局路径规划、Pure Pursuit 纯追踪路径跟踪以及基于 ZED 2i 深度相机的动态避障与安全防护机制。

---

## 📂 工作空间结构 (`src/`)

项目包含以下核心功能包：

* **`wifibot_bringup`** : 系统的启动引导与配置文件整合。
* **`wifibot_control`** : 机器人控制相关配置。
* **`wifibot_description`** : Wifibot Lab V4 机器人车体与 ZED 2i 深度相机的 URDF/Xacro 模型文件。
* **`wifibot_gazebo`** : Gazebo 仿真环境、世界模型及差速驱动插件。
* **`wifibot_navigation`** : 导航地图数据（包含静态地图 `map.yaml` / `.pgm`）。
* **`wifibot_patrol`** : 核心巡逻与自主导航算法模块（包含 A* 规划、Pure Pursuit 跟踪、姿态获取及安全卫士）。
* **`wifibot_slam`** : RTAB-Map SLAM 建图与定位配置文件。
* **`wifibot_tasks`** : 综合任务执行节点。

---

## ✨ 核心功能特性

1. **多轮差速与传感器仿真** : 基于 Xacro 构建了精细的四轮差速 Wifibot 模型，搭载了高精度 ZED 2i RGB-D 深度相机和虚拟 IMU。
2. **高效的 A* 全局路径规划 (`astar.py`)** :
   * 支持通过 OpenCV 图形界面手动指定目标点与航向角（Goal & Heading）。
   * 支持 Headless（无头模式）自动动态重规划，结合安全保护罩避免死锁。
3. **平滑的纯追踪路径跟踪 (`path_follower.py`)** :
   * 实现了自适应前视距离（Lookahead Distance）与曲率限速，保证过弯平滑。
   * 包含初始对齐、路径跟踪、终点精准姿态对齐（Goal Yaw Alignment）的三阶段闭环控制。
4. **智能安全卫士与动态避障 (`safety_guard.py`)** :
   * 实时订阅 ZED 2i 深度图，通过 ROI 区域检测前方障碍物距离。
   * 当检测到前方障碍物持续阻塞超过阈值时，自动触发紧急停车并调用 A* 脚本进行**动态在线重规划（Dynamic Re-planning）**。

---

## 🗄️ 地图数据库下载 (`rtabmap.db`)

由于 `rtabmap.db` 数据库文件体积较大（约 2GB），未直接包含在本 Git 仓库中。如果你需要直接运行纯定位导航，请通过以下 Google Drive 链接下载该文件，并将其放置在 `src/wifibot_navigation/maps/indoor/` 目录下：

* **Google Drive 下载** : [点击这里下载 rtabmap.db 文件](https://drive.google.com/file/d/10x52tCdV76HGq51t29f5NSDp7LCilcUf/view?usp=sharing)

---

## 🛠️ 环境与 Gazebo 模拟器安装

在运行本项目之前，请确保你的系统已安装 **ROS 2 Humble** 及 **Gazebo 仿真器**相关依赖。

### 1. 安装 Gazebo 及 ROS 2 仿真桥接包
如果你使用的是 Ubuntu 22.04 配合 ROS 2 Humble，可以通过以下命令安装 Gazebo 及其 ROS 2 整合包：
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-bridge

```

### 2. 编译工作空间

将仓库克隆到本地的 `wifibot_ws/src` 目录后，执行编译：

```bash
cd ~/wifibot_ws
colcon build --symlink-install
source install/setup.bash

```

---

## 🗺️ 重新建图流程（可选：适用于无 `rtabmap.db` 或更换环境）

如果你在新的电脑上没有下载现成的 `rtabmap.db`，或者修改了 Gazebo 仿真地图，可以通过以下步骤重新构建地图：

### 1. 启动 Gazebo 仿真环境

```bash
source /opt/ros/humble/setup.bash
source ~/wifibot_ws/install/setup.bash
ros2 launch wifibot_gazebo simulation.launch.py

```

### 2. 启动 RTAB-Map 建图节点

打开第二个终端，运行建图命令（不加载旧数据库，让其自动新建）：

```bash
source /opt/ros/humble/setup.bash
source ~/wifibot_ws/install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/camera/zed2i/image_raw \
    depth_topic:=/camera/zed2i/depth/image_raw \
    camera_info_topic:=/camera/zed2i/camera_info \
    frame_id:=base_link \
    odom_topic:=/odom \
    subscribe_odom:=true \
    visual_odometry:=false \
    approx_sync:=true \
    use_sim_time:=true

```

### 3. 控制机器人建图并保存

* 使用键盘控制节点或通过其他方式驱动机器人在仿真环境中四处移动，确保覆盖所有需要导航的区域。
* 建图完成后，RTAB-Map 默认会将生成的数据库保存在主目录下的 `~/.ros/rtabmap.db`。
* 将其复制并重命名放到项目的对应路径下即可：
```bash
cp ~/.ros/rtabmap.db ~/wifibot_ws/src/wifibot_navigation/maps/indoor/rtabmap.db

```



---

## 🚀 详细运行与启动步骤

请依次打开四个独立的终端，分别执行以下命令来启动仿真、定位、路径规划和安全卫士：

### 1. 启动 Gazebo 仿真环境

```bash
source /opt/ros/humble/setup.bash
source ~/wifibot_ws/install/setup.bash
ros2 launch wifibot_gazebo simulation.launch.py

```

### 2. 启动 RTAB-Map 定位节点（加载 `rtabmap.db` 进行纯定位）

```bash
source /opt/ros/humble/setup.bash
source ~/wifibot_ws/install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
    database_path:=/home/yz0000/wifibot_ws/src/wifibot_navigation/maps/indoor/rtabmap.db \
    rgb_topic:=/camera/zed2i/image_raw \
    depth_topic:=/camera/zed2i/depth/image_raw \
    camera_info_topic:=/camera/zed2i/camera_info \
    frame_id:=base_link \
    odom_topic:=/odom \
    subscribe_odom:=true \
    visual_odometry:=false \
    approx_sync:=true \
    use_sim_time:=true \
    rtabmap_args:="\
--Mem/IncrementalMemory false \
--Mem/InitWMWithAllNodes true \
--RGBD/StartAtOrigin true \
--Reg/Force3DoF true \
--Optimizer/GravitySigma 0"

```

### 3. 运行 A* 路径规划（指定目标点与航向）

```bash
python3 ~/wifibot_ws/src/wifibot_patrol/wifibot_patrol/astar.py

```

### 4. 启动安全卫士与路径跟踪（系统总控）

```bash
python3 ~/wifibot_ws/src/wifibot_patrol/wifibot_patrol/safety_guard.py

```


```
