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

## 🗄️ Explication du fichier de base de données `rtabmap.db`

Lors du lancement de la localisation, vous utiliserez un fichier central : `rtabmap.db`. Son rôle et ses caractéristiques sont les suivants :
* **Qu'est-ce que `rtabmap.db`** : Il s'agit du fichier de base de données complet pour RTAB-Map SLAM, qui stocke les points caractéristiques visuels/laser accumulés pendant le processus de cartographie, le nuage de points 3D, les images clés historiques et les contraintes d'optimisation de graphe (Graph Optimization).
* **Quel est son rôle** : Pendant la phase de navigation autonome, le système charge ce fichier `.db` existant comme **fond de carte global (Mode de localisation)**, permettant au robot de se positionner précisément dans le système de coordonnées de la carte établie précédemment grâce à la correspondance des caractéristiques visuelles.
* **Pourquoi n'est-il pas synchronisé sur GitHub** : Étant donné que la taille du fichier `.db` est généralement très volumineuse (plusieurs gigaoctets) et qu'il s'agit d'un produit de données/cache d'exécution, il a été exclu du suivi Git. Si vous devez le reproduire, vous pouvez le régénérer via le processus de cartographie initial ou placer votre propre base de données locale dans le chemin correspondant.

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

```

---

```

```
好的！我为你把刚才的启动步骤和说明重新整理了一遍，并在其中加入了关于 **`rtabmap.db` 数据库文件** 的详细解释，方便你在查看或分享项目时理解它的作用。

你可以把以下内容更新到你的 `README.md` 文件中：

---

# 🤖 Wifibot ROS 2 Autonomous Navigation & Patrol Workspace

这是一个基于 **ROS 2** 开发的 Wifibot 机器人自主导航与巡逻仿真项目。项目集成了 Gazebo 物理仿真、RTAB-Map 定位建图、自定义 A* 全局路径规划、Pure Pursuit 纯追踪路径跟踪以及基于 ZED 2i 深度相机的动态避障与安全防护机制。

---

## 📂 工作空间结构 (`src/`)

项目包含以下核心功能包：

* **`wifibot_bringup`**: 系统的启动引导与配置文件整合。
* **`wifibot_control`**: 机器人控制相关配置。
* **`wifibot_description`**: Wifibot Lab V4 机器人车体与 ZED 2i 深度相机的 URDF/Xacro 模型文件。
* **`wifibot_gazebo`**: Gazebo 仿真环境、世界模型及差速驱动插件。
* **`wifibot_navigation`**: 导航地图数据（包含静态地图 `map.yaml` / `.pgm`）。
* **`wifibot_patrol`**: 核心巡逻与自主导航算法模块（包含 A* 规划、Pure Pursuit 跟踪、姿态获取及安全卫士）。
* **`wifibot_slam`**: RTAB-Map SLAM 建图与定位配置文件。
* **`wifibot_tasks`**: 综合任务执行节点。

---

## ✨ 核心功能特性

1. **多轮差速与传感器仿真**：基于 Xacro 构建了精细的四轮差速 Wifibot 模型，搭载了高精度 ZED 2i RGB-D 深度相机和虚拟 IMU。
2. **高效的 A* 全局路径规划 (`astar.py`)**：
* 支持通过 OpenCV 图形界面手动指定目标点与航向角（Goal & Heading）。
* 支持 Headless（无头模式）自动动态重规划，结合安全保护罩避免死锁。


3. **平滑的纯追踪路径跟踪 (`path_follower.py`)**：
* 实现了自适应前视距离（Lookahead Distance）与曲率限速，保证过弯平滑。
* 包含初始对齐、路径跟踪、终点精准姿态对齐（Goal Yaw Alignment）的三阶段闭环控制。


4. **智能安全卫士与动态避障 (`safety_guard.py`)**：
* 实时订阅 ZED 2i 深度图，通过 ROI 区域检测前方障碍物距离。
* 当检测到前方障碍物持续阻塞超过阈值时，自动触发紧急停车并调用 A* 脚本进行**动态在线重规划（Dynamic Re-planning）**。



---

## 🗄️ 关于 `rtabmap.db` 数据库文件的说明

在启动定位时，你会用到一个核心文件：`rtabmap.db`。它的作用和特性如下：

* **什么是 `rtabmap.db**`：它是 RTAB-Map SLAM 的全量数据库文件，内部保存了建图过程中积累的视觉/激光特征点、3D 地图点云、历史关键帧以及图优化（Graph Optimization）的约束关系。
* **它的作用是什么**：在自主导航阶段，系统会加载这个已有的 `.db` 文件作为**全局地图底图（Localization Mode）**，让机器人通过视觉特征匹配精准把自己定位在之前建好的地图坐标系中。
* **为什么不同步上传到 GitHub**：由于 `.db` 文件体积通常非常庞大（动辄数 GB），属于运行时的缓存/数据产物，因此它已被排除在 Git 追踪之外。如果需要复现，可以通过前期建图过程重新生成，或者将你本地训练好的数据库放在对应的路径下。

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

---

## 📄 License

本项目采用 [MIT License](https://www.google.com/search?q=LICENSE&utm_source=gemini) 开源协议。
