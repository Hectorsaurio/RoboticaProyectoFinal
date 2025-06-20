# Autonomous Cleaning Robot with ROS 2

Este repositorio contiene el código fuente de un **robot de limpieza autónomo** basado en ROS 2, implementado utilizando una **Raspberry Pi 4**, una **Kinect v1**, y una **Raspberry Pi Pico** con micro-ROS.

---

## Descripción General

El sistema está dividido en dos partes principales:

- **Raspberry Pi 4 (Unidad Principal):**
  - Conecta y gestiona la Kinect v1.
  - Ejecuta el agente de micro-ROS para comunicarse con la Pi Pico.
  - Publica datos de profundidad utilizados por `depthimage_to_laserscan`.
  - Lanza el mapeo SLAM con `slam_toolbox`.
  - Ejecuta la navegación autónoma con el nodo `wall_follower_node`.
  - Visualiza el robot y el entorno con RViz2 mediante un modelo URDF.

- **Raspberry Pi Pico (Unidad de Control):**
  - Se comunica con la Pi 4 vía micro-ROS.
  - Se suscribe al tópico `/cmd_vel` para controlar los motores.
  - Publica datos de encoders en el tópico `/encoders`.

---

## Estructura del Proyecto

```bash
ros2_ws/
├── src/
│   ├── pico_odometry/             # Nodo intermedio que convierte /encoders → /odom
│   ├── my_robot_description/      # URDF, meshes, view_robot.launch
│   ├── wall_follower_slam/        # Nodo para navegación automática por pared derecha
│   ├── my_slam_launch/            # Launch que lanza SLAM + wall follower + RViz
│   └── kinect_ros2/               # Nodo modificado minimal_kinect_node.cpp
🔧 Componentes Clave
minimal_kinect_node.cpp: nodo optimizado que publica solo los tópicos esenciales de profundidad (/depth/image_raw y /depth/camera_info) para reducir el uso de RAM.

depthimage_to_laserscan: transforma la imagen de profundidad en un LaserScan en /scan, utilizado para mapeo SLAM.

pico_odometry: nodo que convierte los datos crudos de encoders (/encoders) a odometría (/odom), ejecutado en la laptop.

slam_toolbox: herramienta utilizada para generar mapas en línea usando /scan y /odom.

wall_follower_node: permite al robot navegar de forma autónoma siguiendo la pared derecha y explorando el entorno.

view_robot.launch.py: visualiza el modelo URDF del robot en RViz2.

Cómo Ejecutar
1. En la Raspberry Pi Pico
Sube el código C de micro-ROS que publica /encoders y se suscribe a /cmd_vel.

2. En la Raspberry Pi 4
bash
Copy
Edit
# Ejecutar el agente de micro-ROS
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

# Lanzar el nodo kinect mínimo
ros2 run kinect_ros2 minimal_kinect_node

# Lanzar depthimage_to_laserscan
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node
3. En la Laptop
bash
Copy
Edit
# Ejecutar nodo que convierte encoders a odometría
ros2 run pico_odometry encoder_to_odom_node

# Lanzar SLAM, navegación automática y visualización
ros2 launch my_slam_launch slam_autonomous.launch.py
Dependencias
ROS 2 Jazzy

micro-ROS Agent

depthimage_to_laserscan

slam_toolbox

freenect (compilado desde fuente)

tf_transformations o transformations (Python)

rclcpp_components, image_transport, cv_bridge, camera_info_manager, sensor_msgs, etc.

Modelo URDF
El modelo del robot fue diseñado en SolidWorks y exportado a URDF. Se incluye en el paquete my_robot_description con archivos de malla y configuraciones para RViz2.

🗺️ Estado del Proyecto
✅ Publicación de encoders
✅ Conversión a odometría
✅ Publicación de /scan desde Kinect
✅ Visualización en RViz
✅ Mapeo SLAM en línea
✅ Navegación autónoma por pared derecha
🔲 Guardado y carga automática de mapas
🔲 Planeamiento de rutas a puntos de interés