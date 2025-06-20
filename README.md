# RoboticaProyectoFinal
Repositorio que contiene los codigos usados en la implementacion de un robot de limpieza usando ros2

Se usa una RB Pi 4 para la conexion con la Kinect y con el agente de microros en la RB Pi Pico
Se usaron ros2_kinect_ws junto con freenect compilado desde fuente para el manejo de la Kinectv1.
Para reducir el consumo de RAM se creo el nodo: minimal_kinect_node.cpp que se encarga de mandar simplemente
los datos usados para los topicos de profundidad.
Los mismos topicos de profundidad son usados por depth_to_laserscan para generar el topico /scan que se utiliza
en el mapeo son slam.
En la RB Pi Pico se subscribe al topico /cmd_vel y publica en el nodo /encoders.
Usando un nodo intermedio en la laptop se usa /encoders para calcular /odom.
Usando SLAM toolbox se planea usar /scan y /odom para generar un mapa_online.
Y con el nodo wall_follower_node se envia datos por el nodo /cmd_vel para que recorra el entorno por derecha
automaticamente.
Tambien se creo y cargo un archivo URDF importado desde solidworks el cual usando el launch view_robot de
my_robot_description genera la visualizacion en RViz.
