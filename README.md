# Seguidor-Pared
Proyecto de ROS para un robot seguidor de paredes

Este proyecto consiste en el desarrollo de un **robot seguidor de pared** utilizando **ROS (Robot Operating System)**. Implementa un nodo que utiliza sensores **LIDAR** para detectar la pared más cercana y seguirla a una distancia óptima.

## Características

✔ Implementación en **C++** utilizando **ROS**  
✔ Control de movimiento basado en **detección de obstáculos con LIDAR**  
✔ Servicio para **buscar la pared más cercana**  
✔ Publicación en `/cmd_vel` para controlar el robot  
✔ Simulación en **Gazebo**  
✔ **Action Server** para registrar odometría  

---

## 📷 Demostración

_Añadir imágenes o GIFs del robot en acción._

---

## ⚙️ Tecnologías Utilizadas

| Tecnología        | Descripción |
|------------------|------------|
| ROS (Noetic)     | Sistema operativo para robots |
| C++             | Lenguaje de programación utilizado |
| Gazebo          | Simulación de entorno robótico |
| RViz            | Visualización de sensores y entorno |
| LIDAR          | Sensor para detección de obstáculos |
| Actionlib       | Implementación de servidores de acciones |

---

## Instalación y Configuración

### Instalación de ROS Noetic

Si no tienes ROS instalado, sigue la [documentación oficial](http://wiki.ros.org/noetic/Installation).

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
