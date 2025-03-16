#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <cmath>  

class WallFollower {
public:
    WallFollower() {
        laser_sub = nh.subscribe("/scan", 10, &WallFollower::laserCallback, this);
        velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
        int num_ranges = msg->ranges.size();
        if (num_ranges == 0) return;

        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;

        int index_right = static_cast<int>((-M_PI / 2 - angle_min) / angle_increment);
        int index_front = static_cast<int>((0 - angle_min) / angle_increment);
        int index_front_right = static_cast<int>((-M_PI / 4 - angle_min) / angle_increment); // 45° a la derecha

        if (index_right < 0 || index_front < 0 || index_front_right < 0 ||
            index_right >= num_ranges || index_front >= num_ranges || index_front_right >= num_ranges) {
            return;
        }

        float distance_right = validateRange(msg->ranges[index_right]);
        float distance_front = validateRange(msg->ranges[index_front]);
        float distance_front_right = validateRange(msg->ranges[index_front_right]);

        // Mostrar distancia actual a la pared derecha
        ROS_INFO("Distancia a la pared: %.2f metros", distance_right);

        // Parámetros de distancia
        const float desired_distance = 0.30;   // Distancia ideal a la pared
        const float min_safe_distance = 0.25;  // Distancia mínima antes de corregir
        const float max_safe_distance = 0.35;  // Máxima distancia aceptable para corrección
        const float max_angular_speed = 0.5;   // Límite de giro (ajustado para transiciones suaves)
        const float min_angular_speed = 0.1;   // Giro mínimo para corrección suave
        const float min_linear_speed = 0.06;   // Velocidad mínima 0.03
        const float max_linear_speed = 0.1;    // Velocidad máxima

        float error = desired_distance - distance_right;

        geometry_msgs::Twist cmd;
        cmd.linear.x = max_linear_speed;  // Velocidad base

        // **Evitación de obstáculos frontales (giro progresivo y suave)**
        if (distance_front < 0.5) {
            cmd.linear.x = min_linear_speed;  // Reducir velocidad para girar de manera controlada
            cmd.angular.z = max_angular_speed;  // Giro progresivo a la izquierda
            ROS_WARN("ESQUINA DETECTADA: Girando suavemente a la izquierda.");
        }
        // **Corrección después de una esquina (transición más fluida)**
        else if (distance_front_right > 0.5) {
            cmd.linear.x = min_linear_speed; // Reducir velocidad para ajustar mejor
            cmd.angular.z = -0.5; // Gira suavemente a la derecha después de doblar
            ROS_INFO("RECUPERANDO TRAYECTORIA: Ajustando suavemente a la derecha.");
        }
        // **Corrección cuando está muy cerca de la pared (giro más suave)**
        else if (distance_right < min_safe_distance) {
            cmd.linear.x = min_linear_speed; // Reducir velocidad para evitar choque
            cmd.angular.z = min_angular_speed + 0.2 * (min_safe_distance - distance_right); // Ajuste progresivo más suave
            ROS_WARN("MUY CERCA DE LA PARED: Ajustando a la izquierda de manera controlada.");
        }
        // **Corrección cuando está demasiado lejos de la pared**
        else if (distance_right > max_safe_distance) {
            cmd.linear.x = min_linear_speed; // Reducir velocidad para ajuste fino
            cmd.angular.z = -min_angular_speed - 0.2 * (distance_right - max_safe_distance); // Ajuste progresivo a la derecha
            ROS_INFO("MUY LEJOS DE LA PARED: Ajustando a la derecha.");
        }
        // **Mantener el camino si está en la distancia correcta**
        else {
            cmd.linear.x = max_linear_speed;
            cmd.angular.z = 0.0;
            ROS_INFO("SIGUIENDO PARED: Manteniendo trayectoria.");
        }

        velocity_pub.publish(cmd);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber laser_sub;
    ros::Publisher velocity_pub;

    float validateRange(float range) {
        if (std::isnan(range) || std::isinf(range) || range < 0.01) {
            return 1.0; // Si la medición es inválida, asignamos un valor alto
        }
        return range;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_follower");
    ROS_INFO("Wall Follower Node Started");
    WallFollower wf;
    ros::spin();
    return 0;
}
