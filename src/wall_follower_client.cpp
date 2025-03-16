#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <cmath>
#include <wall_follower/FindWall.h>  
#include <actionlib/client/simple_action_client.h>
#include <wall_follower/OdomRecordAction.h>

class WallFollower {
public:
    WallFollower() {
        // Cliente para encontrar la pared
        client = nh.serviceClient<wall_follower::FindWall>("find_wall");

        // Esperar hasta recibir datos del LIDAR
        ROS_INFO("Esperando datos del LIDAR antes de llamar a find_wall...");
        ros::Rate rate(10);  
        bool lidar_ready = false;

        while (ros::ok() && !lidar_ready) {
            sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh, ros::Duration(5.0));
            if (msg) {
                lidar_ready = true;
                ROS_INFO("Datos del LIDAR recibidos. Ahora llamando al servicio find_wall...");
            } else {
                ROS_WARN("Aún no hay datos del LIDAR. Esperando...");
            }
            rate.sleep();
        }

        // Llamar al servicio find_wall
        ROS_INFO("Esperando el servicio find_wall...");
        client.waitForExistence();
        
        wall_follower::FindWall srv;
        bool wall_found = false;

        while (ros::ok() && !wall_found) {
            if (client.call(srv)) {
                if (srv.response.wallfound) {
                    ROS_INFO("Pared encontrada. Iniciando seguimiento...");
                    wall_found = true;
                } else {
                    ROS_WARN("No se detectó una pared. Intentando nuevamente...");
                    ros::Duration(1.0).sleep();
                }
            } else {
                ROS_ERROR("Fallo al llamar al servicio find_wall. Reintentando...");
                ros::Duration(1.0).sleep();
            }
        }

        // Iniciar el seguimiento de pared antes de enviar la meta de odometría
        laser_sub = nh.subscribe("/scan", 10, &WallFollower::laserCallback, this);
        velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Enviar la meta al servidor de acciones record_odom **sin esperar el resultado**
        action_client = new actionlib::SimpleActionClient<wall_follower::OdomRecordAction>("record_odom", true);
        ROS_INFO("Esperando el servidor de acciones record_odom...");
        action_client->waitForServer();

        wall_follower::OdomRecordGoal goal;
        action_client->sendGoal(goal,
            boost::bind(&WallFollower::doneCb, this, _1, _2), // Callback cuando termine la acción
            actionlib::SimpleActionClient<wall_follower::OdomRecordAction>::SimpleActiveCallback(),
            actionlib::SimpleActionClient<wall_follower::OdomRecordAction>::SimpleFeedbackCallback());

        ROS_INFO("Meta enviada al servidor de acciones. Comenzando registro de odometría.");
    }

    // Función que se ejecuta cuando la acción termina
    void doneCb(const actionlib::SimpleClientGoalState &state, const wall_follower::OdomRecordResultConstPtr &result) {
        ROS_INFO("El servidor de odometría finalizó correctamente.");
        
        // Verificar si se recibió un resultado válido
        if (!result) {
            ROS_ERROR("No se recibió un resultado del servidor de odometría.");
            return;
        }

        

        // Imprimir la lista de posiciones registradas
        ROS_INFO("Lista de odometrías registradas:");
        for (size_t i = 0; i < result->list_of_odoms.size(); i++) {
            ROS_INFO("[%lu] x: %.2f, y: %.2f, yaw: %.2f", i, 
                    result->list_of_odoms[i].x, 
                    result->list_of_odoms[i].y, 
                    result->list_of_odoms[i].z);
        }

        // Publicar mensaje de detención en /cmd_vel
        geometry_msgs::Twist stop_cmd;
        velocity_pub.publish(stop_cmd);
        ROS_INFO("Robot detenido después de completar la vuelta.");

        // Imprimir la distancia total recorrida
        ROS_INFO("Distancia total recorrida: %.2f metros", result->current_total);

        ros::shutdown();  // Detener el nodo
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
        const float desired_distance = 0.30;
        const float min_safe_distance = 0.25;
        const float max_safe_distance = 0.35;
        const float max_angular_speed = 0.5;
        const float min_angular_speed = 0.1;
        const float min_linear_speed = 0.06;
        const float max_linear_speed = 0.1;

        float error = desired_distance - distance_right;

        geometry_msgs::Twist cmd;
        cmd.linear.x = max_linear_speed;  // Velocidad base

        // Corrección de trayectoria
        if (distance_front < 0.5) {
            cmd.linear.x = min_linear_speed;
            cmd.angular.z = max_angular_speed;
            ROS_WARN("ESQUINA DETECTADA: Girando suavemente a la izquierda.");
        } else if (distance_front_right > 0.5) {
            cmd.linear.x = min_linear_speed;
            cmd.angular.z = -0.5;
            ROS_INFO("RECUPERANDO TRAYECTORIA: Ajustando suavemente a la derecha.");
        } else if (distance_right < min_safe_distance) {
            cmd.linear.x = min_linear_speed;
            cmd.angular.z = min_angular_speed + 0.2 * (min_safe_distance - distance_right);
            ROS_WARN("MUY CERCA DE LA PARED: Ajustando a la izquierda.");
        } else if (distance_right > max_safe_distance) {
            cmd.linear.x = min_linear_speed;
            cmd.angular.z = -min_angular_speed - 0.2 * (distance_right - max_safe_distance);
            ROS_INFO("MUY LEJOS DE LA PARED: Ajustando a la derecha.");
        } else {
            cmd.linear.x = max_linear_speed;
            cmd.angular.z = 0.0;
            ROS_INFO("SIGUIENDO PARED: Manteniendo trayectoria.");
        }

        velocity_pub.publish(cmd);
    }

private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    ros::Subscriber laser_sub;
    ros::Publisher velocity_pub;
    actionlib::SimpleActionClient<wall_follower::OdomRecordAction> *action_client; // SE AGREGA AQUÍ

    float validateRange(float range) {
        if (std::isnan(range) || std::isinf(range) || range < 0.01) {
            return 1.0; // Si la medición es inválida, asignamos un valor alto
        }
        return range;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "wall_follower_client");
    ROS_INFO("Nodo Cliente de Wall Follower Iniciado");

    WallFollower wf;
    ros::spin();
    return 0;
}
