#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <wall_follower/FindWall.h>  // Incluir el servicio FindWall.srv
#include <limits>
#include <cmath>

class FindWallServer {
public:
    FindWallServer() {
        // Inicializar el servicio de ROS
        service = nh.advertiseService("find_wall", &FindWallServer::findWallCallback, this);
        // Suscribirse al escáner láser
        laser_sub = nh.subscribe("/scan", 10, &FindWallServer::laserCallback, this);
        // Publicador para mover el robot
        velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // Variable para almacenar la distancia mínima detectada
        min_distance = std::numeric_limits<float>::max();
        min_index = -1;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Guardar los valores del escáner láser
        laser_data = *msg;
        
        // Buscar el rayo con la menor distancia
        min_distance = std::numeric_limits<float>::max();
        min_index = -1;

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float range = validateRange(msg->ranges[i]);
            if (range < min_distance) {
                min_distance = range;
                min_index = i;
            }
        }
    }

    bool findWallCallback(wall_follower::FindWall::Request& req, wall_follower::FindWall::Response& res) {
        ROS_INFO("Servicio find_wall llamado: buscando la pared más cercana.");

        if (laser_data.ranges.empty()) {
            ROS_WARN("No hay datos de láser disponibles.");
            return false;
        }

        // 1️⃣ **Girar el robot hasta que el frente mire la pared más cercana**
        if (min_index == -1) {
            ROS_WARN("No se encontró un rayo láser con una distancia válida.");
            return false;
        }

        float angle_to_turn = (min_index * laser_data.angle_increment) + laser_data.angle_min;
        ROS_INFO("Orientando hacia la pared: girando hasta %f radianes", angle_to_turn);

        geometry_msgs::Twist cmd;
        cmd.angular.z = 0.5;  // Velocidad angular para girar

        while (std::abs(static_cast<int>(min_index) - static_cast<int>(laser_data.ranges.size()) / 2) > 10) {

            velocity_pub.publish(cmd);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }

        cmd.angular.z = 0.0;
        velocity_pub.publish(cmd);
        ROS_INFO("Robot alineado con la pared.");

        // 2️⃣ **Moverse hacia la pared hasta que la distancia frontal sea < 0.3m**
        cmd.linear.x = 0.1;  // Avanzar lentamente
        while (validateRange(laser_data.ranges[laser_data.ranges.size() / 2]) > 0.3) {
            velocity_pub.publish(cmd);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }

        cmd.linear.x = 0.0;
        velocity_pub.publish(cmd);
        ROS_INFO("Robot ha alcanzado la pared.");

        // 3️⃣ **Girar para que el rayo 270° mire la pared**
        cmd.angular.z = 0.5;
        while (validateRange(laser_data.ranges[270]) > 0.3) {
            velocity_pub.publish(cmd);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }

        cmd.angular.z = 0.0;
        velocity_pub.publish(cmd);
        ROS_INFO("Robot alineado para seguir la pared.");

        // 4️⃣ **Finalizar el servicio con éxito**
        res.wallfound = true;
        return true;
    }

private:
    ros::NodeHandle nh;
    ros::ServiceServer service;
    ros::Subscriber laser_sub;
    ros::Publisher velocity_pub;
    sensor_msgs::LaserScan laser_data;
    float min_distance;
    int min_index;

    float validateRange(float range) {
        if (std::isnan(range) || std::isinf(range) || range < 0.01) {
            return std::numeric_limits<float>::max();
        }
        return range;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_wall_server");
    ROS_INFO("Servidor de servicio find_wall iniciado.");
    FindWallServer server;
    ros::spin();
    return 0;
}
