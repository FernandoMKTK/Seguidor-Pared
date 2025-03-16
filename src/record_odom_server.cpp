#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <wall_follower/OdomRecordAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>
#include <vector>

class OdomRecordAction {
protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<wall_follower::OdomRecordAction> as;
    std::string action_name;
    ros::Subscriber odom_sub;
    std::vector<geometry_msgs::Point32> odom_list;
    float total_distance;
    float accumulated_yaw_change;
    float previous_yaw;
    bool recording;
    bool first_reading;
    geometry_msgs::Point32 last_position;

public:
    OdomRecordAction(std::string name) 
        : as(nh, name, boost::bind(&OdomRecordAction::executeCB, this, _1), false),
          action_name(name), total_distance(0.0), accumulated_yaw_change(0.0),
          recording(false), first_reading(true) {
        odom_sub = nh.subscribe("/odom", 10, &OdomRecordAction::odomCallback, this);
        as.start();
        ROS_INFO("Servidor de acciones '%s' iniciado.", action_name.c_str());
    }

    void executeCB(const wall_follower::OdomRecordGoalConstPtr &goal) {
        ROS_INFO("Inicio del registro de odometría...");
        recording = true;
        odom_list.clear();
        total_distance = 0.0;

        ros::Rate rate(1); // 1 Hz - Registrar odometría cada segundo
        ros::Publisher stop_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        while (ros::ok() && recording) {
            wall_follower::OdomRecordFeedback feedback;
            feedback.current_total = total_distance;
            as.publishFeedback(feedback);

            if (as.isPreemptRequested()) {
                ROS_WARN("Acción preemptada por el cliente.");
                as.setPreempted();
                return;
            }

            if (hasCompletedLoop()) {
                recording = false;
                ROS_INFO("¡Vuelta completa detectada! Deteniendo el robot...");

                // Publicar mensaje de parada
                ros::Duration(1.0).sleep();
                geometry_msgs::Twist stop_cmd;
                stop_pub.publish(stop_cmd);
                ROS_INFO("Robot detenido después de completar la vuelta.");

                // Crear y llenar el resultado
                wall_follower::OdomRecordResult result;
                result.list_of_odoms = odom_list;
                result.current_total = total_distance;

                // Imprimir la distancia total recorrida antes de enviarla al cliente
                
                ROS_INFO("Enviando lista de odometrías registradas:");

                for (size_t i = 0; i < odom_list.size(); i++) {
                    ROS_INFO("[%lu] x: %.2f, y: %.2f, yaw: %.2f", i, 
                            odom_list[i].x, odom_list[i].y, odom_list[i].z);
                }

                ROS_INFO("Distancia total recorrida: %.2f metros", total_distance);

                // Enviar resultado al cliente
                as.setSucceeded(result);
                return;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }



    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        if (!recording) return;

        geometry_msgs::Point32 current_position;
        current_position.x = msg->pose.pose.position.x;
        current_position.y = msg->pose.pose.position.y;

        double roll, pitch, yaw;
        tf::Quaternion q(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_position.z = yaw;

        if (!odom_list.empty()) {
            float dx = current_position.x - last_position.x;
            float dy = current_position.y - last_position.y;
            total_distance += sqrt(dx * dx + dy * dy);

            // Cálculo de cambio de yaw con manejo de discontinuidad en ±π
            float delta_yaw = yaw - previous_yaw;
            if (delta_yaw > M_PI) {
                delta_yaw -= 2 * M_PI;
            } else if (delta_yaw < -M_PI) {
                delta_yaw += 2 * M_PI;
            }

            accumulated_yaw_change += delta_yaw;
        }

        if (first_reading) {
            previous_yaw = yaw;
            first_reading = false;
        }

        odom_list.push_back(current_position);
        last_position = current_position;
        previous_yaw = yaw;
    }

    bool hasCompletedLoop() {
        if (odom_list.size() < 10) return false;  // No evaluar si hay pocos puntos registrados

        geometry_msgs::Point32 start_pos = odom_list.front(); // Primer punto registrado
        geometry_msgs::Point32 current_pos = odom_list.back(); // Última posición registrada

        float distance_to_start = sqrt(pow(current_pos.x - start_pos.x, 2) + pow(current_pos.y - start_pos.y, 2));

        // Si el robot está cerca del punto inicial y ha recorrido suficiente distancia
        return (distance_to_start < 0.2 && total_distance > 3.0);  
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "record_odom_server");
    OdomRecordAction record_odom("record_odom");
    ros::spin();
    return 0;
}
