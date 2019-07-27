#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h>

#define sBUFFERSIZE 10
unsigned char s_buffer[sBUFFERSIZE];
std::vector<uint8_t> buf;
std::vector<uint8_t> r_buf;

serial::Serial ser;
ros::Subscriber vel_sub;
ros::Publisher odom_pub;

ros::Time current_time, last_time;

float x=0,y=0,th=0;


void data_analysis(std::vector<uint8_t> csum){


    float Vx=0,Vy=0,W=0;
    float a=0.24,b=0.22;
    float WH1=0,WH2=0,WH3=0,WH4=0;

    nav_msgs::Odometry odom;
    static tf::TransformBroadcaster odom_broadcaster;

    if(csum[2] != 0){
            if(csum[2] == 1){
                WH4 = (float)csum[1];
            }else{
                WH4 = ((float)csum[1])*(-1);
            }
     }
        if(csum[4] != 0){
            if(csum[4] == 1){
                WH2 = (float)csum[3];
            }else{
                WH2 = ((float)csum[3])*-1;
            }
        }
        if(csum[6] != 0){
            if(csum[6] == 1){
                WH1 = (float)csum[5];
            }else{
                WH1 = ((float)csum[5])*(-1);
            }
        }
        if(csum[8] != 0){
            if(csum[8] == 1){
                WH3 = (float)csum[7];
            }else{
                WH3 = ((float)csum[7])*(-1);
            }
        }


//            std::cout << WH1  <<  "    "<< WH2  <<  "    "<< WH3  <<  "    "<< WH4  <<  std::endl;

            //////////// Odom //////////////

            Vx = (WH1+WH2+WH3+WH4)/226.67;
            Vy = (WH1-WH2-WH3+WH4)/226.67;
            W =  (WH1-WH2+WH3-WH4)/(a+b)/226.67;

            current_time = ros::Time::now();

            double dt = (current_time - last_time).toSec();;
            double delta_x = (Vx * cos(th) - Vy * sin(th)) * dt;
            double delta_y = (Vx * sin(th) + Vy * cos(th)) * dt;
            double delta_th = W * dt;




            x += delta_x;
            y += delta_y;
            th += delta_th;

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_footprint";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            odom_broadcaster.sendTransform(odom_trans);

            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";

            //set the position
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            //set the velocity
            odom.child_frame_id = "base_footprint";
            odom.twist.twist.linear.x = Vx;
            odom.twist.twist.linear.y = Vy;
            odom.twist.twist.angular.z = W;

            odom_pub.publish(odom);
//            count = count + 1;
//            std::cout << "dt: " << dt << "  dx: " << delta_x << "  Vx: " << Vx << std::endl;
//            std::cout << "round:" << count << std::endl;
            last_time = current_time;


}

void data_pack(const geometry_msgs::Twist cmd_vel){
    float  Vx,Vy,Ang_v;
    uint8_t Ox,Oy,Oz;
    Vx = (cmd_vel.linear.x)*57.44;
    Vy = (cmd_vel.linear.y)*57.44;
    Ang_v = (cmd_vel.angular.z)*57.44;
    if(fabs(Vx)<256 && fabs(Vy)<256 && fabs(Ang_v)<256){
        memset(s_buffer,0,sBUFFERSIZE);
        Vx < 0 ? Ox = 0x04 : Ox = 0x00;
        Vy < 0 ? Oy = 0x00 : Oy = 0x02;
        Ang_v < 0 ? Oz = 0x01 : Oz = 0x00;
        s_buffer[0] = 0xff;
        s_buffer[1] = 0xfe;
        s_buffer[2] = 1;
        s_buffer[3] = 0;
        s_buffer[4] = fabs(Vx);
        s_buffer[5] = 0;
        s_buffer[6] = fabs(Vy);
        s_buffer[7] = 0;
        s_buffer[8] = fabs(Ang_v);
        s_buffer[9] = Ox+Oy+Oz;
        ser.write(s_buffer,sBUFFERSIZE);
    }
}

void cmd_vel_CB(const geometry_msgs::Twist cmd_vel){
//    ROS_INFO("linear velocity: x-[%f],y-[%f]",cmd_vel.linear.x,cmd_vel.linear.y);
//    ROS_INFO("angular velocity: yaw-[%f]",cmd_vel.angular.z);
//    std::cout << "Twist Received" << std::endl;
    data_pack(cmd_vel);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "uart_stm32");
    ros::NodeHandle nh;
    tf::TransformBroadcaster odom_broadcaster;
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom",1000);
    vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000,cmd_vel_CB);
    try
        {
            ser.setPort("/dev/ttyUSB0");
            ser.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Unable to open port ");
            return -1;
        }

        if(ser.isOpen()){
            ROS_INFO_STREAM("Serial Port initialized");
        }else{
            return -1;
        }

    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        if(ser.available()){
           std::string str = ser.read(ser.available());
           for(int i = 0; i < str.length(); i++)
           {
             buf.push_back(str[i]);
             printf("%x, ", *buf.rbegin());
             if(*buf.rbegin() == 0xaa && *(buf.rbegin()+1) == 0xaa){
                if(buf.size() == 11){
                    if(*(buf.rbegin() + 10) == 0xbb){
                       for(int l=0; l<buf.size(); l++){
                          r_buf.push_back(buf[l]);
                       }
                       data_analysis(r_buf);
                       r_buf.clear();

                    }
                }
               std::cout << std::endl;
               buf.clear();
             }
           }
        }
    }
}
