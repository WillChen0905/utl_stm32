#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <math.h>

#define rBUFFERSIZE 20
#define sBUFFERSIZE 10
unsigned char s_buffer[sBUFFERSIZE];
unsigned char ret=0;

serial::Serial ser;
ros::Subscriber uart_sub;


void data_pack(const geometry_msgs::Twist cmd_vel){
    float  Vx,Vy,Ang_v;
    uint8_t Ox,Oy,Oz;
    Vx = (cmd_vel.linear.x)*57.44;
    Vy = (cmd_vel.linear.y)*57.44;
    Ang_v = (cmd_vel.angular.z)*57.44;
    if(fabs(Vx)<256 && fabs(Vy)<256 && fabs(Ang_v)<256){
        memset(s_buffer,0,sBUFFERSIZE);
        Vx < 0 ? Ox = 0x04 : Ox = 0x00;
        Vy < 0 ? Oy = 0x02 : Oy = 0x00;
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
        for(int k=0; k<10; k++){
            ROS_INFO("%x",s_buffer[k]);
        }
        ser.write(s_buffer,sBUFFERSIZE);
    }
}

void cmd_vel_CB(const geometry_msgs::Twist cmd_vel){
    ROS_INFO("linear velocity: x-[%f],y-[%f]",cmd_vel.linear.x,cmd_vel.linear.y);
    ROS_INFO("angular velocity: yaw-[%f]",cmd_vel.angular.z);
    std::cout << "Twist Received" << std::endl;
    data_pack(cmd_vel);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "uart_stm32");
    ros::NodeHandle nh;
    uart_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000,cmd_vel_CB);
    try
        {
            ser.setPort("/dev/ttyUSB1");
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
            ROS_INFO_STREAM("Reading from serial port");
        }
    }
}
