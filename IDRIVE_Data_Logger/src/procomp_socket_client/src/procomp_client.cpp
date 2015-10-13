#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netdb.h>
#include <idrive_data_logger/Bio_sensor.h>
#include <fcntl.h>
#include <signal.h>

bool first = true;
 int socketfd ; // The socket descripter

bool HandleError(ssize_t control){
    if (control == 0) {
        ROS_INFO("host shut down.");
        exit(0);
        return false;
    }else if (control == -1) {
        ROS_INFO("recieve error!");
        return false;
    }else {
        if (first) {
            ROS_INFO("Recieving data");
            first = false;
        }
    }
    return true;
}

void mySigintHandler(int sig)
{
   ROS_INFO("client stop");
   close(socketfd);
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
  exit(0);
}

//create the socket and connect to the server. Return the reference of the socket
int socket_setup(){
    int status;
    long arg; 
    fd_set myset; 
    struct timeval tv; 
    int valopt; 
    socklen_t lon; 
    
    struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
    struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.
  
    memset(&host_info, 0, sizeof host_info);

    ROS_INFO("Setting up the structs...");
    host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
    host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP
    status = getaddrinfo("192.168.1.10", "27015", &host_info, &host_info_list);
    // getaddrinfo returns 0 on succes, or some other value when an error occured.
    // (translated into human readable text by the gai_gai_strerror function).
    if (status != 0){
        ROS_INFO("getaddrinfo error %s" , gai_strerror(status)) ;
    }

    ROS_INFO("Creating a socket...");
    int socketfd; // The socket descripter
    socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype,
                      host_info_list->ai_protocol);
    if (socketfd == -1){
        ROS_INFO("socket error");
        return -1;
    }

    ROS_INFO("Connecting...");
    status = -1;
    
       
    // Set non-blocking connect function
    arg = fcntl(socketfd, F_GETFL, NULL); 
    arg |= O_NONBLOCK; 
    fcntl(socketfd, F_SETFL, arg); 
    status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    
    while(ros::ok()){
    //Check the connection to the server 
      if (status < 0) { 
         if (errno == EINPROGRESS) { 
            tv.tv_sec = 10; 
            tv.tv_usec = 0; 
            FD_ZERO(&myset); 
            FD_SET(socketfd, &myset); 
            if (select(socketfd+1, NULL, &myset, NULL, &tv) > 0) { 
               lon = sizeof(int); 
               getsockopt(socketfd, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &lon); 
               if (valopt) { 
                  //if there is an error in the connection retry to connect to the server
                  ROS_INFO("Error in connection() %d - %s", valopt, strerror(valopt));
                  close(socketfd);
                  socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype,host_info_list->ai_protocol);
                  if (socketfd == -1){
                      ROS_INFO("socket error");
                      return -1;
                  }
                  // Set non-blocking 
                  arg = fcntl(socketfd, F_GETFL, NULL); 
                  arg |= O_NONBLOCK; 
                  fcntl(socketfd, F_SETFL, arg);
                  status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
               }else{
                  break;
               }
            } 
            else { 
               ROS_INFO("Timeout or error() %d - %s. Retring to connect", valopt, strerror(valopt)); 
            } 
         } 
         else { 
            ROS_INFO("Error connecting %d - %s", errno, strerror(errno)); 
            if (errno = 4) exit(0);
         } 
      } 
  }
     // Set to blocking mode again... 
     arg = fcntl(socketfd, F_GETFL, NULL); 
     arg &= (~O_NONBLOCK); 
     fcntl(socketfd, F_SETFL, arg); 
    
    return socketfd;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "procomp_socket_client");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    ros::Publisher publisher = n.advertise<idrive_data_logger::Bio_sensor>("procomp_sensor", 1000);
    idrive_data_logger::Bio_sensor msg;
    ros::Rate loop_rate(100);

    socketfd = socket_setup();

    ssize_t bytes_recieved;
    char recv_buf[10];
    int* sensor_id = (int*) recv_buf ;
    int* lenght = (int*) recv_buf ;
    int* sec = (int*) recv_buf;
    int sec_temp;
    int* nsec = (int*) recv_buf;
    float* data = (float*) recv_buf;
    float* freq = (float*) recv_buf;
    ROS_INFO("Waiting to recieve data...");

 //------------------------------------------------------------------------------------------------------------

    while (ros::ok())
    {
        bytes_recieved = recv(socketfd, sec, 4, 0);
        // If no data arrives, the program will just wait here until some data arrives.
        if (HandleError(bytes_recieved)){
            //ROS_INFO("sec %i", *sec);
        }
        sec_temp = *sec;
        bytes_recieved = recv(socketfd, nsec, 4, 0);
        // If no data arrives, the program will just wait here until some data arrives.
        if (HandleError(bytes_recieved)){
            //ROS_INFO("nsec %i", *nsec);
        }

        msg.header.stamp = ros::Time(sec_temp,*nsec);
        bytes_recieved = recv(socketfd, sensor_id, 4, 0);
        // If no data arrives, the program will just wait here until some data arrives.
        if (HandleError(bytes_recieved)){
            //ROS_INFO("sensor id %i", *sensor_id);
            switch (*sensor_id){
              case 17:
                 msg.sensor_type = "Temperature";
                 msg.MU = "degree Celsius";
                 break;
              case 5:
                 msg.sensor_type = "Skin conductance";
                 msg.MU = "microSiemens";
                 break;
              case 10:
                 msg.sensor_type = "Respiration";
                 msg.MU = "relative";
                 break;
              case 4:
                 msg.sensor_type = "BVP";
                 msg.MU = "relative";
                 break;
              case 6:
                 msg.sensor_type = "ECG";
                 msg.MU = "milliVolt";
                 break;
              case 9:
                msg.sensor_type = "SEMG";
                msg.MU = "milliVolts, converted according to an RMStype function";
                break;
              case 15:
                msg.sensor_type = "SEMG";
                msg.MU = "milliVolts, converted according to an RMStype function";
                break;

            }
        }

        bytes_recieved = recv(socketfd, freq, 4, 0);
        // If no data arrives, the program will just wait here until some data arrives.
        if (HandleError(bytes_recieved)){
            msg.freq = *freq;
            //ROS_INFO("freq %f", *freq);
        }

        bytes_recieved = recv(socketfd, lenght, 4, 0);
        // If no data arrives, the program will just wait here until some data arrives.
        if (HandleError(bytes_recieved)){
            msg.sample_number = *lenght;
            //ROS_INFO("lenght %i", *lenght);
        }

        int count = *lenght;
        msg.data.clear();
        for(int i = 0;i < count; i++){
            bytes_recieved = recv(socketfd, data, 4, 0);
            // If no data arrives, the program will just wait here until some data arrives.
            if (HandleError(bytes_recieved)){
                msg.data.push_back(*data);
                //ROS_INFO("data %f", *data);
            }
        }

        msg.header.frame_id = "/bio_sensor";
        publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

