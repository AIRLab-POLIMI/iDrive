#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <fcntl.h>
#include <netdb.h>
#include <idrive_data_logger/Bio_sensor.h>
#include <unistd.h>
#include <signal.h>

//constant string part to send to the server
const char SERVER_STATUS[16] = "server_status\r\n";
const char R_SERVER_STATUS[20] = "R system_status OK\n";
const char DEVICE_LIST[14] = "device_list\r\n";
const char DEVICE_CONNECT[16] = "device_connect ";
const char DEVICE_DISCONNECT[20] = "device_disconnect\r\n";
const char R_DEVICE_CONNECT[21] = "R device_connect OK\n";
const char GSR_SUBSCRIBE[26] = "device_subscribe gsr ON\r\n";
const char ACC_SUBSCRIBE[26] = "device_subscribe acc ON\r\n";
const char BVP_SUBSCRIBE[26] = "device_subscribe bvp ON\r\n";
const char IBI_SUBSCRIBE[26] = "device_subscribe ibi ON\r\n";
const char TMP_SUBSCRIBE[26] = "device_subscribe tmp ON\r\n";
const char GSR_UNSUBSCRIBE[27] = "device_subscribe gsr OFF\r\n";
const char ACC_UNSUBSCRIBE[27] = "device_subscribe acc OFF\r\n";
const char BVP_UNSUBSCRIBE[27] = "device_subscribe bvp OFF\r\n";
const char IBI_UNSUBSCRIBE[27] = "device_subscribe ibi OFF\r\n";
const char TMP_UNSUBSCRIBE[27] = "device_subscribe tmp OFF\r\n";
const char R_GSR_SUBSCRIBE[27] = "R device_subscribe gsr OK\n";
const char R_ACC_SUBSCRIBE[27] = "R device_subscribe acc OK\n";
const char R_BVP_SUBSCRIBE[27] = "R device_subscribe bvp OK\n";
const char R_IBI_SUBSCRIBE[27] = "R device_subscribe ibi OK\n";
const char R_TMP_SUBSCRIBE[27] = "R device_subscribe tmp OK\n";


bool first = true;
int socketfd;
bool client_connected = false;

//handle the reading error
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

//read the server response until the EOL char
void read(int socketfd, char result[256]){
    ssize_t bytes_recieved;
    char recv_buf[1000];
    char* rc = (char*) recv_buf;
    int i = 0;
    *rc ='x';
    //ROS_INFO("reading");
    while (*rc != '\n'){
	   bytes_recieved = recv(socketfd, rc, 1, 0);
	   if (HandleError(bytes_recieved)){
               result[i] = *rc; 
               i++;
       }
    }
    if (i > 0){
        result[i] = '\0';
    }else if(i == 0){
        result[i]= '\0';
    }
}

void mySigintHandler(int sig)
{
   ROS_INFO("client stop");
   if (client_connected){
       send(socketfd ,  DEVICE_DISCONNECT , strlen( DEVICE_DISCONNECT) , 0);
   }
   close(socketfd); 
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
  exit(0);
}

//extract the first Empatica device ID from the response of the server
void extractDeviceId(char response[256], char deviceID[20]){
    char * pch;
    if (strcmp(response, "R device_list 0 \n") == 0){
        deviceID[0] = '\0';
        return;
    }
    pch = strtok (response,"|");
    if (pch != NULL)
    {
        pch = strtok (NULL, " "); 
        if (pch != NULL){
           for (int i = 0; i<strlen(pch); i++){
               deviceID[i] = pch[i];
           } 
        }
    }
}

void replace_comma(char* pch){
     for (int i = 0; i<strlen(pch); i++){
         if (pch[i] == ','){
             pch[i] = '.';
             break;
         }
     } 
 }

//extract sensor informations and datas from the response of the server 
void extract_sensor_data(char response[256], char sensor_type[5], double* timestamp, float data[3]){
    char * pch;
    pch = strtok (response," ");
    if (pch != NULL){
        int i = 0;
        for (i=0;i<strlen(pch);i++){
            sensor_type[i] = pch[i];
        }
        sensor_type[i] = '\0';
    }
    pch = strtok (NULL, " ");
    if (pch != NULL){
        replace_comma(pch); 
        *timestamp = strtod(pch, NULL);    
    }  
    pch = strtok(NULL, " ");
    int i = 0;
    while(pch != NULL){
        replace_comma(pch);
        data[i] = strtof(pch, NULL);
        i++;
        pch = strtok (NULL, " ");
       
    }
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
    status = getaddrinfo("192.168.1.10", "28015", &host_info, &host_info_list);
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
    ros::init(argc, argv, "empatica_socket_client");
    ros::NodeHandle n;
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);
    ros::Publisher publisher = n.advertise<idrive_data_logger::Bio_sensor>("empatica_sensor", 1000);
    idrive_data_logger::Bio_sensor msg;
    ros::Rate loop_rate(100);
   
   char response[256];
   
   socketfd = socket_setup();
   
   //check the status of the server  
   while(send(socketfd , SERVER_STATUS , strlen(SERVER_STATUS) , 0) < 0 && ros::ok()){
       ROS_INFO("Send status failed");
   }
   read(socketfd, response);
   while(strcmp(response, R_SERVER_STATUS) != 0 && ros::ok()){
       ROS_INFO("Empatica server error : %s" , response);
       while( send(socketfd , SERVER_STATUS , strlen(SERVER_STATUS) , 0) < 0 && ros::ok()){
            ROS_INFO("Send status failed");
       }
       read(socketfd, response);
   }
   ROS_INFO("Empatica server status ok");
   
   //check if an Empatica device is connect and take the ID of the connected device
   while( send(socketfd , DEVICE_LIST , strlen(DEVICE_LIST) , 0) < 0 && ros::ok()){
       ROS_INFO("Send device list failed");
   }
   read(socketfd, response);
   char deviceID[20];
   extractDeviceId(response, deviceID);
   while(strlen(deviceID) == 0 && ros::ok()){
       while( send(socketfd , DEVICE_LIST , strlen(DEVICE_LIST) , 0) < 0 && ros::ok()){
            ROS_INFO("Send device list failed");
       }    
       read(socketfd, response);
       extractDeviceId(response, deviceID);
   }   
   ROS_INFO("device_ID : %s" , deviceID);
   
   //Connect to the Empatica device with the device ID
   char request[30];
   strcat(request,DEVICE_CONNECT);
   strcat(request,deviceID);
   strcat(request,"\r\n");
   while( send(socketfd , request , strlen(request) , 0) < 0 && ros::ok()){
       ROS_INFO("Send device connect failed");
   }
   read(socketfd, response);
   while(strcmp(response, R_DEVICE_CONNECT) != 0 && ros::ok()){
       while( send(socketfd , request , strlen(request) , 0) < 0 && ros::ok()){
            ROS_INFO("Send device connect failed");
       }
       read(socketfd, response);
   }   
   ROS_INFO("device connected");   
   client_connected = true;
   
   //Subscribe to the Empatica Galvanic Skin Response stream
   while( send(socketfd , GSR_SUBSCRIBE , strlen(GSR_SUBSCRIBE) , 0) < 0 && ros::ok()){
       ROS_INFO("Send device stream failed");
   }
   read(socketfd, response);
   while(strcmp(response, R_GSR_SUBSCRIBE) != 0 && ros::ok()){
      if (response[0] == 'R'){
           while( send(socketfd , request , strlen(request) , 0) < 0 && ros::ok()){
                ROS_INFO("Send device connect failed");
           }
       }
       read(socketfd, response);
   } 
   ROS_INFO("GSR stream on");
   
   //Subscribe to the Empatica 3-axis acceleration stream
    while( send(socketfd , ACC_SUBSCRIBE , strlen(ACC_SUBSCRIBE) , 0) < 0 && ros::ok()){
       ROS_INFO("Send device stream failed");
   }
   read(socketfd, response);
   while(strcmp(response, R_ACC_SUBSCRIBE) != 0 && ros::ok()){
       if (response[0] == 'R'){
           while( send(socketfd , request , strlen(request) , 0) < 0 && ros::ok()){
                ROS_INFO("Send device connect failed");
           }
        }
       read(socketfd, response);
   }    
   ROS_INFO("ACC stream on");
    
   //Subscribe to the Empatica Interbeat Interval stream
    while( send(socketfd , IBI_SUBSCRIBE , strlen(IBI_SUBSCRIBE) , 0) < 0 && ros::ok()){
       ROS_INFO("Send device stream failed");
   }
   read(socketfd, response);
   while(strcmp(response, R_IBI_SUBSCRIBE) != 0 && ros::ok()){
       if (response[0] == 'R'){
           while( send(socketfd , request , strlen(request) , 0) < 0 && ros::ok()){
                ROS_INFO("Send device connect failed");
           }
       }
       read(socketfd, response);
   }
   ROS_INFO("IBI stream on"); 
    
   //Subscribe to the Empatica Skin Temperature stream  
    while( send(socketfd , TMP_SUBSCRIBE , strlen(TMP_SUBSCRIBE) , 0) < 0 && ros::ok()){
       ROS_INFO("Send device stream failed");
   }
   read(socketfd, response);
   while(strcmp(response, R_TMP_SUBSCRIBE) != 0 && ros::ok()){
       if (response[0] == 'R'){
           while( send(socketfd , request , strlen(request) , 0) < 0 && ros::ok()){
                ROS_INFO("Send device connect failed");
           }
       }
       read(socketfd, response);
   }     
   ROS_INFO("TMP stream on");
  
   //Subscribe to the Empatica Blood Volume Pulse stream
    while( send(socketfd , BVP_SUBSCRIBE , strlen(BVP_SUBSCRIBE) , 0) < 0 && ros::ok()){
       ROS_INFO("Send device stream failed");
   }
   read(socketfd, response);
   while(strcmp(response, R_BVP_SUBSCRIBE) != 0 && ros::ok()){
        if (response[0] == 'R'){
           while( send(socketfd , request , strlen(request) , 0) < 0 && ros::ok()){
                ROS_INFO("Send device connect failed");
           }
        }
       read(socketfd, response);
   }   
   ROS_INFO("BVP stream on");
  
   //Read the sensor data from the streams
   char sensor_type[5];
   float data[3];
   double timestamp;
   double prev_timestamp_gsr = 0;
   double prev_timestamp_acc = 0;
   double prev_timestamp_tmp = 0;
   double prev_timestamp_bvp = 0;
   double prev_timestamp_ibi = 0;
   double timestamp_diff = 0;
   double freq =-1;
   
   while (ros::ok())
   {
       read(socketfd, response);
       extract_sensor_data(response, sensor_type, &timestamp, data);
       msg.data.clear();  
       switch (sensor_type[3]){
       case 'G':
            msg.sensor_type = "Galvanic Skin Response";
            msg.MU = "microSiemens";
            msg.data.push_back(data[0]);
            //calculates the frequency
            msg.freq = 4;
            break;
       case 'A':
             //ROS_INFO("x: %f", data[0]);
             //ROS_INFO("y: %f", data[1]);
             //ROS_INFO("z: %f", data[2]);
            msg.sensor_type = "Accelleration";
            msg.MU = "uknown";
            for(int i = 0;i < 3; i++){
                msg.data.push_back(data[i]);
            }
            msg.freq = 32;
            break;      
       case 'T':
            msg.sensor_type = "Temperature";
            msg.MU = "celsius degrees";
            msg.data.push_back(data[0]);
            //calculates the frequency
            msg.freq = 4;
            break;
       case 'B':
            msg.sensor_type = "BVP";
            msg.MU = "relative";
            msg.data.push_back(data[0]);
            //calculates the frequency
            msg.freq = 64;
            break;
       case 'I':
            msg.sensor_type = "Interbeat Interval";
            msg.MU = "seconds";
            timestamp = ros::Time::now().toSec();
            msg.data.push_back(data[0]);
            //calculates the frequency
            if (prev_timestamp_ibi == 0){
                freq = -1;
            }else{
                timestamp_diff = timestamp - prev_timestamp_ibi;
                if(timestamp_diff == 0){
                    freq = -1;
                }else{
                    freq = 1/timestamp_diff; 
                }
            }
            prev_timestamp_ibi = timestamp;
            msg.freq = (int) round(freq);
            break;
       case 'H': 
            msg.sensor_type = "Heart Rate";
            msg.MU = "bpm";
            timestamp = ros::Time::now().toSec();
            msg.data.push_back(data[0]);
            //calculates the frequency
            if (prev_timestamp_ibi == 0){
                freq = -1;
            }else{
                timestamp_diff = timestamp - prev_timestamp_ibi;
                if(timestamp_diff == 0){
                    freq = -1;
                }else{
                    freq = 1/timestamp_diff; 
                }
            }
            prev_timestamp_ibi = timestamp;
            msg.freq = (int) round(freq);
            break;

            break;
       }
       msg.sample_number = 1;
       msg.header.frame_id = "/bio_sensor";
       msg.header.stamp = ros::Time(timestamp);

       publisher.publish(msg); 
       ros::spinOnce();
       loop_rate.sleep();
   }
   return 0;
}

