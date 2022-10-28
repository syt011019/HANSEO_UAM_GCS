#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

#include <ros/ros.h>
#include <darknet_msg/CNNDetectionList.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Point.h>

struct single_detection
{
    union{ uint8_t fdata; char cdata[1];} label_idx;
	union{ uint16_t fdata; char cdata[2];} detection_x;
	union{ uint16_t fdata; char cdata[2];} detection_y;
    union{ uint8_t fdata; char cdata[1];} xc1;
    union{ uint8_t fdata; char cdata[1];} xc2;
    union{ uint8_t fdata; char cdata[1];} yc1;
    union{ uint8_t fdata; char cdata[1];} yc2;
};

struct multiple_detection
{
    char numOfDetection;
	union{ uint8_t fdata; char cdata[1];} cnt;
	struct single_detection sd[20];
};

#define PORT     6284
#define MAXLINE 128
  
// Driver code 
int main(int argc, char *argv[]) { 
    int sockfd; 
    int i;
    char buffer[MAXLINE];
    struct sockaddr_in servaddr, cliaddr; 

    struct multiple_detection md;
    struct single_detection sd;
      
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    memset(&servaddr, 0, sizeof(servaddr)); 

    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT); 
      
    // Bind the socket with the server address 
    if ( bind(sockfd, (struct sockaddr *)&servaddr,  
            sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    int n, len;
    int j, k = 0;
    len = sizeof(cliaddr);  //len is value/resuslt 

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    //ros::Rate rate(50);
    darknet_msg::CNNDetectionList cnnDetectionList;
    ros::Publisher darknet_pub = nh.advertise<darknet_msg::CNNDetectionList>("image/CNNDetection", 1);
    //ros::Publisher target_pos = nh.advertise<geometry_msgs::Point>("target_pos", 1);
    

    geometry_msgs::Point target_pos_msg;

    while(ros::ok())
    {
        n = recvfrom(sockfd, (char *)buffer, MAXLINE,  
                    0, ( struct sockaddr *) &cliaddr, 
                    (socklen_t*) &len);

        buffer[n]='\0';
        
        md.numOfDetection = buffer[0];
        md.cnt.cdata[0] = buffer[1];
/*
        for(int j = 0; j < md.cnt.fdata; ++j){

            printf("Buffer[%d] :: %d\n",5*j + 2, buffer[5*j + 2]);
            printf("Buffer[%d] :: %d\n",5*j + 3, buffer[5*j + 3]);
            printf("Buffer[%d] :: %d\n",5*j + 4, buffer[5*j + 4]);
            printf("Buffer[%d] :: %d\n",5*j + 5, buffer[5*j + 5]);
            printf("Buffer[%d] :: %d\n",5*j + 6, buffer[5*j + 6]);
        }
        printf("---------------------------------------------\n");
        */


        for(int j = 0; j < md.cnt.fdata; ++j)
        {
            md.sd[j].label_idx.cdata[0] = buffer[5*j + 2];
            md.sd[j].xc1.cdata[0] = (buffer[5*j + 3]), md.sd[j].xc2.cdata[0] =  buffer[5*j + 4];
            md.sd[j].yc1.cdata[0] = (buffer[5*j + 5]), md.sd[j].yc2.cdata[0] =  buffer[5*j + 6];

            md.sd[j].detection_x.fdata = md.sd[j].xc1.fdata*100 + md.sd[j].xc2.fdata;
            md.sd[j].detection_y.fdata = md.sd[j].yc1.fdata*100 + md.sd[j].yc2.fdata;

            if(md.sd[j].xc1.cdata[0] > 98){
                md.sd[j].detection_x.fdata = md.sd[j].xc2.fdata;
            }
            if(md.sd[j].yc1.cdata[0] > 98){
                md.sd[j].detection_y.fdata = md.sd[j].yc2.fdata;
            }

            printf("Buffer  x1 = %d, x2 = %d\n", buffer[5*j + 3], buffer[5*j + 4]);
            printf("Buffer  y1 = %d, y2 = %d\n", buffer[5*j + 5], buffer[5*j + 6]);

            printf("Object  x1 = %d, x2 = %d\n", md.sd[j].xc1.fdata, md.sd[j].xc2.fdata);
            printf("Object  y1 = %d, y2 = %d\n", md.sd[j].yc1.fdata, md.sd[j].yc2.fdata);
        }
        printf("---------------------------------------------\n");

        cnnDetectionList.obj.resize(md.numOfDetection);
        
        for(int j = 0; j < md.cnt.fdata; ++j){
            cnnDetectionList.obj[j].roi.x_offset = (int) md.sd[j].detection_x.fdata;
            cnnDetectionList.obj[j].roi.y_offset = (int) md.sd[j].detection_y.fdata;
            cnnDetectionList.obj[j].roi.width = md.sd[j].label_idx.fdata;
            cnnDetectionList.obj[j].roi.height = md.numOfDetection;
            //printf("Yc : %d, %d\n", buffer[5*j + 5], buffer[5*j + 6]);
            //printf("Xc : %d, %d\n", buffer[5*j + 3], buffer[5*j + 4]);
            //printf("Yc : %d, %d\n", buffer[5*j + 5], buffer[5*j + 6]);
            
            printf("Object  x = %d, y = %d, class = %d\n", md.sd[j].detection_x.fdata, md.sd[j].detection_y.fdata, md.sd[j].label_idx.fdata);
        }

        printf("===============================\n");

        darknet_pub.publish(cnnDetectionList);

        //rate.sleep();
        
    }

    //printf("Hello message sent.\n");  
     
      
    return 0; 
}
