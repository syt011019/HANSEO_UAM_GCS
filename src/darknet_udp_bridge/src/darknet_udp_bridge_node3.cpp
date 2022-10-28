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

struct single_detection
{
	uint8_t label_idx;
	union{ float fdata; char cdata[4];} detection_x;
	union{ float fdata; char cdata[4];} detection_y;
};

struct multiple_detection
{
	char numOfDetection;
	struct single_detection sd[20];
};

#define PORT     6284 
#define MAXLINE 1024 
  
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
    memset(&cliaddr, 0, sizeof(cliaddr)); 
      
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT); 
      
    // Bind the socket with the server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
            sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    int n; 
    socklen_t len;
    len = sizeof(cliaddr);  //len is value/resuslt 
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    darknet_msg::CNNDetectionList cnnDetectionList;
    ros::Publisher darknet_pub = nh.advertise<darknet_msg::CNNDetectionList>("image/CNNDetection", 1);

    while(ros::ok())
    {
        n = recvfrom(sockfd, (char *)buffer, MAXLINE,  
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr, 
                    &len); 
        buffer[n] = '\0'; 

        md.numOfDetection = buffer[0];
 //       printf("md.numOfDetection   :   %d\n", md.numOfDetection);
        for(i = 0; i < md.numOfDetection; i++)
        {
            md.sd[i].label_idx = buffer[1+(9*i) + 0];

            md.sd[i].detection_x.cdata[0] = buffer[1+(9*i) + 1];
            md.sd[i].detection_x.cdata[1] = buffer[1+(9*i) + 2];
            md.sd[i].detection_x.cdata[2] = buffer[1+(9*i) + 3];
            md.sd[i].detection_x.cdata[3] = buffer[1+(9*i) + 4];
            
            md.sd[i].detection_y.cdata[0] = buffer[1+(9*i) + 5];
            md.sd[i].detection_y.cdata[1] = buffer[1+(9*i) + 6];
            md.sd[i].detection_y.cdata[2] = buffer[1+(9*i) + 7];
            md.sd[i].detection_y.cdata[3] = buffer[1+(9*i) + 8];
        }


//        printf("-------------------------------------------------------------\n");
        cnnDetectionList.obj.resize(md.numOfDetection);
        for(i = 0; i < md.numOfDetection; i++)
        {
        
            cnnDetectionList.obj[i].roi.x_offset = (int)(md.sd[i].detection_x.fdata*640);
            cnnDetectionList.obj[i].roi.y_offset = (int)(md.sd[i].detection_y.fdata*480);
            cnnDetectionList.obj[i].roi.height = 0;
            cnnDetectionList.obj[i].roi.width = 0;

            switch (md.sd[i].label_idx)
            {
                case 1:
                    cnnDetectionList.obj[i].label.data = "LandingPad";
                    std::cout<<"LandingPad "<<std::endl;
                    break;
                default:
                    cnnDetectionList.obj[i].label.data = "default";
                    std::cout<<"Default Name "<<std::endl;
                    break;
            }

            
//            printf("class idx : %d,     x : %f,   y : %f \n", md.sd[i].label_idx, md.sd[i].detection_x.fdata, md.sd[i].detection_y.fdata);
        }

        darknet_pub.publish(cnnDetectionList);
    }
//    printf("Client : %s\n", buffer); 

		

    //printf("Hello message sent.\n");  
      
    return 0; 
}
