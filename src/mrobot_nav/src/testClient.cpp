#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>  
#include <netinet/in.h>  
#include "ros/ros.h"

char *host_name = "127.0.0.1";
int port = 8000;

int main(int argc , char * argv[])  
{  
    char buf[8192];  
    //char message[256];  
    int socket_descriptor;  
    struct sockaddr_in pin;  
    char * str ="A default test string";  


    bzero(&pin,sizeof(pin));  
    pin.sin_family = AF_INET;  
    inet_pton(AF_INET,host_name,&pin.sin_addr);  
    pin.sin_port = htons(port);  
    if((socket_descriptor =  socket(AF_INET,SOCK_STREAM,0)) == -1)  
    {  
        perror("error opening socket \n");  
        exit(1);  
    }  
    if(connect(socket_descriptor,(struct sockaddr * )&pin,sizeof(pin)) == -1)  
    {  
        perror("error connecting to socket \n");  
        exit(1);  
    }  
    printf("sending message %s to server ..\n",str);  
    //while(1)
    //{
        //scanf("%s", str);
        if( write(socket_descriptor,str,strlen(str)+1) == -1 )  
        {  
            perror("error in send \n");  
            exit(1);  
        }  
    
        printf("..sent message ...wait for message..\n");  
        if( read(socket_descriptor,buf,8192) == -1 )  
        {  
            perror("error in receiving response from server \n");  
            exit(1);  
        }  
    
        printf("\nResponse from server:\n\n%s\n",buf);  

    //}
    close(socket_descriptor);  
    return 1;  

}