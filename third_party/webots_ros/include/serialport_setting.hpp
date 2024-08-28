#include<stdio.h>      
#include<stdlib.h>    
#include<unistd.h>     
#include<sys/types.h>  
#include<sys/stat.h>   
#include<fcntl.h>     
#include<termios.h>    
#include<errno.h> 
    
int speed_arr[] = { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300,    
    B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300, };    
    
int name_arr[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300,    
    115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300, };    
    
/*-----------------------------------------------------------------------------  
  function:      set_speed  
  parameters:    int fd ,int speed  
  return:        void  
  description:   setting Baud rate of fd 
 *-----------------------------------------------------------------------------*/    
void set_speed(int fd ,int speed)    
{    
    struct termios opt;    
    unsigned int i;    
    int status;    
    
    tcgetattr(fd,&opt);    
    for(i = 0;i < sizeof(speed_arr)/sizeof(int);i++)    
    {    
        if(speed == name_arr[i])                            
        {    
            tcflush(fd,TCIOFLUSH);                          
            cfsetispeed(&opt,speed_arr[i]);         
            cfsetospeed(&opt,speed_arr[i]);          
    
    		opt.c_cc[VTIME] =0;
    		opt.c_cc[VMIN] = 57; 

            status = tcsetattr(fd,TCSANOW,&opt);        
            if(status != 0)    
                perror("tcsetattr fd:");                     
            return ;    
        }    
        tcflush(fd,TCIOFLUSH);                              
    }    
}  
  
/*-----------------------------------------------------------------------------  
  function:      set_parity  
  parameters:    int fd  
  return:        int  
  description:   setting the parity of fd  
 *-----------------------------------------------------------------------------*/    
int set_parity(int fd)    
{    
    struct termios opt;    
    
    if(tcgetattr(fd,&opt) != 0)                      
    {    
        perror("Get opt in parity error:");    
        return -1;    
    }    
      
    opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP    
                | INLCR | IGNCR | ICRNL | IXON);    
    opt.c_oflag &= ~OPOST;    
    opt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);    
    opt.c_cflag &= ~(CSIZE | PARENB);    
    opt.c_cflag |= CS8;    
    
    tcflush(fd,TCIFLUSH);                            
    
    if(tcsetattr(fd,TCSANOW,&opt) != 0)    
    {    
        perror("set attr parity error:");    
        return -1;    
    }    
    
    return 0;    
} 
   
/*-----------------------------------------------------------------------------  
  function:      serial_init  
  parameters:    char *dev_path,int speed,int is_block  
  return:        a file descriptor 
  description:   config serial port parameters
 *-----------------------------------------------------------------------------*/    
int serial_init(char *dev_path,int speed,int is_block)    
{    
    int fd;    
    int flag;    
    
    flag = 0;    
    flag |= O_RDWR;                     
    flag |= O_NOCTTY; 
    flag |= O_NDELAY; 
    if(is_block == 0)    
        flag |=O_NONBLOCK;                   
    
    fd = open(dev_path,flag);               
    if(fd < 0)    
    {    
        perror("Open device file err:");    
        close(fd);        
        return -1;    
    }    
    
    set_speed(fd,speed);                    
       
    if(set_parity(fd) != 0)    
    {    
        perror("set parity error:");    
        close(fd);                           
        return -1;    
    }    
    
    return fd;    
}  

