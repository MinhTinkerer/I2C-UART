#include <sys/ioctl.h>
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>


#define        GPIODRV_IOCTL_BASE        'G'
#define GPIOIOC_SETBAUDERATE        _IOR(GPIODRV_IOCTL_BASE, 12, int)

int main(int argc, char *argv[])
//int main()
{
        int fd;
        int value;
        unsigned char buf[100] ="abcdefghijklmnopqrstuvwxyz123456789987654321";
        unsigned char buf1[200];
        int i=0;
	/*if (sscanf (argv[1], "%i", &value)!=1) { printf ("error - not an integer"); }*/
        printf("test for i2c_485 !\n");
        fd = open("/dev/i2c_uart",O_RDWR);
        if(fd < 0)
                {
                        perror("open device i2c_gpio failed \n");
                        return -1;

                }
        printf (" I am coming !\n");
      //  value = 9600;
      //  ioctl(fd,GPIOIOC_SETBAUDERATE,&value);

        while(1)
        {
    	        write(fd,buf,40);
                printf ("\n");
		sleep(1);
                memset(buf1,'\0',200);
		if(read(fd,buf1,100) > 0)
                {
                printf ("%s",buf1);
                }
                else {
                printf("read no chars\n");
                }
                
        }
return 0;

}
