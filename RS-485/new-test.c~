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
        unsigned char buf[100] ="abcdefghijklmnopqrstuvwxyz";
        unsigned char buf1[200];
        int i=0;
	/*if (sscanf (argv[1], "%i", &value)!=1) { printf ("error - not an integer"); }*/
        printf("test for i2c_485 !\n");
        printf (" I am coming !\n");
      //  value = 9600;
      //  ioctl(fd,GPIOIOC_SETBAUDERATE,&value);

        while(1)
        {
	        fd = open("/dev/i2c_uart",O_WRONLY);
		        if(fd < 0)
	                {
                        perror("open device i2c_gpio failed \n");
                        return -1;
        	        }

    	        write(fd,buf,22);
              //  printf ("write completed here in test file \n");
		sleep(1);
		close(fd);
                memset(buf1,'\0',200);

		fd = open("/dev/i2c_uart",O_RDONLY);
		        if(fd < 0)
	                {
                        perror("open device i2c_gpio failed \n");
                        return -1;
        	        }
		sleep(1);
		if(read(fd,buf1,100) > 0)
                {
                printf ("%s",buf1);
                }
                else {
                printf("read no chars\n");
                }
                close(fd);
        }
return 0;

}
