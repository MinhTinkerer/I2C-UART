/*
 * I2C_2_uart.
 * 
 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 **********************************************************************************************
*    Module Name 	:	I2C to UART converter
*    Description    	:  	Its a driver for I2C to UART, xr20m1172_110_07192012 based driver 
*    Function List  	:  	
*    Limitations    	:  	Rectifying
*    Workfile       	:	own.c
*    Soft Version   	:	Linux(2.6.32-28 Version)
*    Teted On Host 	:	Ubuntu 12.04 LTE(VMware)  
*    Date           	:   	28/10/2013
*    Author          	:   	K.Sreenivas
*
*   (C) Stesalit Ltd All rights reserved.
*-----------------------------------------------------------------------------------------------
*   Revision History
*   Date      		Who            Description
*   14-06-2013 		XYZ            First Release for Review
***********************************************************************************************/


#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/serial_reg.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/moduleparam.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>
#include "own.h"
#include <linux/delay.h>

#define DEBUG_PRINTK 		printk
#define PRINTK_I2C_GPIO		printk
#define I2C_GPIO_DRIVER_NAME        "i2c_uart"
#define I2C_GPIO_MAJOR                    230

#define I2C_M_WR     0


#define R_TRG        16
#define T_TRG        8


struct workqueue_struct  *xr20m1172_wq ;

struct work_struct   xr20m1172_work;

static struct file_operations i2c_gpio_fops;

static int i2c_gpio_major = I2C_GPIO_MAJOR;

#define XR20M1172_CLOCK                       14745600
#define PRESCALER                                         1
#define I2C_BAUD_RATE                        115200
#define SAMPLE_RATE                                16

#define I2C_DF_NOTIFY 0x01
//#define IRQ_GPIO_0                           6

#define        GPIODRV_IOCTL_BASE        'G'
#define GPIOIOC_SETBAUDERATE        _IOR(GPIODRV_IOCTL_BASE, 12, int)

#define BUF_HEAD_TX        (i2c_gpio_port->tx_buf[i2c_gpio_port->tx_head])
#define BUF_TAIL_TX         (i2c_gpio_port->tx_buf[i2c_gpio_port->tx_tail])


#define INCBUF(x,mod)   ((++(x)) & ((mod)-1))

#define BUF_HEAD_RX        (i2c_gpio_port->rx_buf[i2c_gpio_port->rx_head])
#define BUF_TAIL_RX         (i2c_gpio_port->rx_buf[i2c_gpio_port->rx_tail])



spinlock_t xr20m1172_lock;
spinlock_t global_lock;


//unsigned char read_or_not = 0;

static atomic_t recv_timeout =ATOMIC_INIT(0);

static DECLARE_MUTEX(i2c_gpio_mutex_read);
static DECLARE_MUTEX(i2c_gpio_mutex_write);
struct tasklet_struct i2c_uart_tasklet;

static struct i2c_driver i2c_gpio_driver;


static const int reg_info[27][3] = {
        {0x0+2, 20, 1},                //RHR
        {0x0+2, 20, 2},                //THR
        {0x0+2, 60, 3},                //DLL
        {0x8+2, 60, 3},                //DLM
        {0x10+2, 50, 3},                //DLD
        {0x8+2, 20, 3},                //IER:bit[4-7] needs efr[4] ==1,but we dont' access them now
        {0x10+2, 20, 1},                //ISR:bit[4/5] needs efr[4] ==1,but we dont' access them now
        {0x10+2, 20, 2},                //FCR :bit[4/5] needs efr[4] ==1,but we dont' access them now
        {0x18+2, 10, 3},                //LCR
        {0x20+2, 40, 3},                //MCR :bit[2/5/6] needs efr[4] ==1,but we dont' access them now
        {0x28+2, 40, 1},                //LSR
        {0x30+2, 70, 1},                //MSR
        {0x38+2, 70, 3},                //SPR
        {0x30+2, 80, 3},                //TCR
        {0x38+2, 80, 3},                //TLR
        {0x40+2, 20, 1},                //TXLVL
        {0x48+2, 20, 1},                //RXLVL
        {0x50+2, 20, 3},                //IODir
        {0x58+2, 20, 3},                //IOState
        {0x60+2, 20, 3},                //IOIntEna
        {0x70+2, 20, 3},                //IOControl
        {0x78+2, 20, 3},                //EFCR
        {0x10+2, 30, 3},                //EFR
        {0x20+2, 30, 3},                //Xon1
        {0x28+2, 30, 3},                //Xon2
        {0x30+2, 30, 3},                //Xoff1
        {0x38+2, 30, 3},                //Xoff2
};

static struct class *my_class;
struct xr20m1172_port *i2c_gpio_port;
//struct timer_list read_timer;
//struct timer_list read_timer1;

int irq_num=0;
unsigned int i2c_set_baudrate(unsigned  int baudrate);


static int xor_write(struct i2c_client *client, u8 reg, u8 val)
{	
	int ret;
	 down(&i2c_gpio_mutex_write);
        ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0)
               printk(&client->dev, "write Error\n");
	up(&i2c_gpio_mutex_write);
	return ret;
}

static int xor_read(struct i2c_client *client, u8 reg)
{
        int ret;
	down(&i2c_gpio_mutex_read);
	ret = i2c_smbus_read_byte_data(client, reg);
        if (ret < 0)
               printk(&client->dev, "Read Error\n");
	up(&i2c_gpio_mutex_read);	
	return ret;
}



void isr_thread(unsigned long data)
{
        unsigned char int_val0;
        unsigned char  i =0;
//        DECLARE_WAITQUEUE(wait, current);
//        add_wait_queue(&i2c_gpio_port->isr_wait, &wait);
//        daemonize("i2c-thread");
	 PRINTK_I2C_GPIO("i2c_gpio thread called scucessfully sreenivas \n");
      
//        while(1)
//                {
 //               retry:
                int_val0 = xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_ISR][0]);
		switch(int_val0)
                {
                        case 0xc4 :

                          while(xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_LSR][0]) & 0x01)
                                                {
                                        
                                BUF_HEAD_RX = xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_RHR][0]);
                                                i2c_gpio_port->rx_head = INCBUF(i2c_gpio_port->rx_head,MAX_BUF);
                                		 wake_up_interruptible(&i2c_gpio_port->rx_wait);
                                                }
				
                   	         	/*	if(read_or_not == 1)
                                                {
						     	read_timer.expires  = jiffies + HZ/100;
							
                                                        add_timer(&read_timer);
                                                }*/
	
                                        break;
                                        
                        case 0xcc :
                                        while(xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_LSR][0]) & 0x01)
                                                {
                                        
                                                BUF_HEAD_RX = xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_RHR][0]);
                                                i2c_gpio_port->rx_head = INCBUF(i2c_gpio_port->rx_head,MAX_BUF);
                                        
                                                }
                                                        atomic_set(&recv_timeout, 1);
                                                     //   wake_up_interruptible(&i2c_gpio_port->rx_wait);
                                        break;
        
                        case 0xc1:   
                                        i = xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_LSR][0]);
                                        break;
                        case 0xc2:
                                        i = xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_LSR][0]);
                                        wake_up_interruptible(&i2c_gpio_port->tx_wait);
                                        break;

                        default :  
					printk("The default is executed in switch \n");
                                        
                                break;
                                                                
                }
                	/*	if(read_or_not == 1)
                                                {
							 read_timer1.expires  = jiffies + HZ/100;
							
                                                        add_timer(&read_timer1);
                                                }*/

 //               interruptible_sleep_on(&i2c_gpio_port->isr_wait); 
//		 goto retry;
                        
//                }
//        remove_wait_queue(&i2c_gpio_port->isr_wait, &wait);
     //   return 0;
}




static int i2c_gpio_open(struct inode *inode, struct file *filp)
{
        PRINTK_I2C_GPIO(" i2c_gpio_open succed !\n");
        return 0;
}



static int i2c_gpio_release(struct inode *inode, struct file *filp)
{	
        PRINTK_I2C_GPIO("i2c_gpio_release \n");
        return 0;
}

/*
static void read_time_handle(unsigned long arg)
{
                atomic_set(&recv_timeout, 1);
                wake_up_interruptible(&i2c_gpio_port->rx_wait);

}



static void read_time1_handle(unsigned long arg)
{
        wake_up_interruptible(&i2c_gpio_port->isr_wait);

}*/



static ssize_t i2c_gpio_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
        unsigned char  temp;
        unsigned char  buf1[1024];
        unsigned int count=0;
     //   DECLARE_WAITQUEUE(wait1, current);        
	//       read_or_not =1;
        
        
        
        if(size > MAX_BUF)
                {
                        printk("read count > 1024");
                        return -EFAULT;
                }
        
   //     add_wait_queue(&i2c_gpio_port->rx_wait, &wait1);
	enable_irq(irq_num);
	msleep(1000);       

        retry1:
        
                if(i2c_gpio_port->rx_head   !=  i2c_gpio_port->rx_tail)
                        {
 
                                temp = BUF_TAIL_RX;
                                i2c_gpio_port->rx_tail = INCBUF(i2c_gpio_port->rx_tail,MAX_BUF);
                                *(buf1 + count) = temp;
                                count++;
                                
                                
                                if(count == size)
                                        {
                                                
                                                copy_to_user((void *)buf, buf1, count);
                                                if(1 == atomic_read(&recv_timeout))
                                                        {
                                                                atomic_set(&recv_timeout,0);
                                                        }
                                                      goto  out;
                                        }
                                 else
                                         {

                                                goto retry1;
                                                                
                                         }
                                                                
                        }
                else
                        {
                        
                                if(1 == atomic_read(&recv_timeout))
                                        {
                                        
                                        atomic_set(&recv_timeout, 0);
                                        copy_to_user((void *)buf, buf1, count);
                       
                                                      goto  out;
                                        
                                        }
                                
                                if(filp->f_flags & O_NONBLOCK)
                                        {
                                               // remove_wait_queue(&i2c_gpio_port->rx_wait,&wait1);
                                                return -EAGAIN;
                                        }
                                else
                                        {
                                                
                                              /*  if(read_or_not == 1)
        	                                 {
        	                                     //  read_timer1.expires  = jiffies + HZ/50;
							 read_timer1.expires  = jiffies + HZ/100;
							
         		                               add_timer(&read_timer1);
                                                }
                                                if(read_or_not == 1)
                                                {
                                                      //  read_timer.expires  = jiffies + HZ/10;
							 read_timer.expires  = jiffies + HZ/100;
							
                                                        add_timer(&read_timer);
                                                }

                                                interruptible_sleep_on(&i2c_gpio_port->rx_wait);
						*/
                                        
                                                goto retry1;
                                        }
                        }

        out: 
  //      remove_wait_queue(&i2c_gpio_port->rx_wait,&wait1);
  //      read_or_not = 0;
	disable_irq(irq_num);
        return count;
}

int  ready_to_write(void)
{
        if(xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_LSR][0]) & 0x20)
        {
                return 1;
        }
        else {        
                mdelay(5);        
                return 0;
        }
}

static ssize_t i2c_gpio_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
        unsigned char buf2[MAX_BUF];
        unsigned int count = 0;
        unsigned char temp;
   /*     DECLARE_WAITQUEUE(wait2, current);
        add_wait_queue(&i2c_gpio_port->tx_wait, &wait2);*/
        
         if(size > MAX_BUF)
                 {
                        printk("write size to large \n");
                        return -EFAULT;
                 }
	enable_irq(irq_num);
	msleep(1000);
  	      copy_from_user(buf2, (void *) buf, size);
retry:
	
        while( ready_to_write() )
                {
			temp = *(buf2 + count );
                        xor_write(i2c_gpio_port->client, reg_info[XR20M1170REG_THR][0], temp);
                        count += 1;
                        if(count == size )
                        {
                                goto out;
                        }
                                        
                }
        
  //      interruptible_sleep_on(&i2c_gpio_port->tx_wait);
        goto retry;
        
        out:
  //      remove_wait_queue(&i2c_gpio_port->tx_wait,&wait2);
	disable_irq(irq_num);
        return count;
}



static long i2c_gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

        int result ;
        unsigned int __user *argp = (unsigned int __user *)arg;
       
        switch(cmd)
                {
                        case GPIOIOC_SETBAUDERATE:
                                copy_from_user(&result, argp, sizeof(result));
				
                                i2c_set_baudrate(result);
				break;
                        default: break;


                }
        
        return 0;
}

unsigned int i2c_set_baudrate(unsigned  int baudrate)
{
        
        unsigned char dld_reg;
        unsigned int baud = baudrate;
        unsigned int temp;

        unsigned long  required_divisor2;
        unsigned short  required_divisor ;
        
        required_divisor2  = (unsigned long) ((XR20M1172_CLOCK * 16)/(PRESCALER * SAMPLE_RATE * baud));
        required_divisor    = required_divisor2 / 16;
        dld_reg        =(char)(required_divisor2   - required_divisor*16);
        dld_reg   &= ~(0x3 << 4);  //16X

        xor_write(i2c_gpio_port->client, reg_info[XR20M1170REG_LCR][0], 0x83);
        
        printk("dld_reg is :%02x\n",dld_reg);
        xor_write(i2c_gpio_port->client, reg_info[XR20M1170REG_DLM][0], required_divisor >> 8);
        temp = xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_DLM][0]);
        PRINTK_I2C_GPIO("i2c_gpio read DLM  data is :%02x\n",temp);

        xor_write(i2c_gpio_port->client, reg_info[XR20M1170REG_DLL][0], required_divisor &0xff);
        temp = xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_DLL][0]);
        PRINTK_I2C_GPIO("i2c_gpio read DLL  data is :%02x\n",temp);

        xor_write(i2c_gpio_port->client, reg_info[XR20M1170REG_DLD][0], dld_reg);
        temp = xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_DLD][0]);
        PRINTK_I2C_GPIO("i2c_gpio read DLD  data is :%02x\n",temp);
     
        xor_write(i2c_gpio_port->client, reg_info[XR20M1170REG_LCR][0], 0x03);
        return 0;
}



void i2c_gpio_init(struct i2c_client *client)
{
        
        unsigned char temp;
     
/*LCR         0000 0011 */
        temp =0xBF;
	xor_write(client,reg_info[XR20M1170REG_LCR][0], temp);
	temp = xor_read(client, reg_info[XR20M1170REG_LCR][0]);
        printk("i2c_gpio read LCR  data is :%02x\n",temp);
        
/*EFR  */
        temp = xor_read(client, reg_info[XR20M1170REG_EFR][0]);
        temp  |=( 1 << 4) ;

         xor_write(client, reg_info[XR20M1170REG_EFR][0], temp);
         temp = xor_read(client, reg_info[XR20M1170REG_EFR][0]);
        PRINTK_I2C_GPIO("i2c_gpio read EFR  data is :%02x\n",temp);



        xor_write(client, reg_info[XR20M1170REG_LCR][0], 0x83);
/*MCR        */
        temp = (1 << 3)  ;

        xor_write(client, reg_info[XR20M1170REG_MCR][0], temp);
        temp = xor_read(client, reg_info[XR20M1170REG_MCR][0]);
        PRINTK_I2C_GPIO("i2c_gpio read MCR  data is :%02x\n",temp);
/* DLD  */        
        

        i2c_set_baudrate(9600);


/*LCR         0000 0011 */
        xor_write(client, reg_info[XR20M1170REG_LCR][0], 0x03);
        temp = xor_read(client, reg_info[XR20M1170REG_LCR][0]);
        PRINTK_I2C_GPIO("i2c_gpio read LCR  data is :%02x\n",temp);

/*IER&*/
        temp =3;
        xor_write(client, reg_info[XR20M1170REG_IER][0], temp);
        temp = xor_read(client, reg_info[XR20M1170REG_IER][0]);
        PRINTK_I2C_GPIO("i2c_gpio read IER  data is :%02x\n",temp);        
        
/*FCR*/
        xor_write(client, reg_info[XR20M1170REG_FCR][0], 0x47);
        printk("The FCR is %02x\n",xor_read(client, reg_info[XR20M1170REG_IOCONTROL][0]));


/*EFCR*/
        temp = xor_read(client,reg_info[XR20M1170REG_EFCR][0]);
        printk("The EFCR is %02x\n", temp);
        temp |= (1<<4)| (1<<5);
        xor_write(client,reg_info[XR20M1170REG_EFCR][0],temp);

        
/*IOCRL*/
        
         temp = xor_read(client, reg_info[XR20M1170REG_IOCONTROL][0]);
        
        PRINTK_I2C_GPIO("i2c_gpio IOCTL data is :%02x\n",temp);

        
         xor_write(client, reg_info[XR20M1170REG_IODIR][0], 0xff);        
         temp = xor_read(client, reg_info[XR20M1170REG_IODIR][0]);
        PRINTK_I2C_GPIO("i2c_gpio IODIR data is :%02x\n",temp);

        
        
        
        xor_write(client, reg_info[XR20M1170REG_IOSTATE][0], 0xaa);
        temp = xor_read(client, reg_info[XR20M1170REG_IOSTATE][0]);
        PRINTK_I2C_GPIO("i2c_gpio read data is  IOSTATE :%02x\n",temp);

}

int i2c_gpio_irq_init(void)
{

	int err,irq_i2c_uart;
	err = gpio_request(80,"i2c_uart_interrupt_pin");
	if (err){
	printk(KERN_ALERT "initilization of GPIO 80 failed: \n");
	return -1;}

	if (gpio_is_valid(80)){
 		err = gpio_direction_input(80);
		if (err)
		printk(KERN_ALERT "GPIO 80 interrupt pin for i2c_uart pin direction setting failed: \n");}

	printk(KERN_ALERT "GPIO 80 Opend for i2c_uart interrupt Acces: \n");
	printk(KERN_ALERT "Sreenivas 80 pin Opend for interrupt of i2c-uart Acces: \n");
	irq_i2c_uart   = gpio_to_irq(80);
	return irq_i2c_uart;
}

void xr20m1172_do_work(void *data)
{
        
        PRINTK_I2C_GPIO("I am in work struct  1111111111111!\n");
}


 static irqreturn_t i2c_gpio_handler(int irq, void *handle)
{
	i2c_gpio_port = handle;
    //    wake_up_interruptible(&i2c_gpio_port->isr_wait);
	tasklet_schedule( &i2c_uart_tasklet); 
        return IRQ_HANDLED;

}


static int __devinit i2c_gpio_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int result ,err;
        int retval ;
	int irq_req;
	dev_t devno;
 //       struct task_struct * i2c_thread = NULL;  
        spin_lock_init(&xr20m1172_lock);
        spin_lock_init(&global_lock);
        
        devno = MKDEV(i2c_gpio_major,0);
        i2c_gpio_port = kmalloc(sizeof(struct xr20m1172_port ),GFP_KERNEL);
	i2c_set_clientdata(client, i2c_gpio_port);
	device_init_wakeup(&client->dev, 1);
        printk(KERN_NOTICE "i2c_uart probe called sucessfully");
        if(!i2c_gpio_port)
                {
                        result = -ENOMEM;
                        goto fail_malloc;
                }
        memset(i2c_gpio_port,0,sizeof(struct xr20m1172_port));
        
        if(i2c_gpio_major)
                result = register_chrdev_region(devno,1,I2C_GPIO_DRIVER_NAME);
        else
                {
                        result = alloc_chrdev_region(&devno,0,1,I2C_GPIO_DRIVER_NAME);
                        i2c_gpio_major = MAJOR(devno);
                }
        if(result < 0)
                goto fail ;
        cdev_init(&i2c_gpio_port->cdev,&i2c_gpio_fops);
        i2c_gpio_port->cdev.owner = THIS_MODULE;
        i2c_gpio_port->cdev.ops   = &i2c_gpio_fops;
	i2c_gpio_port->client = client;
        err = cdev_add(&i2c_gpio_port->cdev,devno,1);
        if(err)
                printk(KERN_NOTICE "i2c gpio cdev add failed");

          my_class = class_create(THIS_MODULE, "I2C_GPIO_DRIVER_NAME");
        if(IS_ERR(my_class))
                {
                        printk("Err :failed int creating class !");
                        return -1;
                }

	device_create(my_class, NULL, MKDEV(i2c_gpio_major,0),NULL,"i2c_uart");

        i2c_gpio_port->rx_head = i2c_gpio_port->rx_tail =0;
        i2c_gpio_port->tx_head = i2c_gpio_port->tx_tail = 0;
        init_waitqueue_head(&i2c_gpio_port->rx_wait);
        init_waitqueue_head(&i2c_gpio_port->tx_wait);
        init_waitqueue_head(&i2c_gpio_port->isr_wait);
        
	tasklet_init( &i2c_uart_tasklet, isr_thread, (unsigned long)i2c_gpio_port );          
    
         irq_req = i2c_gpio_irq_init();
	retval = request_irq(irq_req, i2c_gpio_handler, IRQF_SHARED, "I2C_UART_DRV", i2c_gpio_port);
        if(retval)
                {
                        printk("request i2c_uart IRQ failed !");
			
                }
	
		
	 i2c_gpio_init(client); 
	printk(KERN_NOTICE "sreenivas i2c_uart chip initilization called sucessfully");
//	 init_timer(&read_timer);
//        init_timer(&read_timer1);
        
//        read_timer.function = &read_time_handle;
//        read_timer1.function = & read_time1_handle;        
//        i2c_thread = kthread_run(isr_thread, NULL, "i2c-thread");
//	if (i2c_thread->state == -1)
//	{printk(KERN_NOTICE "i2c_uart isr thread call failled");}
//      
	 printk(KERN_NOTICE "i2c_uart probe called sucessfully");
        
	printk("the FCR is %02x\n", xor_read(i2c_gpio_port->client, reg_info[XR20M1170REG_FCR][0]));
        irq_num = irq_req;
	disable_irq(irq_req);
        return 0;
       
fail_malloc: 
        kfree(i2c_gpio_port);
fail:        
        unregister_chrdev_region(devno,1); 
       return result;
           
}



static struct file_operations i2c_gpio_fops ={
        .owner        = THIS_MODULE,
        .open        = i2c_gpio_open,
        .read        = i2c_gpio_read,
        .write        = i2c_gpio_write,
        .compat_ioctl = i2c_gpio_ioctl,
        .release        = i2c_gpio_release,
};


static const struct i2c_device_id i2c_uart_id[] = {
         { "i2c_uart", 0 },
         { "i2c_uart", 0 },
         { }
 };
MODULE_DEVICE_TABLE(i2c, i2c_uart_id);

static struct i2c_driver i2c_gpio_driver = {
         .driver = {
                 .name = I2C_GPIO_DRIVER_NAME,
              },
         .probe    = i2c_gpio_probe,
         .id_table = i2c_uart_id,
 };




static int __init i2c_uart_init(void)
 {
         return i2c_add_driver(&i2c_gpio_driver);
 }
 module_init(i2c_uart_init);
 
static void __exit i2c_uart_exit(void)
 {
         i2c_del_driver(&i2c_gpio_driver);
        cdev_del(&i2c_gpio_port->cdev);
        class_destroy(my_class);
        kfree(i2c_gpio_port);
        unregister_chrdev_region(MKDEV(i2c_gpio_major,0),1);
 //       del_timer(&read_timer);
//	del_timer(&read_timer1);
        PRINTK_I2C_GPIO("unregister i2c  gpio driver !");
 }
 module_exit(i2c_uart_exit);


MODULE_LICENSE("GPL"); // this avoids kernel taint warning
MODULE_DESCRIPTION("i2c_uart_driver");
MODULE_AUTHOR("Sreenivas");

