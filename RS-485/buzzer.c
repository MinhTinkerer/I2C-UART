/*
 * Simple character interface to GPIO81 (i.e GPIO5_1).  Allows you to read/write values and
 * control the direction is fixed.
 *
 ***********************************************************************************
 * To use, just declare in your board resources: (i.e board_da850.c)
 ***********************************************************************************	
 * static struct resource foo_resources[] = {
 *     .start = 0, //(this value decideds the inital value)
 *     .flags = IORESOURCE_IRQ,
 * };
 * static struct platform_device foo_dev = {
 *     .name = "buzzer",
 *     .num_resources = 1,
 *     .resource = &foo_resources
 * };
 *************************************************************************************
 *
 * This will setup GPIO81 (i.e GPIO5_1) (inclusive) for use by this driver.
 *
 *
 *
 *************************************************************************************
 * In the board_da850.c file, in function  static __init void da850_evm_init(void)
 *************************************************************************************
 *
 *	    ret = davinci_cfg_reg_list(da850_my_gpio_pins); 
 *  if (ret)
 *      pr_warning("da850_evm_init: my gpio mux setup failed: %d\n", ret); 
 *
 *	    ret =platform_device_register(&foo_dev); 
 *  if (ret)
 *      pr_warning("da850_evm_init: my gpio driver initiliatation failed: %d\n", ret); 
 *	pr_warning("Buzzer device driver was sucessfully called by sreenivas:\n");
 *
 ***************************************************************************************
 *
 * 
 *Developed By Kowtharapu Sreenivas, Stesalit ltd, hyderabad
 * 
 */

/*included for platform device driver*/

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/slab.h>
//#include <stdio.h>

/*included for platform device driver*/

#include <linux/module.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>

#define DRVNAME "buzzer"

// module attributes
MODULE_LICENSE("GPL"); // this avoids kernel taint warning
MODULE_DESCRIPTION("buzzer");
MODULE_AUTHOR("Sreenivas");


static int times = 0;


// protoypes,else the structure initialization that follows fail
static int buzzer_open(struct inode *, struct file *);
static int buzzer_rls(struct inode *, struct file *);
static ssize_t buzzer_read(struct file *, char *, size_t, loff_t *);
static ssize_t buzzer_write(struct file *, const char *, size_t, loff_t *);

static struct class *gpiodev_class;
struct resource *buzz_resource;

// structure containing callbacks
static struct file_operations fops = 
{
	.read = buzzer_read, // address of dev_read
	.open = buzzer_open,  // address of dev_open
	.write = buzzer_write, // address of dev_write 
	.release = buzzer_rls, // address of dev_rls
};




// called when 'open' system call is done on the device file
static int buzzer_open(struct inode *inod,struct file *fil) 
{	
	
	times++;
	printk(KERN_ALERT "Device Buzzer Accessed %d times\n",times);
	return 0;
}

// called when 'read' system call is done on the device file
static ssize_t buzzer_read(struct file *filp,char *buff,size_t len,loff_t *off)
{

	int val;
	printk(KERN_ALERT "Nothing to read: \n");
	 val = gpio_get_value(81);
	printk(KERN_ALERT "Present GPIO81 Value is: %d", val);
	return val;	
}

// called when 'write' system call is done on the device file
static ssize_t buzzer_write(struct file *filp,const char *buff,size_t len,loff_t *off)
{	

/*	int err;
	err = gpio_request(81,"Buzzer");
	if (err){
	printk(KERN_ALERT "Buzzer initilization of GPIO 81 failed: \n");
	return -1;}

	if (gpio_is_valid(81)){
 		err = gpio_direction_output(81, 0);
		if (err)
		printk(KERN_ALERT "GPIO direction setting failed: \n");}

	printk(KERN_ALERT "GPIO 81 Opend for Buzzer Acces: \n");
	printk(KERN_ALERT "Sreenivas Device Opend for Buzzer Acces: \n");


	gpio_set_value(81, buzz_resource->start);
	if (err)
	printk(KERN_ALERT "Setting value to GPIO 81 is failed : \n");	*/
	
	int val;
	val = buff[0] - 48;  	//ASCII to Integer
	gpio_set_value(81, val);
			
	printk(KERN_ALERT "Setting value to GPIO 81 is %d : \n", val);

		
	return len;

}

// called when 'close' system call is done on the device file
static int buzzer_rls(struct inode *inod,struct file *fil)
{	
	gpio_free(81);
	printk(KERN_ALERT"Buzzer: Device closed\n");
	return 0;
}



/*******************for platfrom device driver *********/

/**
 *	simple_gpio_probe - setup the range of GPIOs
 *
 *	Create a character device for the GPIO81 
 *	
 */
static int __devinit buzzer_probe(struct platform_device *pdev)
{
	int ret;
	int err;
	int result = 0;
	
	ret = register_chrdev(200, DRVNAME, &fops);
	        if (ret)
	        {
	                printk(KERN_ERR DRVNAME ": Error whilst opening %s \n", DRVNAME);
	                result = -ENODEV;
	                goto out;
	        }
	gpiodev_class = class_create(THIS_MODULE, DRVNAME);
	device_create(gpiodev_class, NULL, MKDEV(200, 0), pdev, DRVNAME);
	printk(KERN_INFO DRVNAME ": gpio device registered with major 200\n");

	buzz_resource = platform_get_resource(pdev, IORESOURCE_IRQ, 0); 
	/******************************************************************/
	

	
	err = gpio_request(81,"Buzzer");
	if (err){
	printk(KERN_ALERT "Buzzer initilization of GPIO 81 failed: \n");
	return -1;}

	if (gpio_is_valid(81)){
 		err = gpio_direction_output(81, 0);
		if (err)
		printk(KERN_ALERT "GPIO direction setting failed: \n");}

	printk(KERN_ALERT "GPIO 81 Opend for Buzzer Acces: \n");
	printk(KERN_ALERT "Sreenivas Device Opend for Buzzer Acces: \n");


	gpio_set_value(81, buzz_resource->start);
	if (err)
	printk(KERN_ALERT "Setting value to GPIO 81 is failed : \n");





	/*********************************************************************/
		
	out:
	        return result;

}

/**
 *	simple_gpio_remove - break down the range of GPIOs
 *
 *	Release the character device and related pieces for this range of GPIOs.
 */
static int __devexit buzzer_remove(struct platform_device *pdev)
{
	
	unregister_chrdev(200, DRVNAME);
	return 0;
}



struct platform_driver buzzer_driver = {
	.probe   = buzzer_probe,
	.remove  = __devexit_p(buzzer_remove),
	.driver  = {
		.name = DRVNAME,
	}
};



/******************for platform driver**************/












// called when module is loaded, similar to main()
int init_module(void)
{
	        int ret = platform_driver_register(&buzzer_driver);
	        if (ret)
	                printk(KERN_INFO DRVNAME ": Error registering platfom driver!\n");

		
	        return ret;
}


// called when module is unloaded, similar to destructor in OOP
void cleanup_module(void)
{
	platform_driver_unregister(&buzzer_driver);
	printk(KERN_ALERT "Device: Buzzer unregistered...\n");
}








module_init(init_module);
module_exit(cleanup_module);
