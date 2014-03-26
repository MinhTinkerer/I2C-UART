
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <linux/i2c/adp5588.h>

struct adp5588_kpad {
        struct i2c_client *client;
        struct input_dev *input;
        struct delayed_work work;
        unsigned long delay;
        unsigned short keycode[ADP5588_KEYMAPSIZE];
        const struct adp5588_gpi_map *gpimap;
        unsigned short gpimapsize;
#ifdef CONFIG_GPIOLIB
        unsigned char gpiomap[ADP5588_MAXGPIO];
        bool export_gpio;
        struct gpio_chip gc;
        struct mutex gpio_lock; /* Protect cached dir, dat_out */
        u8 dat_out[3];
        u8 dir[3];
#endif
};
static int adp5588_read(struct i2c_client *client, u8 reg)
{
        int ret = i2c_smbus_read_byte_data(client, reg);
        if (ret < 0)
               dev_err(&client->dev, "Read Error\n");

        return ret;
}
static int adp5588_write(struct i2c_client *client, u8 reg, u8 val)
{
        return i2c_smbus_write_byte_data(client, reg, val);
}
#ifdef CONFIG_GPIOLIB
static int adp5588_gpio_get_value(struct gpio_chip *chip, unsigned off)
{
        struct adp5588_kpad *kpad = container_of(chip, struct adp5588_kpad, gc);
       /****************************************************************/
        return !!(adp5588_read(kpad->client, GPIO_DAT_STAT1 + bank) & bit);
}
static void adp5588_gpio_set_value(struct gpio_chip *chip,
                                   unsigned off, int val)
{
        struct adp5588_kpad *kpad = container_of(chip, struct adp5588_kpad, gc);
        unsigned int bank = ADP5588_BANK(kpad->gpiomap[off]);
        unsigned int bit = ADP5588_BIT(kpad->gpiomap[off]);

        mutex_lock(&kpad->gpio_lock);
       /****************************************************************/
         mutex_unlock(&kpad->gpio_lock);
 }
 
 static int adp5588_gpio_direction_input(struct gpio_chip *chip, unsigned off)
 {
         struct adp5588_kpad *kpad = container_of(chip, struct adp5588_kpad, gc);
         unsigned int bank = ADP5588_BANK(kpad->gpiomap[off]);
         unsigned int bit = ADP5588_BIT(kpad->gpiomap[off]);
         int ret;
 
         mutex_lock(&kpad->gpio_lock);
 
              /****************************************************************/
         mutex_unlock(&kpad->gpio_lock);
 
         return ret;
 }
 
 static int adp5588_gpio_direction_output(struct gpio_chip *chip,
                                          unsigned off, int val)
 {
         struct adp5588_kpad *kpad = container_of(chip, struct adp5588_kpad, gc);
         unsigned int bank = ADP5588_BANK(kpad->gpiomap[off]);
         unsigned int bit = ADP5588_BIT(kpad->gpiomap[off]);
         int ret;
 
         mutex_lock(&kpad->gpio_lock);
 
         /****************************************************************/
         mutex_unlock(&kpad->gpio_lock);
 
         return ret;
 }
 
 static int __devinit adp5588_build_gpiomap(struct adp5588_kpad *kpad,
                                 const struct adp5588_kpad_platform_data *pdata)
 {
         bool pin_used[ADP5588_MAXGPIO];
         int n_unused = 0;
         int i;
 
         memset(pin_used, 0, sizeof(pin_used));
 
          /****************************************************************/
 
         return n_unused;
 }
 
 static int __devinit adp5588_gpio_add(struct adp5588_kpad *kpad)
 {
         struct device *dev = &kpad->client->dev;
         const struct adp5588_kpad_platform_data *pdata = dev->platform_data;
         const struct adp5588_gpio_platform_data *gpio_data = pdata->gpio_data;
         int i, error;
 
         /****************************************************************/
         }
 
         return 0;
 }
 
 static void __devexit adp5588_gpio_remove(struct adp5588_kpad *kpad)
 {
         struct device *dev = &kpad->client->dev;
         const struct adp5588_kpad_platform_data *pdata = dev->platform_data;
         const struct adp5588_gpio_platform_data *gpio_data = pdata->gpio_data;
         int error;
 
      /****************************************************************/
 }
 #else
 static inline int adp5588_gpio_add(struct adp5588_kpad *kpad)
 {
         return 0;
 }
 
 static inline void adp5588_gpio_remove(struct adp5588_kpad *kpad)
 {
 }
 #endif
 
static void adp5588_report_events(struct adp5588_kpad *kpad, int ev_cnt)
 {
         int i, j;
 
         /****************************************************************/
 }

 static void adp5588_work(struct work_struct *work)
 {
         struct adp5588_kpad *kpad = container_of(work,
                                                 struct adp5588_kpad, work.work);
         struct i2c_client *client = kpad->client;
         int status, ev_cnt;
 
         status = adp5588_read(client, INT_STAT);
  /****************************************************************/
         adp5588_write(client, INT_STAT, status); /* Status is W1C */
 }
 
 static irqreturn_t adp5588_irq(int irq, void *handle)
 {
         struct adp5588_kpad *kpad = handle;
 
         /*
          * use keventd context to read the event fifo registers
          * Schedule readout at least 25ms after notification for
          * REVID < 4
          */
 
         schedule_delayed_work(&kpad->work, kpad->delay);
 
         return IRQ_HANDLED;
 }
 
 static int __devinit adp5588_setup(struct i2c_client *client)
 {
         const struct adp5588_kpad_platform_data *pdata = client->dev.platform_data;
         const struct adp5588_gpio_platform_data *gpio_data = pdata->gpio_data;
         int i, ret;
         unsigned char evt_mode1 = 0, evt_mode2 = 0, evt_mode3 = 0;
 
       /****************************************************************/
 
         return 0;
 }
 
 static void __devinit adp5588_report_switch_state(struct adp5588_kpad *kpad)
 {
         int gpi_stat1 = adp5588_read(kpad->client, GPIO_DAT_STAT1);
         int gpi_stat2 = adp5588_read(kpad->client, GPIO_DAT_STAT2);
         int gpi_stat3 = adp5588_read(kpad->client, GPIO_DAT_STAT3);
         int gpi_stat_tmp, pin_loc;
         int i;
 
         /****************************************************************/
 }
 
 
 static int __devinit adp5588_probe(struct i2c_client *client,
                                         const struct i2c_device_id *id)
 {
         struct adp5588_kpad *kpad;
         const struct adp5588_kpad_platform_data *pdata = client->dev.platform_data;
         struct input_dev *input;
         unsigned int revid;
         int ret, i;
         int error;
 
         /****************************************************************/
         return error;
 }
 
 static int __devexit adp5588_remove(struct i2c_client *client)
 {
         struct adp5588_kpad *kpad = i2c_get_clientdata(client);
 
         adp5588_write(client, CFG, 0);
         free_irq(client->irq, kpad);
         cancel_delayed_work_sync(&kpad->work);
         input_unregister_device(kpad->input);
         adp5588_gpio_remove(kpad);
         kfree(kpad);
 
         return 0;
 }
 
 #ifdef CONFIG_PM
 static int adp5588_suspend(struct device *dev)
 {
         struct adp5588_kpad *kpad = dev_get_drvdata(dev);
         struct i2c_client *client = kpad->client;
 
         disable_irq(client->irq);
         cancel_delayed_work_sync(&kpad->work);
 
         if (device_may_wakeup(&client->dev))
                 enable_irq_wake(client->irq);
 
         return 0;
 }
 
 static int adp5588_resume(struct device *dev)
 {
         struct adp5588_kpad *kpad = dev_get_drvdata(dev);
         struct i2c_client *client = kpad->client;
 
         if (device_may_wakeup(&client->dev))
                 disable_irq_wake(client->irq);
 
         enable_irq(client->irq);
 
         return 0;
 }
 
 static const struct dev_pm_ops adp5588_dev_pm_ops = {
         .suspend = adp5588_suspend,
         .resume  = adp5588_resume,
 };
 #endif
 
 static const struct i2c_device_id adp5588_id[] = {
         { "adp5588-keys", 0 },
         { "adp5587-keys", 0 },
         { }
 };
 MODULE_DEVICE_TABLE(i2c, adp5588_id);
 
 static struct i2c_driver adp5588_driver = {
         .driver = {
                 .name = KBUILD_MODNAME,
 #ifdef CONFIG_PM
                 .pm   = &adp5588_dev_pm_ops,
 #endif
         },
         .probe    = adp5588_probe,
         .remove   = __devexit_p(adp5588_remove),
         .id_table = adp5588_id,
 };
 
 static int __init adp5588_init(void)
 {
         return i2c_add_driver(&adp5588_driver);
 }
 module_init(adp5588_init);
 
 static void __exit adp5588_exit(void)
 {
         i2c_del_driver(&adp5588_driver);
 }
 module_exit(adp5588_exit);
 
 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
 MODULE_DESCRIPTION("ADP5588/87 Keypad driver");
 
