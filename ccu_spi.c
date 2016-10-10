/*******************************************************************************
 *                     SPIRI APS PROPRIETARY INFORMATION
 *
 * Property of Spiri ApS, Unauthorized reproduction and/or distribution
 * is strictly prohibited.  This product  is  protected  under  copyright  law
 * and  trade  secret law as an unpublished work.
 * (C) Copyright Spiri ApS.  All rights reserved.
 *
 * Description      :   Internal SPI communication layer between the  
 *                      microcontroller and the Computation Module in the 
 *                      Central Communication Unit (CCU).
 *
 * Author           :   Nikolaj Due Oesterbye
 * Date             :   September 14 2016
 *
 ******************************************************************************/

//#include <linux/device.h>
//#include <linux/fs.h>
//#include <linux/kernel.h>
//#include <linux/delay.h>
//#include <linux/slab.h>
//#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/interrupt.h>
//  #include <linux/unistd.h>
//#include <linux/workqueue.h>
//  #include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>

#include <asm/ioctls.h>
//#include <asm/uaccess.h>

#include "ccu_spi.h"

#define MAX_READ_BUFFER_LEN     2048
#define READ_LEN                128
#define MAX_WRITE_BUFFER_LEN    256


struct ccu_spi_drvdata {
    struct spi_device *spi;
    struct workqueue_struct *workqueue;
    struct work_struct	irq_work;
    wait_queue_head_t data_queue;
    u8 rx_buffer[MAX_READ_BUFFER_LEN];
    u16 rx_buffer_len;
    u8 tx_buffer[MAX_WRITE_BUFFER_LEN];
    u16 tx_buffer_len;
    u32 cts_gpio;
    u32 dr_irq;
    u8 exit;
    u8 customers;
};

static struct ccu_spi_drvdata *ccu_spi_drvdata;

#if 0
static void pr_buff(u8 *caption, u8 *buff, int len)
{
	int i;

	printk("[ccu_spi] %s: ", caption);
	for (i = 0; i < len; i++) {
		printk("0x%02x", buff[i]);
		if (i < (len-1))
			printk(", ");
	}
	printk("\n");
}
#endif


/*static void write_to_uc(void)
{
    int ret;

    ret = spi_write(ccu_spi_drvdata->spi, ccu_spi_drvdata->tx_buffer, ccu_spi_drvdata->tx_buffer_len);
    if (ret) {
        dev_err(&ccu_spi_drvdata->spi->dev, "[ccu_spi] Failed SPI write. ret = %d\n", ret);
        return;
    }
    ccu_spi_drvdata->tx_buffer_len = 0;
    //mod_timer(&ccu_spi_drvdata->read_timer, jiffies+msecs_to_jiffies(10));
}*/

static void read_from_uc(struct work_struct *work)
{
    int ret;
    
    //printk("[ccu_spi] read_from_uc - begin\n");
    if (unlikely((ccu_spi_drvdata->rx_buffer_len+READ_LEN) > MAX_READ_BUFFER_LEN) ){
        dev_err(&ccu_spi_drvdata->spi->dev, "[ccu_spi] RX buffer full\n");
        wake_up_interruptible(&ccu_spi_drvdata->data_queue);
        return;
    }
    do {
        ret = spi_read(ccu_spi_drvdata->spi, &ccu_spi_drvdata->rx_buffer[ccu_spi_drvdata->rx_buffer_len], READ_LEN);
        if (unlikely(ret)) {
	        dev_err(&ccu_spi_drvdata->spi->dev, "[ccu_spi] Failed SPI read. ret = %d\n", ret);
	        return;
        }
        ccu_spi_drvdata->rx_buffer_len += READ_LEN;
    } while ((0xFF != ccu_spi_drvdata->rx_buffer[ccu_spi_drvdata->rx_buffer_len-1]) && (0xFF != ccu_spi_drvdata->rx_buffer[ccu_spi_drvdata->rx_buffer_len-2]) && (0xFF != ccu_spi_drvdata->rx_buffer[ccu_spi_drvdata->rx_buffer_len-3]) && (MAX_READ_BUFFER_LEN >= (ccu_spi_drvdata->rx_buffer_len+READ_LEN)));
    wake_up_interruptible(&ccu_spi_drvdata->data_queue);
    //printk("\n[ccu_spi] read_from_uc - end\n\n\n");
}

// TODO: add gpio interrupt function that queues the reading function on the work queue.
irqreturn_t dr_interrupt_handler(int irq, void *dev_id)
{
	printk(KERN_DEBUG "interrupt received (irq: %d)\n", irq);

	if (irq == gpio_to_irq(ccu_spi_drvdata->dr_irq)) {
			printk(KERN_DEBUG "[ccu_spi] Data ready interrupt received, queueing work READ\n");
			queue_work(ccu_spi_drvdata->workqueue, &ccu_spi_drvdata->irq_work);
			return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

#if 0
static u32 read_timer_next_timeout(void)
{
    return ((((jiffies - ccu_spi_drvdata->read_timer_base_jiffies) / ccu_spi_drvdata->read_timer_interval_jiffies) + 1) * ccu_spi_drvdata->read_timer_interval_jiffies) + ccu_spi_drvdata->read_timer_base_jiffies;
}

static void read_timer_func(unsigned long data)
{
    if (unlikely(ccu_spi_drvdata->exit)) {
        printk("[ccu_spi] Killing timer\n");
        complete(&ccu_spi_drvdata->read_timer_done);
    } else {
        //printk("[ccu_spi] read_timer_func()\n");
        queue_work(ccu_spi_drvdata->workqueue, &ccu_spi_drvdata->irq_work);
        mod_timer(&ccu_spi_drvdata->read_timer, read_timer_next_timeout());
    }
}
#endif

static ssize_t 
ccu_spi_read(struct file *file, char __user *buff, size_t len, loff_t *offset)
{
    int ret;
    //printk("[ccu_spi] enter READ function\n");
    ret = wait_event_interruptible(ccu_spi_drvdata->data_queue, 
            (0 != ccu_spi_drvdata->rx_buffer_len));

    if (unlikely(ret))
        goto out;

    if (unlikely(copy_to_user(buff, &ccu_spi_drvdata->rx_buffer, 
                    ccu_spi_drvdata->rx_buffer_len))) {
        dev_err(&ccu_spi_drvdata->spi->dev, 
            "[ccu_spi] Failed to copy rx_buffer to user space\n");
        ret = -EFAULT;
        goto out;
    }

    ret = ccu_spi_drvdata->rx_buffer_len;
    ccu_spi_drvdata->rx_buffer_len = 0;

out:
    //printk("[ccu_spi] exit READ ret = %d\n", ret);
    return ret;
}

static ssize_t ccu_spi_write(struct file *file, const char __user *buff, 
                size_t len, loff_t *offset)   
{
    //printk("[ccu_spi] Trying to write %d bytes. TX buffer already contains %d bytes\n", len, ccu_spi_drvdata->tx_buffer_len);
    if (unlikely(len > MAX_WRITE_BUFFER_LEN))
        return -ENOMEM;

    if (unlikely(copy_from_user(&ccu_spi_drvdata->tx_buffer[0], buff, len))) {
        dev_err(&ccu_spi_drvdata->spi->dev, 
            "[ccu_spi] Failed to copy tx_buffer from user space\n");
        return -EFAULT;
    }
    ccu_spi_drvdata->tx_buffer_len = len;

    if (gpio_get_value(ccu_spi_drvdata->cts_gpio))
        return -EBUSY;
    
    if (unlikely(spi_write(ccu_spi_drvdata->spi, ccu_spi_drvdata->tx_buffer, 
                                            ccu_spi_drvdata->tx_buffer_len))) {
        dev_err(&ccu_spi_drvdata->spi->dev, "[ccu_spi] Failed SPI write.\n");
        return -EIO;
    }

    return len;
}

static long 
ccu_spi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)   
{
    switch (cmd) {
    /*case CCU_SPI_SYNC_READ:
        if (arg < 0x64 || arg > 0xFFFFFFFF)
            return -EFAULT;
        ccu_spi_drvdata->read_timer_base_jiffies = jiffies;
        ccu_spi_drvdata->read_timer_interval_jiffies = msecs_to_jiffies(arg);
        mod_timer(&ccu_spi_drvdata->read_timer, jiffies+1);
        break;*/
    case FIONREAD:
        printk("[ccu_spi] returning buffer len %d\n", ccu_spi_drvdata->rx_buffer_len);
        *(unsigned int *)arg = ccu_spi_drvdata->rx_buffer_len;
        break;
	default:
    	dev_err(&ccu_spi_drvdata->spi->dev, "[ccu_spi] Unknown IOCTL called\n");
		return -EINVAL;
    }

    return 0;
}

//TODO: is the poll function needed?
static unsigned int ccu_spi_poll(struct file *file, poll_table * wait)
{
    unsigned int mask = 0;
    
    //printk("[ccu_spi] poll called. Waiting...\n");
    poll_wait(file, &ccu_spi_drvdata->data_queue, wait);

    mask |= POLLIN | POLLRDNORM;
    return mask;
}

static int ccu_spi_open(struct inode *inode, struct file *file)   
{
    printk("[ccu_spi] opened\n");
    ccu_spi_drvdata->exit = 0;
    ccu_spi_drvdata->customers += 1;
    //TODO: set up gpio interrupt
    return 0;
}   
   
static int ccu_spi_release(struct inode *inode, struct file *file)   
{
    // TODO: Create a proper "I'm leaving now" message.
    //u8 power_off[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x33};

    printk("[ccu_spi] release\n");
    ccu_spi_drvdata->customers -= 1;
    if (ccu_spi_drvdata->customers <= 0) {
        ccu_spi_drvdata->exit = 1;
        ccu_spi_drvdata->customers = 0;
        //TODO: delete gpio interrupt
        //spi_write(ccu_spi_drvdata->spi, power_off, sizeof(power_off));
        printk("[ccu_spi] released\n");
    }
    return 0;   
}

static const struct file_operations ccu_spi_fops = {   
    .owner   = THIS_MODULE,
    .read    = ccu_spi_read,
    .write   = ccu_spi_write,
    .unlocked_ioctl   = ccu_spi_ioctl,
    .poll    = ccu_spi_poll,
    .open    = ccu_spi_open,
    .release = ccu_spi_release,
};   
   
static struct miscdevice ccu_spi_miscdev =   
{   
     .minor = MISC_DYNAMIC_MINOR,   
     .name  = "spiri_ccu",   
     .fops  = &ccu_spi_fops   
};   

#ifdef CONFIG_OF_GPIO
/*
 * Translate OpenFirmware node properties into platform_data.
 */
static int
ccu_spi_get_of_pdata(struct device *dev, struct ccu_spi_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int ret;
	
	ret = of_get_named_gpio(np, "rtr-gpio", 0);
	if (ret < 0)
		return ret;
	pdata->rtr_gpio = ret;

	ret = of_get_named_gpio(np, "cts-gpio", 0);
	if (ret < 0)
		return ret;
	pdata->cts_gpio = ret;



	//if (!gpio_is_valid(pdata->cts-gpio))
		//return -ENODEV;

	/*ret = gpio_request(pdata->cts-gpio, "cci-spi-cts-gpio");
	if (ret < 0) {
		dev_err(&client->dev, "request gpio failed: %d\n", ret);
		return ret;
	}

	// controller should be waken up, return irq.  
	gpio_direction_input(gpio);*/


	return 0;
}

static const struct of_device_id of_ccu_spi_match[] = {
	{ .compatible = "spiri,ccu_spi", },
	{},
};
#endif /* CONFIG_OF_GPIO */

static int ccu_spi_probe(struct spi_device *spi)
{
	struct ccu_spi_platform_data *pdata = dev_get_platdata(&spi->dev);
	int ret;
	
	printk("[ccu_spi] Probing...\n");

#ifdef CONFIG_OF_GPIO
	if (!pdata) {
		pdata = devm_kzalloc(&spi->dev,
				     sizeof(struct ccu_spi_platform_data), GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto err;
		}
			

		ret = ccu_spi_get_of_pdata(&spi->dev, pdata);
		if (ret)
			goto err;
	}
#else
	if (!pdata) {
		ret = -EINVAL;
		goto err;
	}
#endif /* CONFIG_OF_GPIO */

    // Allocate mem for driver data
	ccu_spi_drvdata = devm_kzalloc(&spi->dev,
	                        sizeof(struct ccu_spi_drvdata), GFP_KERNEL);
	if (ccu_spi_drvdata == NULL) {
		dev_err(&spi->dev, "Failed to allocate memory for driver data\n");
		ret = -ENOMEM;
		goto err;
	}
	ccu_spi_drvdata->rx_buffer_len = 0;
	ccu_spi_drvdata->tx_buffer_len = 0;
	ccu_spi_drvdata->exit = 0;
	ccu_spi_drvdata->customers = 0;

    // Request CTS gpio
	if (!gpio_is_valid(pdata->cts_gpio)) {
		ret = -ENODEV;
		goto err;
	}

    ret = devm_gpio_request_one(&spi->dev, pdata->cts_gpio, 
            GPIOF_DIR_IN, "cts-gpio");
	if (ret) {
		dev_err(&spi->dev, "[ccu_spi] Failed to setup CTS GPIO\n");
		goto err;
	}
    ccu_spi_drvdata->cts_gpio = pdata->cts_gpio;

    // Setup SPI and defered work queue
	spi->mode = SPI_MODE_0;
	ccu_spi_drvdata->spi = spi;
	
	INIT_WORK(&ccu_spi_drvdata->irq_work, read_from_uc);
	ccu_spi_drvdata->workqueue = create_singlethread_workqueue("spiri_ccu");
	if (ccu_spi_drvdata->workqueue == NULL) {
		dev_err(&ccu_spi_drvdata->spi->dev, 
				"[ccu_spi] Couldn't create workqueue\n");
		ret = -ENOMEM;
		goto err;
	}

    // Init data read queue
    init_waitqueue_head(&ccu_spi_drvdata->data_queue);

	// Setup interrupt
	/*ccu_spi_drvdata->nirq = OMAP_GPIO_IRQ(pdata->nirq_gpio);
	ret = request_irq(ccu_spi_drvdata->nirq, ccu_spi_nirq_interrupt, IRQF_TRIGGER_FALLING, 
								spi->dev.driver->name, ccu_spi_drvdata);
	if (ret) {
		dev_err(&spi->dev, "Failed to request irq\n");
		goto err2;
	}*/
	

    // TODO: Test for uC presence


    // Register device for user space interaction
	ret = misc_register(&ccu_spi_miscdev);   
	if (ret) {
		dev_err(&spi->dev, "Failed to register miscdev\n");
		goto err3;
	}
	
	dev_set_drvdata(&spi->dev, ccu_spi_drvdata);
	
	printk("[ccu_spi] Probe done\n");
	return 0;

err3:
	//free_irq(ccu_spi_drvdata->nirq, ccu_spi_drvdata);

//err2:
	cancel_work_sync(&ccu_spi_drvdata->irq_work);
	INIT_WORK(&ccu_spi_drvdata->irq_work, read_from_uc);
	destroy_workqueue(ccu_spi_drvdata->workqueue);
	ccu_spi_drvdata->workqueue = NULL;

err:
	/*if (pdata != NULL)
		gpio_free(pdata->shutdown_gpio);*/

	dev_err(&spi->dev, "[ccu_spi] Probe failed\n");
	return ret;
}

static int ccu_spi_remove(struct spi_device *spi)
{
	misc_deregister(&ccu_spi_miscdev);

	//free_irq(ccu_spi_drvdata->nirq, ccu_spi_drvdata);

	cancel_work_sync(&ccu_spi_drvdata->irq_work);
	INIT_WORK(&ccu_spi_drvdata->irq_work, read_from_uc);
	destroy_workqueue(ccu_spi_drvdata->workqueue);
	ccu_spi_drvdata->workqueue = NULL;

	/*if (pdata != NULL)
		gpio_free(pdata->shutdown_gpio);*/

	//kfree(ccu_spi_drvdata);

	return 0;
}

static struct spi_driver ccu_spi_driver = {
	.driver = {
		.name	= "spiri_ccu",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(of_ccu_spi_match),
	},
	.probe		= ccu_spi_probe,
	.remove		= ccu_spi_remove,
};

static __init int ccu_spi_init(void)
{
	return spi_register_driver(&ccu_spi_driver);
}

static __exit void ccu_spi_exit(void)
{
	spi_unregister_driver(&ccu_spi_driver);
}

module_init(ccu_spi_init);
module_exit(ccu_spi_exit);

MODULE_AUTHOR("Nikolaj Due Oesterbye <ndo@spiri.io>");
MODULE_DESCRIPTION("Spiri internal CCU SPI communication");
MODULE_LICENSE("GPL");

