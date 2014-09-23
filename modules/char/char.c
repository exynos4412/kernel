#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kfifo.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/version.h> // for "LINUX_VERSION_CODE"
#include <linux/poll.h>

struct chr_dev{
	int major;  //device major number
	int minor;  //base minor
	struct cdev dev;    //all device,s type:chr dev
	atomic_t user;
	struct kfifo fifo;  //share mem for all devices
	struct mutex lock;  //devices access mem sync
	struct class *cls;  //all device's class
	wait_queue_head_t  w_wq; //write wait queue head
	wait_queue_head_t  r_wq; //read  wait queue head
	int ndev;   //array sizes and num of devices
	struct device *device[]; //0 size array don't hava mem, size=0. must at the end of struct data
};
static struct chr_dev *gchr_dev;
static struct class *chrdev_class;
#define MDDEBUG
#ifdef MDDEBUG
#define chr_dbg(argc, argv ...)	 printk("########"argc, ##argv)
#else
#define chr_dbg(argc, argv ...)
#endif
#define DEV_NAME    "chr_dev"
#define CHRDEV_MAJOR   0  //0:alloc auto  others:static define
#define CHRDEV_MINOR  0
#define CHRDEV_NUM    1
#define CHRDEV_CLASS_ATTR 1
#define FIFO_SIZE    64

static loff_t chr_dev_llseek(struct file *file, loff_t offset, int arg)
{
	loff_t ret;
	return ret;
}
static ssize_t chr_dev_read(struct file *file, char __user * buf, size_t cnt, loff_t *oft)
{
	int err;
	size_t fifo_len;
	unsigned int copied;
	struct chr_dev *chrdev = file->private_data;
	if (kfifo_is_empty(&chrdev->fifo) || (cnt > FIFO_SIZE)){
		printk("Chr dev read error! empty or cnt > fifo_size");
		return 0;
	}
	mutex_lock(&chrdev->lock);
	fifo_len = kfifo_len(&chrdev->fifo); //number of used elements in the fifo
	err = kfifo_to_user(&chrdev->fifo, buf, min(cnt, fifo_len), &copied);
	if(err < 0){
		printk("kfifo_to_user error\n");
		mutex_unlock(&chrdev->lock);
		return err;
	}
	*oft += copied; //update files_ops
	mutex_unlock(&chrdev->lock);
	chr_dbg("%s read size:%d\n", __func__, copied);
	return copied;
}
static ssize_t chr_dev_write(struct file *file, const char __user *buf, size_t cnt, loff_t *oft)
{
	int err;
	size_t fifo_len;
	unsigned int copied;
	struct chr_dev *chrdev = file->private_data;
	if (kfifo_is_empty(&chrdev->fifo) || (cnt > FIFO_SIZE)){
		printk("Chr dev read error! full or cnt > fifo_size");
		return 0;
	}
	mutex_lock(&chrdev->lock);
	fifo_len = kfifo_avail(&chrdev->fifo); //number of unused elements in the fifo
	err = kfifo_from_user(&chrdev->fifo, buf, min(cnt, fifo_len), &copied);
	if(err < 0){
		printk("kfifo_from_user error\n");
		mutex_unlock(&chrdev->lock);
		return err;
	}
	*oft += copied;
	mutex_unlock(&chrdev->lock);
	chr_dbg("%s write size:%d\n", __func__, copied);
	return copied;
}
static unsigned int chr_dev_poll(struct file *file, struct poll_table_struct *pt)
{
	unsigned int mask = 0;
	struct chr_dev *chrdev = file->private_data;
	mutex_lock(&chrdev->lock);
	poll_wait(file, &chrdev->w_wq, pt);
	poll_wait(file, &chrdev->r_wq, pt);
	if(!kfifo_is_full(&chrdev->fifo))
		mask |= POLLOUT | POLLWRNORM; //write
	if(!kfifo_is_empty(&chrdev->fifo))
		mask |= POLLIN  | POLLRDNORM; //read
	mutex_unlock(&chrdev->lock);	
	chr_dbg("%s mask:0x%x\n", __func__, mask);
	return mask;
}
static long chr_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}
static int chr_dev_mmap(struct file *file, struct vm_area_struct *vm)
{
	return 0;
}
static int chr_dev_open(struct inode *id, struct file *file)
{
	int err;
	struct chr_dev *chrdev = container_of(id->i_cdev, struct chr_dev, dev);
	file->private_data = chrdev;
	if(!atomic_read(&chrdev->user)){ //first open alloc fifo
		err = kfifo_alloc(&chrdev->fifo, FIFO_SIZE, GFP_KERNEL);
		if(err){
			printk("fifo open error!\n");
			return err;
		}
	}
	atomic_inc(&chrdev->user);
	chr_dbg("%s users:%d\n", __func__, atomic_read(&chrdev->user));
	return 0;
}
static int chr_dev_release(struct inode *id, struct file *file)
{
	struct chr_dev *chrdev = file->private_data;
	if(atomic_dec_and_test(&chrdev->user))
		kfifo_free(&chrdev->fifo);
	chr_dbg("%s users:%d\n", __func__, atomic_read(&chrdev->user));
	return 0;
}

static const struct file_operations chr_dev_fops = {
	.read		= chr_dev_read,
	.write		= chr_dev_write,
	.open		= chr_dev_open,
	.poll		= chr_dev_poll,
	.release	= chr_dev_release,
	.mmap		= chr_dev_mmap,
	.unlocked_ioctl = chr_dev_ioctl,
	.llseek		= chr_dev_llseek,
	.owner		= THIS_MODULE,
};

static ssize_t class_chrdev_show_map(struct class *c, struct class_attribute *attr, char *data)
{
	int n = 0;
	int idx;
	struct chr_dev *chr_dev = gchr_dev;
	mutex_lock(&chr_dev->lock);
	for (idx = 0; idx < chr_dev->ndev; idx++) {
		n += sprintf(data+n, "%s %u:%u\n",
			dev_name(chr_dev->device[idx]),
			MAJOR(chr_dev->major), MINOR(chr_dev->minor));
	}
	mutex_unlock(&chr_dev->lock);
	return n;
}

static ssize_t class_chrdev_store_remove(struct class *c,struct class_attribute *attr,const char *buf,size_t count)
{
	unsigned int major, minor;
#if 0
	if (sscanf(buf, "%u:%u", &major, &minor) == 2) {
		pkt_remove_dev(MKDEV(major, minor));
		return count;
	}
#endif
	return -EINVAL;
}
static struct class_attribute class_chrdev_attrs[] = {
 __ATTR(remove,         0200, NULL, class_chrdev_store_remove),
 __ATTR(device_map,     0444, class_chrdev_show_map, NULL),
 __ATTR_NULL
};

static ssize_t chrdev_num_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return 0;
}

static ssize_t chrdev_num_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	return count;
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 8)
static struct device_attribute class_chrdev_dev_attrs[] = {
__ATTR(num, 755, chrdev_num_show, chrdev_num_store),
__ATTR_NULL
};
#else
DEVICE_ATTR(num, 755, chrdev_num_show, chrdev_num_store);
static struct attribute * chrdev_attrs[] = {
	&dev_attr_num.attr,
	NULL,
};

static struct attribute_group chrdev_attr_group = {
	.attrs = chrdev_attrs,
};

static const struct attribute_group *chrdev_dev_groups[] = {
	&chrdev_attr_group,
	NULL,
};
#endif

static void class_chrdev_release(struct class *cls)
{
	kfree(cls);
}

static int __init chr_dev_init(void)
{
	int ret, i, n;
	struct chr_dev *chr_dev;
	dev_t dev;
	//malloc struct data mem if necessary
	chr_dev = kzalloc(sizeof(chr_dev) + sizeof(struct device *) * CHRDEV_NUM, GFP_KERNEL);
	if(!chr_dev){
		printk("Chr_dev malloc memory error!\n");
		return -ENOMEM;
	}
	//init cdev, bind file_operations. cdev->ops = fops
	cdev_init(&chr_dev->dev, &chr_dev_fops);
	chr_dev->dev.owner = THIS_MODULE;
	//chr_dev devices num
	chr_dev->major = CHRDEV_MAJOR;
	chr_dev->minor = CHRDEV_MINOR;
	chr_dev->ndev  = CHRDEV_NUM;
	if(chr_dev->major){
		//major device num  minor device num
		dev = MKDEV(chr_dev->major, chr_dev->minor);
		ret = register_chrdev_region(dev, chr_dev->ndev, DEV_NAME); //only one device num
	}else{
		ret = alloc_chrdev_region(&dev, chr_dev->minor, chr_dev->ndev, DEV_NAME);
		chr_dev->major = MAJOR(dev);
	}
	if(ret < 0){
		printk("Chr_dev can not get major!\n");
		goto devnum_err;
	}
	//add cdev to maps
	cdev_add(&chr_dev->dev, dev, chr_dev->ndev);
	//register class,1.null attrs  2.create dev/class attrs
	chr_dev->cls = chrdev_class;
#ifndef CHRDEV_CLASS_ATTR
	chrdev_class = class_create(THIS_MODULE, DEV_NAME);//class register, null dev/class attrs
#else
	chrdev_class = 	kzalloc(sizeof(*chrdev_class), GFP_KERNEL);
	if (!chrdev_class){
		goto class_err;
	}else{
		chrdev_class->name 			= DEV_NAME;  //must add
		chrdev_class->owner 		= THIS_MODULE; //must add
		chrdev_class->class_release = class_chrdev_release; //must add
		chrdev_class->class_attrs 	= class_chrdev_attrs;
		#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
		chrdev_class->dev_attrs		= class_chrdev_dev_attrs;
		#else
		chrdev_class->dev_groups    = chrdev_dev_groups;
		#endif
		ret = class_register(chrdev_class);
		if(ret){
			printk("Chr_dev can not register class!\n");
			goto class_rgerr;
		}
	}
#endif
	for(i = chr_dev->minor, n = 0; i < chr_dev->ndev; i++){
		//set dev minor to dev->p->drvdata
		chr_dev->device[n] = device_create(chrdev_class, NULL, MKDEV(chr_dev->major, i), &i, "chr_dev%d", i);
		n++;
	}	
	mutex_init(&chr_dev->lock);
	gchr_dev = chr_dev;
	atomic_set(&chr_dev->user, 0);
	return 0;
class_rgerr:
	kfree(chrdev_class);
	chrdev_class = NULL;
class_err:
	unregister_chrdev_region(dev, chr_dev->ndev);
devnum_err:
	kfree(chr_dev);
	return -ENODEV;

}

static void __exit chr_dev_exit(void)
{
	int i;
	struct chr_dev *chr_dev = gchr_dev;
	for(i = chr_dev->minor; i < chr_dev->ndev; i++)
		device_destroy(chrdev_class, MKDEV(chr_dev->major, i));
	class_destroy(chrdev_class);
	unregister_chrdev_region(MKDEV(chr_dev->major, chr_dev->minor), chr_dev->ndev);
	kfree(chr_dev);
}

module_init(chr_dev_init);
module_exit(chr_dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("jf.s");
