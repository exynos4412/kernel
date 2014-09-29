#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kfifo.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/version.h> // for "LINUX_VERSION_CODE"
#include <linux/poll.h>
#include "char.h"

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
#define CHRDEV_NUM    4
#define CHRDEV_CLASS_ATTR 1
#define FIFO_SIZE    64
#define SUPPORT_CAT  1
//#undef  SUPPORT_CAT //if using standard read/write operations, open it

static loff_t chr_dev_llseek(struct file *file, loff_t offset, int arg)
{
	chr_dbg("%s offset:%d not support!\n", __func__, offset);
	return -EINVAL;
}

static ssize_t chr_dev_read(struct file *file, char __user * buf, size_t cnt, loff_t *oft)
{
	int err;
	size_t fifo_len;
	unsigned int copied;
#ifdef SUPPORT_CAT
	size_t rlen = 0;
#endif
	struct chr_dev *chrdev = file->private_data;
	DEFINE_WAIT(wait);  //autoremove wake(current process)
	while (1) {
		mutex_lock(&chrdev->lock);
		//1.add to waite queue  2.set current state:TASK_INTERRUPTIBLE
		prepare_to_wait(&chrdev->r_wq, &wait, TASK_INTERRUPTIBLE);
		err = (kfifo_is_empty(&chrdev->fifo));
		mutex_unlock(&chrdev->lock);
		if (!err)
			break;

		if (file->f_flags & O_NONBLOCK) { //open with flags:O_NONBLOCK
			err = -EAGAIN;
			break;
		}

		if (signal_pending(current)) { //ctrl+C, signal interrupt
			err = -EINTR;
			break;
		}

		schedule();
	}
	finish_wait(&chrdev->r_wq, &wait);
	if (err)
		return err;

	if (kfifo_is_empty(&chrdev->fifo) || (cnt > FIFO_SIZE)){
		printk("Chr dev read error!cnt=%d\n", cnt);
#ifdef SUPPORT_CAT
		rlen = cnt;
		cnt  = FIFO_SIZE;
#else
		return -ENOMEM;
#endif
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
	wake_up_interruptible(&chrdev->w_wq); //wakeup write waitqueue
	mutex_unlock(&chrdev->lock);
	chr_dbg("%s read size:%d\n", __func__, copied);
#ifdef SUPPORT_CAT
	return rlen;
#else
	return copied;
#endif
}

static ssize_t chr_dev_write(struct file *file, const char __user *buf, size_t cnt, loff_t *oft)
{
	int err;
	size_t fifo_len;
	unsigned int copied;
	struct chr_dev *chrdev = file->private_data;

	DEFINE_WAIT(wait);  //autoremove wake(current process)
	while (1) {
		mutex_lock(&chrdev->lock);
		//1.add to waite queue  2.set current state:TASK_INTERRUPTIBLE
		prepare_to_wait(&chrdev->w_wq, &wait, TASK_INTERRUPTIBLE);
		err = (kfifo_is_full(&chrdev->fifo));
		mutex_unlock(&chrdev->lock);
		if (!err)
			break;

		if (file->f_flags & O_NONBLOCK) { //open with flags:O_NONBLOCK
			err = -EAGAIN;
			break;
		}

		if (signal_pending(current)) { //ctrl+C, signal interrupt
			err = -EINTR;
			break;
		}

		schedule(); //switch to other process
	}
	finish_wait(&chrdev->w_wq, &wait); //remove wait queue form wait
	if (err)
		return err;

	if (kfifo_is_full(&chrdev->fifo) || (cnt > FIFO_SIZE)){
		printk("Chr dev write error! cnt=%d\n", cnt);
		return -ENOMEM;
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
	wake_up_interruptible(&chrdev->r_wq); //wakeup read waitqueue
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
	int ret = 0;
	int nums;
	struct chr_dev *chrdev = file->private_data;
	mutex_lock(&chrdev->lock);
	switch(cmd){
		case CHRDEV_GET_SMEM_SIZE:
			ret = copy_to_user((void __user *)arg, &chrdev->smsize, sizeof(chrdev->smsize));
			break;
		case CHRDEV_SET_SMEM_LEN:
			ret = copy_from_user(&chrdev->smsize, (void __user *)arg, sizeof(chrdev->smsize));
			if(chrdev->shre_mem)
				kfree(chrdev->shre_mem);
			chrdev->shre_mem = kzalloc(chrdev->smsize, GFP_KERNEL);
			if(!chrdev->shre_mem){
				printk("Kernel set share mem error!\n");
				ret = -ENOMEM;
			}
			break;		
		case CHRDEV_GET_SMEM_OFS:
			ret = copy_to_user((void __user *)arg, &chrdev->offset, sizeof(chrdev->ndev));
			break;
		case CHRDEV_SET_SMEM_OFS:
			ret = copy_from_user(&chrdev->offset, (void __user *)arg, sizeof(chrdev->offset));
			break;
		case CHRDEV_CLEAR_SMEM:
			if(chrdev->shre_mem)
				memset(chrdev->shre_mem, 0, chrdev->smsize);
			break;
		case CHRDEV_SET_SMEM_NUMS: //write nums
			ret = copy_from_user(&nums, (void __user *)arg, sizeof(nums));
			if((nums + chrdev->offset) > chrdev->smsize){ //offset 1-> sizes
				ret = -EINVAL;
				chrdev->usernums = 0;
			}else
				chrdev->usernums = nums;
			break;
		case CHRDEV_GET_SMEM_NUMS: //read nums
			ret = copy_from_user(&nums, (void __user *)arg, sizeof(nums));
			if(nums > chrdev->offset){
				ret = -EINVAL;
				chrdev->usernums = 0;
			}else
				chrdev->usernums = nums;
			break;
		case CHRDEV_GET_SMEMVALS:
			if(chrdev->shre_mem){
				if((chrdev->usernums == 0) && (chrdev->usernums > chrdev->offset))
					ret = -EINVAL;
				else{
					ret = copy_to_user((void __user *)arg, &chrdev->shre_mem[chrdev->offset - chrdev->usernums], chrdev->usernums);
					chrdev->offset -= chrdev->usernums;
				}
			}else
				ret = -ENOMEM;
			break;
		case CHRDEV_SET_SMEMVALS:
			if(chrdev->shre_mem){
				if((chrdev->usernums == 0) && ((chrdev->usernums + chrdev->offset) > chrdev->smsize))
					ret = -EINVAL;
				else{
					ret = copy_from_user(&chrdev->shre_mem[chrdev->offset], (void __user *)arg, chrdev->usernums);
					chrdev->offset += chrdev->usernums;
				}
			}else
				ret = -ENOMEM;
			break;
		default:break;
	}
	mutex_unlock(&chrdev->lock);
	return ret;
}

static int chr_dev_mmap(struct file *file, struct vm_area_struct *vm)
{
	//see drivers/staging/android/binder.c
	//binder_mmap
	return 0;
}

static int chr_dev_open(struct inode *id, struct file *file)
{
	int err;
	struct chr_dev *chrdev = container_of(id->i_cdev, struct chr_dev, dev);
	file->private_data = chrdev;
	//nonlessk, fops->llseek not support
	file->f_mode &= ~FMODE_LSEEK;
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
		if(!chr_dev->devinfo[idx]){
			n += sprintf(data+n, "Device:%d is removed\n", idx);
			continue;
		}
		n += sprintf(data+n, "%s %u:%u\n",
			dev_name(chr_dev->devinfo[idx]->dev),
			chr_dev->major, chr_dev->minor + idx);
	}
	mutex_unlock(&chr_dev->lock);
	return n;
}

static ssize_t class_chrdev_store_remove(struct class *c,struct class_attribute *attr,const char *buf,size_t count)
{
	int devn;
	unsigned int major, minor;
	struct chr_dev *chrdev = gchr_dev;
	devn = simple_strtoul(buf, NULL, 16);
	if((devn > chrdev->ndev) || (devn < 1)){
		printk("Input arg:%d is too big or too small, max:%d!\n", devn, chrdev->ndev);
		return -EINVAL;
	}
	if(chrdev->devinfo[devn - 1] && !atomic_read(&chrdev->user)){
		major = chrdev->major;
		minor = chrdev->minor + devn -1;
		chr_dbg("Remove devices:%s %d:%d\n", dev_name(chrdev->devinfo[devn - 1]->dev), major, minor);
		device_destroy(chrdev_class, MKDEV(major, minor));
		free_percpu(chrdev->devinfo[devn - 1]->priv);
		kfree(chrdev->devinfo[devn - 1]);
		unregister_chrdev_region(MKDEV(major, minor), 1);
		chrdev->devinfo[devn - 1] = NULL;
	}else
		printk("Warning device:%d is removed or in using!\n", devn - 1);
	return count;
}

static ssize_t kfifo_show(struct class *c, struct class_attribute *attr, char *data)
{
	int i, val, n = 0;
	size_t fifo_len;
	struct chr_dev *chrdev = gchr_dev;
	
	if(!atomic_read(&chrdev->user))
		return sprintf(data, "No Users, No Kfifo!\n");
	if (kfifo_is_empty(&chrdev->fifo))
		return sprintf(data, "Empty Kfifo!\n");
	mutex_lock(&chrdev->lock);
	fifo_len = kfifo_len(&chrdev->fifo); //number of used elements in the fifo
	i = fifo_len;
	while(i){
		val = kfifo_peek(&chrdev->fifo, &i);
		n += sprintf(data + n, "kfifo[i]=%d\n", val);
		i--;
	}
	mutex_unlock(&chrdev->lock);
	return n;
}

static ssize_t kfifo_store(struct class *c,struct class_attribute *attr,const char *buf,size_t count)
{
	int err;
	size_t fifo_len;
	struct chr_dev *chrdev = gchr_dev;

	if(!atomic_read(&chrdev->user)){
		printk("No Users, No Kfifo!\n");
		return -EINVAL;
	}
	if (kfifo_is_full(&chrdev->fifo)){
		printk("Fifo is full!\n");
		return -EINVAL;
	}
	mutex_lock(&chrdev->lock);
	fifo_len = kfifo_avail(&chrdev->fifo); //number of unused elements in the fifo
	err = kfifo_in(&chrdev->fifo, buf, min(count, fifo_len));
	if(err < 0){
		printk("kfifo_in error\n");
		mutex_unlock(&chrdev->lock);
		return err;
	}
	mutex_unlock(&chrdev->lock);
	return count;
}

static struct class_attribute class_chrdev_attrs[] = {
 __ATTR(remove,         0200, NULL, class_chrdev_store_remove),
 __ATTR(device_map,     0444, class_chrdev_show_map, NULL),
  __ATTR(kfifo,         0755, kfifo_show, kfifo_store),
 //__ATTR_RW(kfifo),
 __ATTR_NULL
};

static ssize_t chrdev_num_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int minor;
	struct dev_info *info;
	info = dev_get_drvdata(dev);
	minor = MINOR(info->devn);
	return sprintf(buf,"minior:%d\n", minor);
}

static ssize_t chrdev_num_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned int val;
	val = simple_strtoul(buf, NULL, 16);
	chr_dbg("Test for scanf:0x%x\n", val);
	return count;
}

static ssize_t chrdev_priv_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int i, n = 0;	
	int *ptr;
	struct dev_info *info;
	info = dev_get_drvdata(dev);
	for_each_possible_cpu(i) { 
		ptr = per_cpu_ptr(info->priv, i);
		n += sprintf(buf + n, "%s cpu[%u]:%u\n",
			dev_name(info->dev),
			i, *ptr);
	}
	return n;
}

static ssize_t chrdev_priv_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int i, val[2]; //only for args:cpu,val
	const char *p;
	int *ptr;
	struct dev_info *info;
	info = dev_get_drvdata(dev);
	p = buf;
	i = 0;
	while(*p++){
		if((*p == '0') && ((*(p + 1) == 'x') || (*(p + 1) == 'X')))
			continue;
		switch(*p){
			case '0' ... '9': 
				if(i > 1)
					break;
				val[i] = simple_strtoul(p, NULL, 10);
				i++;
				break;
			default:
				break;
		}
	}
	if(val[0] > (nr_cpu_ids - 1)){ //id 0 - 3,num=4
		printk("ERROR! cpus max id is %d\n", nr_cpu_ids - 1);
		return -EINVAL;
	}
	ptr = per_cpu_ptr(info->priv, val[0]);
	*ptr = val[1];
	chr_dbg("Set cpu[%d]:0x%x\n", val[0], val[1]);
	return count;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
static struct device_attribute class_chrdev_dev_attrs[] = {
__ATTR(num, 755, chrdev_num_show, chrdev_num_store),
__ATTR(private, 755, chrdev_priv_show, chrdev_priv_store),
__ATTR_NULL
};
#else
DEVICE_ATTR(num, 755, chrdev_num_show, chrdev_num_store);
DEVICE_ATTR(private, 755, chrdev_priv_show, chrdev_priv_store);
static struct attribute * chrdev_attrs[] = {
	&dev_attr_num.attr,
	&dev_attr_private.attr,
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
	int ret, i, n, j;
	struct chr_dev *chr_dev;
	dev_t dev;
	//malloc struct data mem if necessary
	chr_dev = kzalloc(sizeof(struct chr_dev) + sizeof(struct dev_info *) * CHRDEV_NUM, GFP_KERNEL);
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
	chr_dev->ncurdev = chr_dev->ndev;
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
		chr_dev->devinfo[n] = kzalloc(sizeof(struct dev_info), GFP_KERNEL);
		if(!chr_dev->devinfo[n]){
			printk("##ERROR! Kzalloc device mem!\n");
			goto err;
		}
		chr_dev->devinfo[n]->priv = alloc_percpu(int); //for each cpu alloc val for sync study
		for_each_possible_cpu(j) { //init for each cpu val
			int *ptr;
			ptr = per_cpu_ptr(chr_dev->devinfo[n]->priv, j); //get each cpu val
			*ptr = 100 + j*100; //charge the val
		}
		//set devinfo to dev->p->drvdata
		chr_dev->devinfo[n]->devn = MKDEV(chr_dev->major, i);
		chr_dev->devinfo[n]->dev  = device_create(chrdev_class, NULL, MKDEV(chr_dev->major, i), chr_dev->devinfo[n], "chr_dev%d", i);
		n++;
	}	
	mutex_init(&chr_dev->lock); //init mutex lock for share_mem,kfifo
	init_waitqueue_head(&chr_dev->r_wq); //init waitqueue for all devices
	init_waitqueue_head(&chr_dev->w_wq);
	gchr_dev = chr_dev;
	atomic_set(&chr_dev->user, 0);
	return 0;
err:
	while(n)
		kfree(chr_dev->devinfo[--n]);
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
	int i, n;
	struct chr_dev *chr_dev = gchr_dev;
	for(i = chr_dev->minor, n = 0; i < chr_dev->ndev; i++){
		if(!chr_dev->devinfo[n]){
			n++;
			continue;
		}
		device_destroy(chrdev_class, MKDEV(chr_dev->major, i));	
		free_percpu(chr_dev->devinfo[n]->priv);
		kfree(chr_dev->devinfo[n]);
		n++;
	}
	class_destroy(chrdev_class);
	unregister_chrdev_region(MKDEV(chr_dev->major, chr_dev->minor), chr_dev->ndev);
	kfree(chr_dev->shre_mem);
	kfree(chr_dev);
}

module_init(chr_dev_init);
module_exit(chr_dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("jf.s");
