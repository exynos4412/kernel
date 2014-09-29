#ifndef __CHAR_H__
#define __CHAR_H__
#include <linux/ioctl.h>

struct dev_info{ //device info
	dev_t devn;
	int __percpu *priv;
	struct device *dev;
};
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
	char *shre_mem;   //share mem for process
	int offset;  //share mem offset
	int usernums; //user wants to get numbers
	int smsize;  //share mem size
	int ncurdev; //cur device numbers
	int ndev;   //array sizes and num of devices
	struct dev_info *devinfo[]; //0 size array don't hava mem, size=0. must at the end of struct data
};

//ioctl magic
#define __CHRDEVIO	0xAE

#define CHRDEV_GET_SMEM_SIZE	_IO(__CHRDEVIO, 1) /* uses smem len */
#define CHRDEV_SET_SMEM_LEN		_IO(__CHRDEVIO, 2) /* size of mem */
#define CHRDEV_GET_SMEM_OFS 	_IO(__CHRDEVIO, 3) /* devices numbers */
#define CHRDEV_SET_SMEM_OFS		_IO(__CHRDEVIO, 4) /* devices numbers */
#define CHRDEV_CLEAR_SMEM		_IO(__CHRDEVIO, 5) /* clear share mem */
#define CHRDEV_SET_SMEM_NUMS	_IO(__CHRDEVIO, 6) /* write share mem nums */
#define CHRDEV_GET_SMEM_NUMS	_IO(__CHRDEVIO, 7) /* read share mem nums */
#define CHRDEV_GET_SMEMVALS		_IO(__CHRDEVIO, 8) /* get share mem vals*/
#define CHRDEV_SET_SMEMVALS		_IO(__CHRDEVIO, 9) /* set share mem vals */
#endif
