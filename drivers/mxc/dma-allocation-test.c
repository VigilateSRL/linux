/*
 * Demo module for allocation of contiguous dmable memory
 *
 * Author Davide Ciminaghi <ciminaghi@gnudd.com> Sept 2017
 */
#define DEBUG
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <uapi/linux/dma_allocation_test.h>


#define DMA_ALLOC 1
#define DMA_FREE 2

struct dma_allocation {
	dma_addr_t addr;
	void *vaddr;
	size_t sz;
	struct list_head list;
};

struct dma_allocation_test_data {
	struct platform_device *pdev;
	struct miscdevice md;
	struct list_head allocations;
};

#define to_dma_allocation_test_data(ptr) \
	container_of(ptr, struct dma_allocation_test_data, md);

static struct dma_allocation *
find_allocation(struct dma_allocation_test_data *p, dma_addr_t addr)
{
	struct dma_allocation *a;

	list_for_each_entry(a, &p->allocations, list) {
		if (a->addr == addr)
			return a;
	}
	return a;
}

static void free_allocation(struct device *dev, struct dma_allocation *a)
{
	dma_free_coherent(dev, a->sz, a->vaddr, a->addr);
	list_del(&a->list);
	devm_kfree(dev, a);
}

static void free_allocations(struct device *dev)
{
	struct dma_allocation_test_data *data = dev->driver_data;
	struct dma_allocation *tmp, *ptr;

	if (!data) {
		WARN_ON(1);
		return;
	}
	list_for_each_entry_safe(ptr, tmp, &data->allocations, list)
		free_allocation(dev, ptr);
}

static int dma_allocation_test_open(struct inode *inode, struct file *file)
{
	return 0;
}

static long dma_allocation_test_ioctl(struct file *file, unsigned int cmd,
				      unsigned long arg)
{
	struct dma_allocation_test_data *data;
	int ret = 0;
	struct miscdevice *md = file->private_data;
	struct platform_device *pdev;

	dev_dbg(md->this_device, "dev = %p\n", md->this_device);
	data = to_dma_allocation_test_data(md);
	if (!data) {
		WARN_ON(1);
		return -EINVAL;
	}
	pdev = data->pdev;
	if (!pdev) {
		WARN_ON(1);
		return -EINVAL;
	}
	dev_dbg(md->this_device, "alloc data = %p\n", data);
	switch (cmd) {
	case DMA_ALLOC_TEST_ALLOC:
	{
		size_t sz;
		struct dma_allocation *a;

		get_user(sz, (__u32 __user *)arg);
		dev_dbg(&pdev->dev, "Allocating %zu bytes\n", sz);
		a = devm_kzalloc(&pdev->dev, sizeof(*a), GFP_KERNEL);
		if (!a)
			return -ENOMEM;
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
		a->vaddr = dma_alloc_coherent(&pdev->dev, sz, &a->addr,
					      GFP_KERNEL);
		a->sz = sz;
		if (!a->vaddr) {
			devm_kfree(&pdev->dev, a);
			return -ENOMEM;
		}
		list_add(&a->list, &data->allocations);
		put_user(a->addr, (__u32 __user *)arg);
		break;
	}
	case DMA_ALLOC_TEST_FREE:
	{
		struct dma_allocation *a =
			find_allocation(data, (dma_addr_t)arg);

		dev_dbg(&pdev->dev, "Freeing @%x\n", (__u32)arg);
		if (!a) {
			dev_err(&pdev->dev, "No allocation found @%xu\n",
				(__u32)arg);
			return -EINVAL;
		}
		free_allocation(&pdev->dev, a);
		break;
	}
	default:
		return -EINVAL;
	}
	return ret;
}

const struct file_operations dma_allocation_test_fops = {
	.owner		= THIS_MODULE,
	.open = dma_allocation_test_open,
	.unlocked_ioctl	= dma_allocation_test_ioctl,
};

static int dma_allocation_test_probe(struct platform_device *pdev)
{
	int ret;
	struct dma_allocation_test_data *data;

	dev_dbg(&pdev->dev, "probe entered\n");
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	INIT_LIST_HEAD(&data->allocations);
	data->md.minor = MISC_DYNAMIC_MINOR;
	data->md.name = "dma-allocation-test";
	data->md.fops = &dma_allocation_test_fops;
	data->pdev = pdev;
	dev_dbg(&pdev->dev, "dev = %p\n", &pdev->dev);
	dev_dbg(&pdev->dev, "allocation data = %p\n", data);
	platform_set_drvdata(pdev, data);
	dev_dbg(&pdev->dev, "allocation data 2 = %p\n",
		platform_get_drvdata(pdev));
	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "error in of_reserved_mem_device_init\n");
		return ret;
	}
	ret = misc_register(&data->md);
	if (ret) {
		dev_err(&pdev->dev,
			"error registering dma allocation test misc\n");
		return ret;
	}
	dev_set_drvdata(&pdev->dev, data);
	dev_dbg(&pdev->dev, "allocation data 3 = %p\n",
		platform_get_drvdata(pdev));
	return 0;
}

static int dma_allocation_test_remove(struct platform_device *pdev)
{
	struct dma_allocation_test_data *data = dev_get_drvdata(&pdev->dev);

	if (!data) {
		WARN_ON(1);
		return -ENODEV;
	}
	misc_deregister(&data->md);
	free_allocations(&pdev->dev);
	return 0;
}

static const struct of_device_id dma_allocation_test_match[] = {
	{ .compatible = "dma-allocation-test", },
	{},
};
MODULE_DEVICE_TABLE(of, dma_allocation_test_match);


static struct platform_driver dma_allocation_test_driver = {
	.probe		= dma_allocation_test_probe,
	.remove		= dma_allocation_test_remove,
	.driver		= {
		.name	= "dma-allocation-test",
		.of_match_table = of_match_ptr(dma_allocation_test_match),
	},
};

static int __init dma_allocation_test_init(void)
{
	return platform_driver_register(&dma_allocation_test_driver);
}

static void __exit dma_allocation_test_cleanup(void)
{
	platform_driver_unregister(&dma_allocation_test_driver);
}

module_init(dma_allocation_test_init);
module_exit(dma_allocation_test_cleanup);

MODULE_LICENSE("GPL v2");
