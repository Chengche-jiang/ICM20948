#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

MODULE_INFO(intree, "Y");

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xbc24861, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0x9b9df045, __VMLINUX_SYMBOL_STR(iio_triggered_buffer_cleanup) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0xcf6e01ee, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0xea09ae58, __VMLINUX_SYMBOL_STR(iio_trigger_notify_done) },
	{ 0xf7698149, __VMLINUX_SYMBOL_STR(regmap_update_bits_base) },
	{ 0x5a906313, __VMLINUX_SYMBOL_STR(regmap_bulk_read) },
	{ 0x44b1d426, __VMLINUX_SYMBOL_STR(__dynamic_pr_debug) },
	{ 0xa4cabf58, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x67664fc8, __VMLINUX_SYMBOL_STR(regmap_read) },
	{ 0x3dd0cf5b, __VMLINUX_SYMBOL_STR(iio_get_time_ns) },
	{ 0x98fbfb02, __VMLINUX_SYMBOL_STR(devm_iio_trigger_register) },
	{ 0x8a90937a, __VMLINUX_SYMBOL_STR(devm_iio_trigger_alloc) },
	{ 0xcb7fa4aa, __VMLINUX_SYMBOL_STR(devm_iio_device_register) },
	{ 0x823629ed, __VMLINUX_SYMBOL_STR(devm_iio_device_alloc) },
	{ 0x5f5d53fc, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x9bc5022b, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x86216d6e, __VMLINUX_SYMBOL_STR(iio_trigger_poll) },
	{ 0xa23e9931, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0xebf7fe1e, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0x346b4705, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0x12a38747, __VMLINUX_SYMBOL_STR(usleep_range) },
	{ 0xb2cfc5ca, __VMLINUX_SYMBOL_STR(__devm_regmap_init_i2c) },
	{ 0x2ceef042, __VMLINUX_SYMBOL_STR(__dynamic_dev_dbg) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x835292b9, __VMLINUX_SYMBOL_STR(iio_push_to_buffers) },
	{ 0x78b0b25e, __VMLINUX_SYMBOL_STR(regmap_get_device) },
	{ 0x2e0d2f7f, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0x59b0504a, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0x8ad97e3e, __VMLINUX_SYMBOL_STR(devm_request_threaded_irq) },
	{ 0x4994d38a, __VMLINUX_SYMBOL_STR(regmap_write) },
	{ 0xc9c3e79c, __VMLINUX_SYMBOL_STR(iio_triggered_buffer_setup) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("i2c:icm20948");
MODULE_ALIAS("of:N*T*Cinvensense,icm20948");
MODULE_ALIAS("of:N*T*Cinvensense,icm20948C*");
