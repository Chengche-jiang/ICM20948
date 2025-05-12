// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Invensense, Inc.
 */

#include "inv_icm20948.h"

/* 空的调试文件实现，用于满足CONFIG_INV_ICM20948_DEBUG条件下的编译要求 */

#ifdef CONFIG_INV_ICM20948_DEBUG

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>

/* 调试文件初始化函数 */
int inv_icm20948_debug_init(struct inv_icm20948_state *st)
{
    /* 构建空实现，实际未创建任何debugfs项 */
    dev_info(regmap_get_device(st->map), "Debug support initialized (placeholder)\n");
    return 0;
}

/* 调试文件清理函数 */
void inv_icm20948_debug_cleanup(struct inv_icm20948_state *st)
{
    /* 空实现 */
}

#endif /* CONFIG_INV_ICM20948_DEBUG */