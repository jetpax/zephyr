/*
 * Copyright (c) 2026 Jonathan E. Peace <jep@alphabetiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WO-ORANGE-1 P3 exit gate: mount the SD card's FAT volume, read an
 * asset file, stage a freshly built llext module onto the card, then
 * load and run it from the filesystem.
 */

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/llext/llext.h>
#include <zephyr/llext/fs_loader.h>
#include <ff.h>

static const uint8_t hello_ext_buf[] = {
#include "hello_ext.inc"
};

static FATFS fat_fs;

static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = "/SD:",
};

static int read_asset(const char *path)
{
	struct fs_file_t file;
	char buf[128];
	ssize_t n;
	int ret;

	fs_file_t_init(&file);
	ret = fs_open(&file, path, FS_O_READ);
	if (ret < 0) {
		printk("ASSET: open %s failed (%d)\n", path, ret);
		return ret;
	}

	n = fs_read(&file, buf, sizeof(buf) - 1);
	fs_close(&file);
	if (n < 0) {
		printk("ASSET: read failed (%zd)\n", n);
		return (int)n;
	}
	buf[n] = '\0';
	printk("ASSET: %s (%zd bytes): %s\n", path, n, buf);
	return 0;
}

static int stage_module(const char *path)
{
	struct fs_file_t file;
	ssize_t n;
	int ret;

	fs_file_t_init(&file);
	ret = fs_open(&file, path, FS_O_CREATE | FS_O_TRUNC | FS_O_WRITE);
	if (ret < 0) {
		printk("STAGE: open %s failed (%d)\n", path, ret);
		return ret;
	}

	n = fs_write(&file, hello_ext_buf, sizeof(hello_ext_buf));
	fs_close(&file);
	if (n != sizeof(hello_ext_buf)) {
		printk("STAGE: write failed (%zd)\n", n);
		return -EIO;
	}
	printk("STAGE: %s written (%zu bytes)\n", path, sizeof(hello_ext_buf));
	return 0;
}

static int run_llext(const char *path)
{
	struct llext_fs_loader fs_loader = LLEXT_FS_LOADER(path);
	struct llext_load_param ldr_param = LLEXT_LOAD_PARAM_DEFAULT;
	struct llext *ext;
	int ret;

	ret = llext_load(&fs_loader.loader, "hello", &ext, &ldr_param);
	if (ret != 0) {
		printk("LLEXT: load %s failed (%d)\n", path, ret);
		return ret;
	}
	printk("LLEXT: %s loaded\n", path);

	ret = llext_call_fn(ext, "hello_world");
	if (ret != 0) {
		printk("LLEXT: call hello_world failed (%d)\n", ret);
	} else {
		printk("LLEXT: hello_world() returned\n");
	}

	llext_unload(&ext);
	return ret;
}

int main(void)
{
	int ret;

	ret = fs_mount(&mp);
	if (ret < 0) {
		printk("MOUNT: /SD: failed (%d)\n", ret);
		return 0;
	}
	printk("MOUNT: /SD: ok\n");

	ret = read_asset("/SD:/asset.txt");
	if (ret == 0) {
		ret = stage_module("/SD:/hello2.llext");
	}
	if (ret == 0) {
		ret = run_llext("/SD:/hello2.llext");
	}

	printk(ret == 0 ? "P3-EXIT-GATE: PASS\n" : "P3-EXIT-GATE: FAIL\n");
	return 0;
}
