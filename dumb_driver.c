/*
 * Copyright 2020 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>
#include <xf86drm.h>

#include "drv_helpers.h"
#include "drv_priv.h"
#include "util.h"

/* After xf86drm.h, so the "drm.h" it includes is already guarded out. */
#include <drm/exynos_drm.h>

#define INIT_DUMB_DRIVER_WITH_NAME(driver, _name)                                                  \
	const struct backend backend_##driver = {                                                  \
		.name = _name,                                                                     \
		.init = dumb_driver_init,                                                          \
		.bo_create = drv_dumb_bo_create,                                                   \
		.bo_create_with_modifiers = dumb_bo_create_with_modifiers,                         \
		.bo_destroy = drv_dumb_bo_destroy,                                                 \
		.bo_import = drv_prime_bo_import,                                                  \
		.bo_map = drv_dumb_bo_map,                                                         \
		.bo_unmap = drv_bo_munmap,                                                         \
		.resolve_format_and_use_flags = drv_resolve_format_and_use_flags_helper,           \
	};

#define INIT_DUMB_DRIVER(driver) INIT_DUMB_DRIVER_WITH_NAME(driver, #driver)

static const uint32_t scanout_render_formats[] = { DRM_FORMAT_ARGB8888, DRM_FORMAT_XRGB8888,
						   DRM_FORMAT_ABGR8888, DRM_FORMAT_XBGR8888,
						   DRM_FORMAT_BGR888,	DRM_FORMAT_RGB565 };

static const uint32_t texture_only_formats[] = { DRM_FORMAT_R8, DRM_FORMAT_NV12, DRM_FORMAT_NV21,
						 DRM_FORMAT_YVU420, DRM_FORMAT_YVU420_ANDROID };

static int dumb_driver_init(struct driver *drv)
{
	drv_add_combinations(drv, scanout_render_formats, ARRAY_SIZE(scanout_render_formats),
			     &LINEAR_METADATA, BO_USE_RENDER_MASK | BO_USE_SCANOUT);

	drv_add_combinations(drv, texture_only_formats, ARRAY_SIZE(texture_only_formats),
			     &LINEAR_METADATA, BO_USE_TEXTURE_MASK);

	drv_modify_combination(drv, DRM_FORMAT_R8, &LINEAR_METADATA,
			       BO_USE_HW_VIDEO_ENCODER | BO_USE_HW_VIDEO_DECODER |
				   BO_USE_CAMERA_READ | BO_USE_CAMERA_WRITE |
				   BO_USE_GPU_DATA_BUFFER | BO_USE_SENSOR_DIRECT_DATA);
	drv_modify_combination(drv, DRM_FORMAT_NV12, &LINEAR_METADATA,
			       BO_USE_HW_VIDEO_ENCODER | BO_USE_HW_VIDEO_DECODER |
				   BO_USE_CAMERA_READ | BO_USE_CAMERA_WRITE);
	drv_modify_combination(drv, DRM_FORMAT_NV21, &LINEAR_METADATA, BO_USE_HW_VIDEO_ENCODER);

	return drv_modify_linear_combinations(drv);
}

static int dumb_bo_create_with_modifiers(struct bo *bo, uint32_t width, uint32_t height,
					 uint32_t format, const uint64_t *modifiers, uint32_t count)
{
	for (uint32_t i = 0; i < count; i++) {
		if (modifiers[i] == DRM_FORMAT_MOD_LINEAR) {
			return drv_dumb_bo_create(bo, width, height, format, 0);
		}
	}

	return -EINVAL;
}

/*
 * exynos-drm is paired with a Mali Utgard GPU driven by lima, which imports
 * these buffers as render targets. lima_resource_from_handle() demands a stride
 * of exactly align(width, 16) * bpp, so llvmpipe's 64-pixel padding makes narrow
 * buffers (a 1px-wide divider, say) fail to import.
 */
#define EXYNOS_DUMB_QUIRKS BO_QUIRK_TILE_ALIGN_16

struct exynos_map_data {
	int prime_fd;
};

/*
 * Dumb buffers are write-combined, which is fine for CPU writes but makes CPU
 * reads uncached and very slow on the Cortex-A9. Buffers the CPU reads back
 * (ImageReader, camera YUV consumers, screenshots) get a cached allocation
 * instead, kept coherent with DMA_BUF_IOCTL_SYNC in lock/unlock. Scanout
 * buffers stay write-combined.
 */
/*
 * cros_gralloc adds BO_USE_SW_READ_OFTEN to every HW_VIDEO_ENCODER buffer
 * (b/30054495), but the MFC reads encoder input by DMA. Cached, those buffers
 * only pay a full-buffer cache clean/invalidate per lock (720p recording
 * dropped from ~15 to ~10 fps).
 */
static bool exynos_use_cached(uint64_t use_flags)
{
	return (use_flags & BO_USE_SW_READ_OFTEN) &&
	       !(use_flags & (BO_USE_SCANOUT | BO_USE_PROTECTED | BO_USE_HW_VIDEO_ENCODER));
}

static int exynos_bo_create_cached(struct bo *bo, uint32_t width, uint32_t height,
				   uint32_t format)
{
	struct drm_exynos_gem_create gem_create = { 0 };
	uint32_t aligned_width, aligned_height, bpp, layout_height, pitch;

	drv_dumb_bo_get_dimensions(bo, width, height, format, EXYNOS_DUMB_QUIRKS, &aligned_width,
				   &aligned_height, &bpp, &layout_height);
	/* Same pitch and size as exynos_drm_gem_dumb_create() would pick. */
	pitch = aligned_width * DIV_ROUND_UP(bpp, 8);

	gem_create.size = (uint64_t)pitch * aligned_height;
	gem_create.flags = EXYNOS_BO_NONCONTIG | EXYNOS_BO_CACHABLE;

	if (drmIoctl(bo->drv->fd, DRM_IOCTL_EXYNOS_GEM_CREATE, &gem_create)) {
		drv_loge("DRM_IOCTL_EXYNOS_GEM_CREATE failed (size=%llu, errno=%d)\n",
			 (unsigned long long)gem_create.size, errno);
		return -errno;
	}

	drv_bo_from_format(bo, pitch, 1, layout_height, format);

	bo->handle.u32 = gem_create.handle;
	bo->meta.total_size = gem_create.size;
	bo->meta.cached = true;
	return 0;
}

static int exynos_bo_create(struct bo *bo, uint32_t width, uint32_t height, uint32_t format,
			    uint64_t use_flags)
{
	/* A failed cached allocation falls back to a write-combined dumb buffer. */
	if (exynos_use_cached(use_flags) && !exynos_bo_create_cached(bo, width, height, format))
		return 0;

	return drv_dumb_bo_create_ex(bo, width, height, format, use_flags, EXYNOS_DUMB_QUIRKS);
}

static int exynos_bo_create_with_modifiers(struct bo *bo, uint32_t width, uint32_t height,
					   uint32_t format, const uint64_t *modifiers,
					   uint32_t count)
{
	for (uint32_t i = 0; i < count; i++) {
		if (modifiers[i] == DRM_FORMAT_MOD_LINEAR)
			return exynos_bo_create(bo, width, height, format, 0);
	}

	return -EINVAL;
}

static void *exynos_bo_map(struct bo *bo, struct vma *vma, uint32_t map_flags)
{
	struct drm_exynos_gem_info gem_info = { 0 };
	struct exynos_map_data *priv;
	int prime_fd;
	void *addr;

	addr = drv_dumb_bo_map(bo, vma, map_flags);
	if (addr == MAP_FAILED)
		return addr;

	/*
	 * Ask the kernel rather than trusting use_flags: an imported bo may come
	 * from another process's allocation, or from a kernel without cached
	 * GEM support.
	 */
	gem_info.handle = bo->handle.u32;
	if (drmIoctl(bo->drv->fd, DRM_IOCTL_EXYNOS_GEM_GET, &gem_info) ||
	    !(gem_info.flags & EXYNOS_BO_CACHABLE) || (gem_info.flags & EXYNOS_BO_WC))
		return addr;

	/* A cached mapping without cache maintenance would be incoherent. */
	if (drmPrimeHandleToFD(bo->drv->fd, bo->handle.u32, DRM_CLOEXEC, &prime_fd)) {
		drv_loge("drmPrimeHandleToFD failed for cached bo (errno=%d)\n", errno);
		munmap(addr, vma->length);
		return MAP_FAILED;
	}

	priv = calloc(1, sizeof(*priv));
	if (!priv) {
		close(prime_fd);
		munmap(addr, vma->length);
		return MAP_FAILED;
	}

	priv->prime_fd = prime_fd;
	vma->priv = priv;
	return addr;
}

static int exynos_bo_unmap(struct bo *bo, struct vma *vma)
{
	struct exynos_map_data *priv = vma->priv;

	if (priv) {
		close(priv->prime_fd);
		free(priv);
		vma->priv = NULL;
	}

	return munmap(vma->addr, vma->length);
}

static int exynos_bo_sync(struct mapping *mapping, uint64_t flags)
{
	struct exynos_map_data *priv = mapping->vma->priv;
	struct dma_buf_sync sync = { 0 };

	if (!priv)
		return 0;

	sync.flags = flags;
	if (mapping->vma->map_flags & BO_MAP_READ)
		sync.flags |= DMA_BUF_SYNC_READ;
	if (mapping->vma->map_flags & BO_MAP_WRITE)
		sync.flags |= DMA_BUF_SYNC_WRITE;

	if (drmIoctl(priv->prime_fd, DMA_BUF_IOCTL_SYNC, &sync)) {
		drv_loge("DMA_BUF_IOCTL_SYNC failed (flags=0x%llx, errno=%d)\n",
			 (unsigned long long)sync.flags, errno);
		return -errno;
	}

	return 0;
}

/* Also waits for the GPU's implicit fences before the CPU reads. */
static int exynos_bo_invalidate(struct bo *bo, struct mapping *mapping)
{
	return exynos_bo_sync(mapping, DMA_BUF_SYNC_START);
}

static int exynos_bo_flush(struct bo *bo, struct mapping *mapping)
{
	return exynos_bo_sync(mapping, DMA_BUF_SYNC_END);
}

const struct backend backend_exynos = {
	.name = "exynos",
	.init = dumb_driver_init,
	.bo_create = exynos_bo_create,
	.bo_create_with_modifiers = exynos_bo_create_with_modifiers,
	.bo_destroy = drv_dumb_bo_destroy,
	.bo_import = drv_prime_bo_import,
	.bo_map = exynos_bo_map,
	.bo_unmap = exynos_bo_unmap,
	.bo_invalidate = exynos_bo_invalidate,
	.bo_flush = exynos_bo_flush,
	.resolve_format_and_use_flags = drv_resolve_format_and_use_flags_helper,
};

INIT_DUMB_DRIVER(evdi)
INIT_DUMB_DRIVER(komeda)
INIT_DUMB_DRIVER(marvell)
INIT_DUMB_DRIVER(meson)
INIT_DUMB_DRIVER(nouveau)
INIT_DUMB_DRIVER(radeon)
INIT_DUMB_DRIVER_WITH_NAME(sun4i_drm, "sun4i-drm")
INIT_DUMB_DRIVER(synaptics)
INIT_DUMB_DRIVER(udl)
INIT_DUMB_DRIVER(vkms)

#ifndef DRV_ROCKCHIP
INIT_DUMB_DRIVER(rockchip)
#endif
#ifndef DRV_MEDIATEK
INIT_DUMB_DRIVER(mediatek)
#endif
