/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Contiguous DMA allocation returning a struct page, valid on both the
 * dma_direct and iommu_dma_ops paths.
 *
 * The Q10 Pro 32-bit kernel used dma_alloc_attrs(DMA_ATTR_NO_KERNEL_MAPPING)
 * as a "struct page cookie" primitive, because on 32-bit HIGHMEM kernels
 * dma_alloc_pages() throws away CMA blocks that sit in highmem.  That cookie
 * is ONLY a struct page on the dma_direct path: iommu_dma_alloc() ignores
 * DMA_ATTR_NO_KERNEL_MAPPING entirely and returns a vmap'ed kernel address
 * instead, which is not a page -- dereferencing it (clear_highpage on a
 * garbage page_address()) oopses with an abort like
 * "Unable to handle kernel paging request at ffdf803400000000".
 *
 * On ARM64 (no highmem, either ops) the plain dma_alloc_pages() primitive
 * is strictly better: it returns a physically contiguous block, zeroed, as
 * a real struct page, and works identically whether the device ends up on
 * dma_direct (dma_direct_alloc_pages) or iommu_dma_ops
 * (iommu_dma_alloc_pages -> dma_alloc_contiguous + memset).
 */
#ifndef _HISTB_DMA_H
#define _HISTB_DMA_H

#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/slab.h>

/*
 * A physically contiguous DMA block, zeroed.  Returns the first page;
 * *dma_handle is the device address.
 */
static inline struct page *histb_dma_alloc_block(struct device *dev, size_t size,
						 dma_addr_t *dma_handle,
						 enum dma_data_direction dir,
						 gfp_t gfp)
{
	struct page *page;

	page = dma_alloc_pages(dev, size, dma_handle, dir, gfp);
	if (!page)
		return NULL;
	/* dma_alloc_pages() zeroes but does not clean to PoC; hardware may
	 * read the block right away (e.g. blank PMV tables, VPSS staging). */
	dma_sync_single_for_device(dev, *dma_handle, size, dir);
	return page;
}

static inline void histb_dma_free_block(struct device *dev, size_t size,
					struct page *page, dma_addr_t dma_handle)
{
	dma_free_pages(dev, size, page, dma_handle, DMA_BIDIRECTIONAL);
}

/* Cacheable kernel mapping of a block.  ARM64 has no highmem, so the
 * linear-map address of the first page always covers the whole block. */
static inline void *histb_dma_map_block(struct page *page, size_t size)
{
	return page_address(page);
}

static inline void histb_dma_unmap_block(void *vaddr)
{
}

static inline void *histb_dma_alloc_noncoherent(struct device *dev, size_t size,
						dma_addr_t *dma_handle,
						enum dma_data_direction dir,
						gfp_t gfp)
{
	struct page *page;

	page = histb_dma_alloc_block(dev, size, dma_handle, dir, gfp);
	if (!page)
		return NULL;
	return page_address(page);
}

static inline void histb_dma_free_noncoherent(struct device *dev, size_t size,
					      void *vaddr,
					      dma_addr_t dma_handle,
					      enum dma_data_direction dir)
{
	if (!vaddr)
		return;
	dma_free_pages(dev, size, virt_to_page(vaddr), dma_handle, dir);
}

#endif /* _HISTB_DMA_H */
