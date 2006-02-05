/*
 * Copyright (C) 2001-2005 Jakub Jermar
 * Copyright (C) 2005 Sergey Bondari
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <typedefs.h>
#include <arch/types.h>
#include <mm/heap.h>
#include <mm/frame.h>
#include <mm/as.h>
#include <panic.h>
#include <debug.h>
#include <adt/list.h>
#include <synch/spinlock.h>
#include <arch/asm.h>
#include <arch.h>
#include <print.h>
#include <align.h>

SPINLOCK_INITIALIZE(zone_head_lock);	/**< this lock protects zone_head list */
LIST_INITIALIZE(zone_head);            	/**< list of all zones in the system */

/** Blacklist containing non-available areas of memory.
 *
 * This blacklist is used to exclude frames that cannot be allocated
 * (e.g. kernel memory) from available memory map.
 */
region_t zone_blacklist[ZONE_BLACKLIST_SIZE];
count_t zone_blacklist_count = 0;

static struct buddy_system_operations  zone_buddy_system_operations = {
	.find_buddy = zone_buddy_find_buddy,
	.bisect = zone_buddy_bisect,
	.coalesce = zone_buddy_coalesce,
	.set_order = zone_buddy_set_order,
	.get_order = zone_buddy_get_order,
	.mark_busy = zone_buddy_mark_busy,
};

/** Initialize physical memory management
 *
 * Initialize physical memory managemnt.
 */
void frame_init(void)
{
	if (config.cpu_active == 1) {
		frame_region_not_free(KA2PA(config.base), config.kernel_size);
		if (config.init_size > 0)
			frame_region_not_free(KA2PA(config.init_addr), config.init_size);
	}

	frame_arch_init();
}

/**
 * Find AND LOCK zone that can allocate order frames
 *
 * Assume zone_head_lock is locked.
 */
static zone_t * find_free_zone(__u8 order)
{
	link_t *cur;
	zone_t *z;

	for (cur = zone_head.next; cur != &zone_head;cur = cur->next) {
		z = list_get_instance(cur, zone_t, link);
		
		spinlock_lock(&z->lock);
		
		/* Check if the zone has 2^order frames area available  */
		if (buddy_system_can_alloc(z->buddy_system, order))
			return z;
		
		spinlock_unlock(&z->lock);
	}
	return NULL;
}

/** Allocate power-of-two frames of physical memory.
 *
 * @param flags Flags for host zone selection and address processing.
 * @param order Allocate exactly 2^order frames.
 * @param pzone Pointer to preferred zone pointer, on output it changes
 *              to the zone that the frame was really allocated to
 *
 * @return Allocated frame.
 */
__address frame_alloc_generic(__u8 order, int flags, int * status, zone_t **pzone) 
{
	ipl_t ipl;
	link_t *tmp;
	zone_t *zone = NULL;
	frame_t *frame = NULL;
	int freed;
	__address v;
	
loop:
	ipl = interrupts_disable();
	spinlock_lock(&zone_head_lock);

	/*
	 * First, find suitable frame zone.
	 */
	if (pzone && *pzone) {
		spinlock_lock(&(*pzone)->lock);
		if (!buddy_system_can_alloc((*pzone)->buddy_system, order))
			spinlock_unlock(&(*pzone)->lock);
		else
			zone = *pzone;
	}
	if (!zone) {
		zone = find_free_zone(order);
		/* If no memory, reclaim some slab memory,
		   if it does not help, reclaim all */
		if (!zone && !(flags & FRAME_NO_RECLAIM)) {
			spinlock_unlock(&zone_head_lock);
			freed = slab_reclaim(0);
			spinlock_lock(&zone_head_lock);
			if (freed)
				zone = find_free_zone(order);
			if (!zone) {
				spinlock_unlock(&zone_head_lock);
				freed = slab_reclaim(SLAB_RECLAIM_ALL);
				spinlock_lock(&zone_head_lock);
				if (freed)
					zone = find_free_zone(order);
			}
		}
	}

	if (!zone) {
		if (flags & FRAME_PANIC)
			panic("Can't allocate frame.\n");
		
		/*
		 * TODO: Sleep until frames are available again.
		 */
		spinlock_unlock(&zone_head_lock);
		interrupts_restore(ipl);

		if (flags & FRAME_ATOMIC) {
			ASSERT(status != NULL);
			*status = FRAME_NO_MEMORY;
			return NULL;
		}
		
		panic("Sleep not implemented.\n");
		goto loop;
	}
		
	/* Allocate frames from zone buddy system */
	tmp = buddy_system_alloc(zone->buddy_system, order);
	
	ASSERT(tmp);
	
	/* Update zone information. */
	zone->free_count -= (1 << order);
	zone->busy_count += (1 << order);

	/* Frame will be actually a first frame of the block. */
	frame = list_get_instance(tmp, frame_t, buddy_link);
	
	/* get frame address */
	v = FRAME2ADDR(zone, frame);

	spinlock_unlock(&zone->lock);
	spinlock_unlock(&zone_head_lock);
	interrupts_restore(ipl);

	ASSERT(v == ALIGN_UP(v, FRAME_SIZE << order));

	if (flags & FRAME_KA)
		v = PA2KA(v);
	
	if (status)
		*status = FRAME_OK;

	if (pzone)
		*pzone = zone;
	return v;
}

/** Convert address to zone pointer
 *
 * Assume zone_head_lock is held
 *
 * @param addr Physical address
 * @param lock If true, lock the zone
 */
static zone_t * addr2zone(__address addr, int lock)
{
	link_t *cur;
	zone_t *z = NULL;

	for (cur = zone_head.next; cur != &zone_head; cur = cur->next) {
		z = list_get_instance(cur, zone_t, link);
		
		spinlock_lock(&z->lock);
		
		/*
		 * Check if addr belongs to z.
		 */
		if ((addr >= z->base) && (addr <= z->base + (z->free_count + z->busy_count) * FRAME_SIZE)) {
			if (!lock)
				spinlock_unlock(&z->lock);
			return z;
		}

		spinlock_unlock(&z->lock);
	}

	panic("Cannot find addr2zone: 0x%X", addr);
}

/** Return frame_t structure corresponding to address
 *
 * 
 */
frame_t * frame_addr2frame(__address addr)
{
	ipl_t ipl;
	frame_t *frame;
	zone_t *zone;

	if (IS_KA(addr))
		addr = KA2PA(addr);

	/* Disable interrupts to avoid deadlocks with interrupt handlers */
	ipl = interrupts_disable();
	spinlock_lock(&zone_head_lock);
	
	zone = addr2zone(addr,0);
	frame = ADDR2FRAME(zone, addr);

	spinlock_unlock(&zone_head_lock);
	interrupts_restore(ipl);

	return frame;
}


/** Free a frame.
 *
 * Find respective frame structrue for supplied addr.
 * Decrement frame reference count.
 * If it drops to zero, move the frame structure to free list.
 *
 * @param addr Address of the frame to be freed. It must be a multiple of FRAME_SIZE.
 */
void frame_free(__address addr)
{
	ipl_t ipl;
	zone_t *zone;
	frame_t *frame;
	int order;
	
	ASSERT(addr % FRAME_SIZE == 0);
	
	if (IS_KA(addr))
		addr = KA2PA(addr);

	ipl = interrupts_disable();
	spinlock_lock(&zone_head_lock);
	
	/*
	 * First, find host frame zone for addr.
	 */
	zone = addr2zone(addr, 1); /* This locks the zone automatically */
	
	frame = ADDR2FRAME(zone, addr);
	
	/* remember frame order */
	order = frame->buddy_order;

	ASSERT(frame->refcount);

	if (!--frame->refcount) {
		buddy_system_free(zone->buddy_system, &frame->buddy_link);
	}

	/* Update zone information. */
	zone->free_count += (1 << order);
	zone->busy_count -= (1 << order);
	
	spinlock_unlock(&zone->lock);
	spinlock_unlock(&zone_head_lock);
	interrupts_restore(ipl);
}

/** Mark frame region not free.
 *
 * Mark frame region not free.
 *
 * @param base Base address of non-available region.
 * @param size Size of non-available region.
 */
void frame_region_not_free(__address base, size_t size)
{
	index_t index;
	index = zone_blacklist_count++;

	/* Force base to the nearest lower address frame boundary. */
	base = ALIGN_DOWN(base, FRAME_SIZE);
	/* Align size to frame boundary. */
	size = ALIGN_UP(size, FRAME_SIZE);

	ASSERT(index < ZONE_BLACKLIST_SIZE);
	zone_blacklist[index].base = base;
	zone_blacklist[index].size = size;
}

/** Create frame zones in region of available memory.
 *
 * Avoid any black listed areas of non-available memory.
 * Assume that the black listed areas cannot overlap
 * one another or cross available memory region boundaries.
 *
 * @param base Base address of available memory region.
 * @param size Size of the region.
 */
void zone_create_in_region(__address base, size_t size) {
	int i;
	zone_t * z;
	__address s;
	size_t sz;
	
	ASSERT(base % FRAME_SIZE == 0);
	ASSERT(size % FRAME_SIZE == 0);
	
	if (!size)
		return;

	for (i = 0; i < zone_blacklist_count; i++) {
		if (zone_blacklist[i].base >= base && zone_blacklist[i].base < base + size) {
			s = base; sz = zone_blacklist[i].base - base;
			ASSERT(base != s || sz != size);
			zone_create_in_region(s, sz);
			
			s = zone_blacklist[i].base + zone_blacklist[i].size;
			sz = (base + size) - (zone_blacklist[i].base + zone_blacklist[i].size);
			ASSERT(base != s || sz != size);
			zone_create_in_region(s, sz);
			return;
		
		}
	}
	
	z = zone_create(base, size, 0);

	if (!z) {
		panic("Cannot allocate zone (base=%P, size=%d).\n", base, size);
	}
	
	zone_attach(z);
}


/** Create frame zone
 *
 * Create new frame zone.
 *
 * @param start Physical address of the first frame within the zone.
 * @param size Size of the zone. Must be a multiple of FRAME_SIZE.
 * @param flags Zone flags.
 *
 * @return Initialized zone.
 */
zone_t * zone_create(__address start, size_t size, int flags)
{
	zone_t *z;
	count_t cnt;
	int i;
	__u8 max_order;

	ASSERT(start % FRAME_SIZE == 0);
	ASSERT(size % FRAME_SIZE == 0);
	
	cnt = size / FRAME_SIZE;
	
	z = (zone_t *) early_malloc(sizeof(zone_t));
	if (z) {
		link_initialize(&z->link);
		spinlock_initialize(&z->lock, "zone_lock");
	
		z->base = start;
		z->base_index = start / FRAME_SIZE;
		
		z->flags = flags;

		z->free_count = cnt;
		z->busy_count = 0;
		
		z->frames = (frame_t *) early_malloc(cnt * sizeof(frame_t));
		if (!z->frames) {
			early_free(z);
			return NULL;
		}
		
		for (i = 0; i<cnt; i++) {
			frame_initialize(&z->frames[i], z);
		}
		
		/*
		 * Create buddy system for the zone
		 */
		for (max_order = 0; cnt >> max_order; max_order++)
			;
		z->buddy_system = buddy_system_create(max_order, &zone_buddy_system_operations, (void *) z);
		
		/* Stuffing frames */
		for (i = 0; i<cnt; i++) {
			z->frames[i].refcount = 0;
			buddy_system_free(z->buddy_system, &z->frames[i].buddy_link);	
		}
		
	}
	return z;
}

/** Attach frame zone
 *
 * Attach frame zone to zone list.
 *
 * @param zone Zone to be attached.
 */
void zone_attach(zone_t *zone)
{
	ipl_t ipl;
	
	ipl = interrupts_disable();
	spinlock_lock(&zone_head_lock);
	
	list_append(&zone->link, &zone_head);
	
	spinlock_unlock(&zone_head_lock);
	interrupts_restore(ipl);
}

/** Initialize frame structure
 *
 * Initialize frame structure.
 *
 * @param frame Frame structure to be initialized.
 * @param zone Host frame zone.
 */
void frame_initialize(frame_t *frame, zone_t *zone)
{
	frame->refcount = 1;
	frame->buddy_order = 0;
}


/** Buddy system find_buddy implementation
 *
 * @param b Buddy system.
 * @param block Block for which buddy should be found
 *
 * @return Buddy for given block if found
 */
link_t * zone_buddy_find_buddy(buddy_system_t *b, link_t * block) {
	frame_t * frame;
	zone_t * zone;
	index_t index;
	bool is_left, is_right;

	frame = list_get_instance(block, frame_t, buddy_link);
	zone = (zone_t *) b->data;
	ASSERT(IS_BUDDY_ORDER_OK(FRAME_INDEX_ABS(zone, frame), frame->buddy_order));
	
	
	is_left = IS_BUDDY_LEFT_BLOCK_ABS(zone, frame);
	is_right = IS_BUDDY_RIGHT_BLOCK_ABS(zone, frame);
	
	ASSERT(is_left ^ is_right);
	
	if (is_left) {
		index = (FRAME_INDEX(zone, frame)) + (1 << frame->buddy_order);
	} else { // if (is_right)
		index = (FRAME_INDEX(zone, frame)) - (1 << frame->buddy_order);
	}
	
	if (FRAME_INDEX_VALID(zone, index)) {
		if (	zone->frames[index].buddy_order == frame->buddy_order && 
			zone->frames[index].refcount == 0) {
			return &zone->frames[index].buddy_link;
		}
	}
	
	return NULL;	
}

/** Buddy system bisect implementation
 *
 * @param b Buddy system.
 * @param block Block to bisect
 *
 * @return right block
 */
link_t * zone_buddy_bisect(buddy_system_t *b, link_t * block) {
	frame_t * frame_l, * frame_r;

	frame_l = list_get_instance(block, frame_t, buddy_link);
	frame_r = (frame_l + (1 << (frame_l->buddy_order - 1)));
	
	return &frame_r->buddy_link;
}

/** Buddy system coalesce implementation
 *
 * @param b Buddy system.
 * @param block_1 First block
 * @param block_2 First block's buddy
 *
 * @return Coalesced block (actually block that represents lower address)
 */
link_t * zone_buddy_coalesce(buddy_system_t *b, link_t * block_1, link_t * block_2) {
	frame_t * frame1, * frame2;
	
	frame1 = list_get_instance(block_1, frame_t, buddy_link);
	frame2 = list_get_instance(block_2, frame_t, buddy_link);
	
	return frame1 < frame2 ? block_1 : block_2;
}

/** Buddy system set_order implementation
 *
 * @param b Buddy system.
 * @param block Buddy system block
 * @param order Order to set
 */
void zone_buddy_set_order(buddy_system_t *b, link_t * block, __u8 order) {
	frame_t * frame;
	frame = list_get_instance(block, frame_t, buddy_link);
	frame->buddy_order = order;
}

/** Buddy system get_order implementation
 *
 * @param b Buddy system.
 * @param block Buddy system block
 *
 * @return Order of block
 */
__u8 zone_buddy_get_order(buddy_system_t *b, link_t * block) {
	frame_t * frame;
	frame = list_get_instance(block, frame_t, buddy_link);
	return frame->buddy_order;
}

/** Buddy system mark_busy implementation
 *
 * @param b Buddy system
 * @param block Buddy system block
 *
 */
void zone_buddy_mark_busy(buddy_system_t *b, link_t * block) {
	frame_t * frame;
	frame = list_get_instance(block, frame_t, buddy_link);
	frame->refcount = 1;
}

/** Prints list of zones
 *
 */
void zone_print_list(void) {
	zone_t *zone = NULL;
	link_t *cur;
	ipl_t ipl;

	ipl = interrupts_disable();
	spinlock_lock(&zone_head_lock);
	printf("Base address\tFree Frames\tBusy Frames\n");
	printf("------------\t-----------\t-----------\n");
	for (cur = zone_head.next; cur != &zone_head; cur = cur->next) {
		zone = list_get_instance(cur, zone_t, link);
		spinlock_lock(&zone->lock);
		printf("%L\t%d\t\t%d\n",zone->base, zone->free_count, zone->busy_count);
		spinlock_unlock(&zone->lock);
	}
	spinlock_unlock(&zone_head_lock);
	interrupts_restore(ipl);
}

/** Prints zone details
 *
 * @param base Zone base address
 */
void zone_print_one(__address base) {
	zone_t *zone = NULL, *z	;
	link_t *cur;
	ipl_t ipl;

	ipl = interrupts_disable();
	spinlock_lock(&zone_head_lock);
	
	for (cur = zone_head.next; cur != &zone_head; cur = cur->next) {
		z = list_get_instance(cur, zone_t, link);
		if (base == z->base) { 
			zone = z;
			break;
		}
	}
	
	if (!zone) {
		spinlock_unlock(&zone_head_lock);
		interrupts_restore(ipl);
		printf("No zone with address %X\n", base);
		return;
	}
	
	spinlock_lock(&zone->lock);
	printf("Memory zone information\n\n");
	printf("Zone base address: %P\n", zone->base);
	printf("Zone size: %d frames (%dK)\n", zone->free_count + zone->busy_count, ((zone->free_count + zone->busy_count) * FRAME_SIZE) >> 10);
	printf("Allocated space: %d frames (%dK)\n", zone->busy_count, (zone->busy_count * FRAME_SIZE) >> 10);
	printf("Available space: %d (%dK)\n", zone->free_count, (zone->free_count * FRAME_SIZE) >> 10);
	
	printf("\nBuddy allocator structures:\n\n");
	buddy_system_structure_print(zone->buddy_system, FRAME_SIZE);
	
	spinlock_unlock(&zone->lock);
	spinlock_unlock(&zone_head_lock);
	interrupts_restore(ipl);
}

