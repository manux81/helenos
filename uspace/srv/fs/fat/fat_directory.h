/*
 * Copyright (c) 2008 Jakub Jermar
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

/** @addtogroup fs
 * @{
 */ 

#ifndef FAT_FAT_DIRECTORY_H_
#define FAT_FAT_DIRECTORY_H_

#include <stdint.h>
#include "fat.h"
#include "fat_dentry.h"

typedef struct {
	/* Directory data */
	fat_bs_t *bs;
	fat_node_t *nodep;
	uint32_t blocks;
	uint32_t bnum;
	aoff64_t pos;
	block_t *b;
	bool last;
	/* Long entry data */
	uint16_t wname[FAT_LFN_MAX_COUNT * FAT_LFN_ENTRY_SIZE];
	size_t lfn_offset;
	size_t lfn_size;
	bool long_entry;
	int long_entry_count;
	uint8_t checksum;
} __attribute__ ((packed)) fat_directory_t;


extern int fat_directory_open(fat_node_t *, fat_directory_t *);
extern int fat_directory_close(fat_directory_t *);

extern int fat_directory_next(fat_directory_t *);
extern int fat_directory_prev(fat_directory_t *);
extern int fat_directory_seek(fat_directory_t *, aoff64_t pos);
extern int fat_directory_get(fat_directory_t *, fat_dentry_t **);
extern int fat_directory_dirty(fat_directory_t *);

extern int fat_directory_read(fat_directory_t *, char *, fat_dentry_t **);
extern int fat_directory_erase(fat_directory_t *);


#endif

/**
 * @}
 */
