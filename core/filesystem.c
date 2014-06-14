/*
 * Copyright (c) 2014, Peter Rutenbar <pruten@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "../core/shoebill.h"

/* --- Disk/partition management stuff --- */
#pragma mark Disk/partition management stuff

typedef struct __attribute__ ((__packed__)) {
    uint32_t magic;
    uint8_t cluster;
    uint8_t type;
    uint16_t inode;
    
    // no no no SLICErrr rrrrrCUR (NOPE, sys-V compiler orders these bits backwards)
    // yes RUCrrrrr rrrSLICE
    
    uint8_t dummy:5;
    uint8_t crit:1;
    uint8_t usr:1;
    uint8_t root:1;
    
    uint8_t slice:5;
    uint8_t dummy2:3;
    
    uint16_t dummy3;
    uint32_t tmade;
    uint32_t tmount;
    uint32_t tunmount;
    
    /* "Alternate block map" */
    uint32_t abm_size;
    uint32_t abm_ents;
    uint32_t abm_start;
} block_zero_t;

typedef struct __attribute__ ((__packed__)) {
    uint8_t pmSig[2];
    uint16_t pmSigPad;
    uint32_t pmMapBlkCnt;
    uint32_t pmPyPartStart;
    uint32_t pmPartBlkCnt;
    char pmPartName[32];
    char pmPartType[32];
    uint32_t pmLgDataStart;
    uint32_t pmDataCnt;
    uint32_t pmPartStatus;
    uint32_t pmLgBootStart;
    uint32_t pmBootSize;
    uint32_t pmBootAddr;
    uint32_t pmBootAddr2;
    uint32_t pmBootEntry;
    uint32_t pmBootEntry2;
    uint32_t pmBootCksum;
    char pmProcessor[16];
    block_zero_t bz;
} apple_partition_map_t;

typedef struct __attribute__ ((__packed__)) {
    uint8_t sbSig[2]; // device signature
    uint16_t sbBlkSize; // block size of the device
    uint32_t sbBlkCount; // number of blocks on the device
    uint16_t sbDevType; // reserved
    uint16_t sbDevId; // reserved
    uint32_t sbData; // reserved
    uint16_t sbDrvrCount; // nnumber of driver descriptor entries
    uint32_t ddBlock; // first driver's starting block
    uint16_t ddSize; // size of the driver, in 512-byte blocks
    uint16_t ddType; // operating system type (MacOS = 1)
} driver_descriptor_record_t;


typedef struct {
    struct _disk_t *disk;
    char *error_str;
    alloc_pool_t *pool;
    uint32_t start_block, num_blocks;
    char name[33], type[33];
} partition_t;

typedef struct _disk_t {
    // -- fill these out --
    const char *path;
    char *error_str;
    
    // -- "private" --
    alloc_pool_t *pool;
    FILE *f;
    uint32_t block_size;
    
    driver_descriptor_record_t ddr;
    
    uint32_t num_partitions;
    apple_partition_map_t *partition_maps;
    partition_t *partitions;
} disk_t;

static void disk_get_block (disk_t *disk, uint8_t buf[512], uint32_t blockno)
{
    const uint32_t block_size = disk->block_size;
    assert(0 == fseeko(disk->f, block_size * blockno, SEEK_SET));
    assert(fread(buf, block_size, 1, disk->f) == 1);
}

static void part_get_block(partition_t *part, uint8_t buf[512], uint32_t blockno)
{
    assert(blockno < part->num_blocks);
    disk_get_block(part->disk, buf, part->start_block + blockno);
}

static uint8_t disk_load_partition_map(disk_t *disk, apple_partition_map_t *apm, uint32_t idx)
{
    uint8_t block[512];
    
    disk_get_block(disk, block, 1 + idx);
    memcpy(apm, block, sizeof(apple_partition_map_t));
    
    fix_endian(apm->pmSigPad);
    fix_endian(apm->pmMapBlkCnt);
    fix_endian(apm->pmPyPartStart);
    fix_endian(apm->pmPartBlkCnt);
    fix_endian(apm->pmLgDataStart);
    fix_endian(apm->pmDataCnt);
    fix_endian(apm->pmPartStatus);
    fix_endian(apm->pmLgBootStart);
    fix_endian(apm->pmBootSize);
    fix_endian(apm->pmBootAddr);
    fix_endian(apm->pmBootAddr2);
    fix_endian(apm->pmBootEntry);
    fix_endian(apm->pmBootEntry2);
    fix_endian(apm->pmBootCksum);
    
    fix_endian(apm->bz.magic);
    fix_endian(apm->bz.inode);
    fix_endian(apm->bz.tmade);
    fix_endian(apm->bz.tmount);
    fix_endian(apm->bz.tunmount);
    fix_endian(apm->bz.abm_size);
    fix_endian(apm->bz.abm_ents);
    fix_endian(apm->bz.abm_start);
    
    if (memcmp(apm->pmSig, "PM", 2) != 0) {
        sprintf(disk->error_str, "partition index %u has bad magic %02x%02x",
                idx, apm->pmSig[0], apm->pmSig[1]);
        return 0;
    }
    
    return 1;
}

static void close_disk(disk_t *disk)
{
    fclose(disk->f);
    p_free_pool(disk->pool);
}

static disk_t* open_disk (const char *disk_path, char *error_str)
{
    disk_t *disk;
    uint8_t block[512];
    apple_partition_map_t apm;
    uint32_t i;
    alloc_pool_t *pool = p_new_pool(NULL);
    FILE *f;
    
    disk = p_alloc(pool, sizeof(disk_t));
    
    disk->pool = pool;
    disk->block_size = 512;
    disk->error_str = error_str;
    disk->path = disk_path;
    
    f = fopen(disk_path, "rb");
    if (f == NULL) {
        sprintf(error_str, "Can't open that path");
        goto fail;
    }
    
    disk->f = f;
    
    // Load the driver descriptor record
    
    disk_get_block(disk, block, 0);
    memcpy(&disk->ddr, block, sizeof(disk->ddr));
    
    fix_endian(disk->ddr.sbBlkSize);
    fix_endian(disk->ddr.sbBlkCount);
    fix_endian(disk->ddr.sbDevType);
    fix_endian(disk->ddr.sbDevId);
    fix_endian(disk->ddr.sbData);
    fix_endian(disk->ddr.sbDrvrCount);
    fix_endian(disk->ddr.ddBlock);
    fix_endian(disk->ddr.ddSize);
    fix_endian(disk->ddr.ddType);

    // If the DDR block exists, (it doesn't have to necessarially)
    if (memcmp(disk->ddr.sbSig, "ER", 2) == 0) {
        // Can't handle non-512 byte block sizes
        if (disk->ddr.sbBlkSize != 512) {
            sprintf(error_str, "This disk uses blkSize=%u and I can't handle that",
                    disk->ddr.sbBlkSize);
            goto fail;
        }
    }
    
    // slog("sizeof(apple_part_map_t) = %lu\n", sizeof(apple_partition_map_t));
    
    // Load the partition maps
    
    if (!disk_load_partition_map(disk, &apm, 0))
        goto fail;
    else if ((apm.pmMapBlkCnt > 256) || (apm.pmMapBlkCnt == 0)) {
        sprintf(error_str, "Crazy number of partitions on this disk %u", apm.pmMapBlkCnt);
        goto fail;
    }
    
    disk->num_partitions = apm.pmMapBlkCnt;
    disk->partition_maps = p_alloc(disk->pool, disk->num_partitions * sizeof(apple_partition_map_t));
    disk->partitions = p_alloc(disk->pool, disk->num_partitions * sizeof(partition_t));
    
    for (i=0; i<disk->num_partitions; i++) {
        if (!disk_load_partition_map(disk, &disk->partition_maps[i], i))
            goto fail;
        
        memset(&disk->partitions[i], 0, sizeof(partition_t));
        disk->partitions[i].disk = disk;
        disk->partitions[i].pool = disk->pool;
        disk->partitions[i].error_str = error_str;
        disk->partitions[i].start_block = disk->partition_maps[i].pmPyPartStart;
        disk->partitions[i].num_blocks = disk->partition_maps[i].pmPartBlkCnt;
        
        memcpy(disk->partitions[i].name, disk->partition_maps[i].pmPartName, 32);
        memcpy(disk->partitions[i].type, disk->partition_maps[i].pmPartType, 32);
        
        slog("%u type:%s name:%s\n", i, disk->partitions[i].type, disk->partitions[i].name);
        slog("bz_magic=0x%08x slice=%u\n", disk->partition_maps[i].bz.magic, disk->partition_maps[i].bz.slice);
    }
    
    return disk;
    
fail:
    if (f) fclose(f);
    p_free_pool(pool);
    return NULL;
}

/*static uint8_t translate_aux_to_apm_partition_num(disk_t *disk,
                                                  uint32_t aux_idx,
                                                  uint32_t *apm_idx)
{
    uint32_t i, aux_i = 0;
    
    for (i=0; i<disk->num_partitions; i++) {
        partition_t *part = &disk->partitions[i];
        if (strcmp("Apple_UNIX_SVR2", part->type) != 0)
            continue;
        else if (strstr(part->name, "Eschatology") != NULL)
            continue;
        else if (aux_i == aux_idx) {
            *apm_idx = i;
            return 1;
        }
        aux_i++;
    }
    
    return 0;
}*/

static int32_t find_root_partition_number(disk_t *disk, uint8_t clus_num)
{
    /* 
     * See the man page for bzb for the full scoop.
     * Basically, A/UX partition 0 is the first partition with
     * ** type="Apple_UNIX_SVR2", 
     * ** a matching cluster number, and
     * ** the root bit set
     *
     * A/UX partition 1 is the first partition with
     * ** type="Apple_UNIX_SVR2",
     * ** a matching cluster number, and
     * ** the swap filesystem type
     *
     * A/UX partition 2 is the next partition with
     * ** type="Apple_UNIX_SVR2",
     * ** a matching cluster number
     * ** and the user bit set
     *
     * The logic is probably even more arbitrary
     * and complicated than that. Why doesn't A/UX 
     * just use the native APM partition numbers?
     */
    
    uint32_t i;
    for (i=0; i<disk->num_partitions; i++) {
        partition_t *part = &disk->partitions[i];
        apple_partition_map_t *apm = &disk->partition_maps[i];
        
        // slog("%u magic=0x%08x root=%u type=%s\n", i, apm->bz.magic, apm->bz.root, part->type);
        
        if (apm->bz.magic != 0xabadbabe)
            continue;
        
        if (!apm->bz.root)
            continue;
        
        if (apm->bz.cluster != clus_num)
            continue;
            
        // slice==0 -> This partition doesn't prefer any particular slice number
        // slice==N -> This partition prefers slice number (N-1)
        // (I think)
        /*if (apm->bz.slice != 0 && apm->bz.slice != 1)
            continue;*/
        
        if (strcmp("Apple_UNIX_SVR2", part->type) != 0)
            continue;
        
        return (int32_t)i;
    }
    return -1;
}

#pragma mark SVFS stuff
/* --- SVFS stuff --- */

typedef struct __attribute__ ((__packed__)) {
    uint16_t isize;
    uint32_t fsize;
    uint16_t nfree;
    uint32_t free[50];
    uint16_t ninode;
    uint16_t inode[100];
    uint8_t flock;
    uint8_t ilock;
    uint8_t fmod;
    uint8_t ronly;
    uint32_t time;
    uint16_t dinfo[4];
    uint32_t tfree;
    uint16_t tinode;
    uint8_t fname[6];
    uint8_t fpack[6];
    uint32_t _unused[13];
    uint32_t state;
    uint16_t lasti;
    uint16_t nbehind;
    uint32_t magic;
    uint32_t type;
} svfs_superblock_t;

typedef struct __attribute__ ((__packed__)) {
    uint16_t mode;
    uint16_t nlink;
    uint16_t uid;
    uint16_t gid;
    uint32_t size;
    uint8_t __addr[39];
    uint8_t gen;
    uint32_t atime;
    uint32_t mtime;
    uint32_t ctime;
    
    uint32_t addr[13];
} svfs_inode_t;

typedef struct {
    char *error_str;
    alloc_pool_t *pool;
    partition_t *part;
    
    svfs_superblock_t superblock;
    
    uint32_t blocksize; // SVFS can use 512, 1024, 2048, 4096-byte block sizes
    
} svfs_t;

static uint8_t svfs_read_block(svfs_t *mount, uint8_t *block, uint32_t blockno)
{
    const uint32_t sectors_per_block = mount->blocksize / 512;
    const uint32_t start_sector = blockno * sectors_per_block;
    uint32_t i;
    
    // slog("sectors_per_block = %u, start_sector=%u\n", sectors_per_block, start_sector);
    
    for (i=0; i<sectors_per_block; i++) {
        part_get_block(mount->part, &block[i * 512], start_sector+i);
    }
    
    return 1;
}

static uint8_t svfs_load_inode(svfs_t *mount, svfs_inode_t *inode, uint32_t inum)
{
    uint32_t i;
    uint8_t block[4096];
    const uint32_t max_inode_blocks = mount->superblock.isize - 2;
    const uint32_t max_inodes = (max_inode_blocks * mount->blocksize) / 64;
    if (inum > max_inodes) {
        sprintf(mount->error_str, "svfs_load_inode: inode %u too big (max=%u)", inum, max_inodes);
        return 0;
    }
    
    const uint32_t inum_byte_idx_in_partition = ((inum-1) * 64) + (2 * mount->blocksize);
    const uint32_t inum_block = inum_byte_idx_in_partition / mount->blocksize;
    const uint32_t inum_byte_idx_in_block = inum_byte_idx_in_partition % mount->blocksize;
    
    if (!svfs_read_block(mount, block, inum_block))
        return 0;
    
    memcpy(inode, &block[inum_byte_idx_in_block], 64);
    
    fix_endian(inode->mode);
    fix_endian(inode->nlink);
    fix_endian(inode->uid);
    fix_endian(inode->gid);
    fix_endian(inode->size);
    fix_endian(inode->atime);
    fix_endian(inode->mtime);
    fix_endian(inode->ctime);
    
    for (i=0; i<13; i++) {
        uint32_t addr = inode->__addr[i*3 + 0];
        addr = (addr << 8) + inode->__addr[i*3 + 1];
        addr = (addr << 8) + inode->__addr[i*3 + 2];
        inode->addr[i] = addr;
    }
    
    return 1;
}

static uint8_t svfs_read_level(svfs_t *mount,
                               svfs_inode_t *inode,
                               uint8_t *buf,
                               uint32_t *len,
                               uint32_t *indirects,
                               uint32_t level)
{
    uint8_t *tmp = p_alloc(mount->pool, mount->blocksize);
    const uint32_t num_indirects = mount->blocksize / 4;
    uint32_t i;
    
    for (i=0; (i<num_indirects) && (*len < inode->size); i++) {
        uint32_t chunk_size = inode->size - *len;
        if (chunk_size > mount->blocksize)
            chunk_size = mount->blocksize;
        
        const uint32_t addr = ntohl(indirects[i]);
        
        if (!svfs_read_block(mount, tmp, addr)) {
            sprintf(mount->error_str, "couldn't read svfs block num %u at L%u", addr, level);
            goto fail;
        }
        
        if (level == 1) {
            memcpy(buf + *len, tmp, chunk_size);
            *len += chunk_size;
        }
        else {
            if (!svfs_read_level(mount, inode, buf, len, (uint32_t*)tmp, level-1))
                goto fail;
        }
    }
    
    p_free(tmp);
    return 1;
    
fail:
    p_free(tmp);
    return 0;
}
                               

static uint8_t* svfs_read_inode_data(svfs_t *mount, svfs_inode_t *inode)
{
    uint8_t *tmp = p_alloc(mount->pool, mount->blocksize);
    uint8_t *buf = p_alloc(mount->pool, inode->size);
    uint32_t i, len = 0;
    
    // The first 10 block pointers in the inode point to data
    // The addr[10] is a L1 block pointer, [11] is L2, and [12] is L3
    for (i=0; (len < inode->size) && (i < 10); i++) {
        uint32_t chunk_size = inode->size - len;
        if (chunk_size > mount->blocksize)
            chunk_size = mount->blocksize;
        
        if (!svfs_read_block(mount, tmp, inode->addr[i])) {
            sprintf(mount->error_str, "couldn't read svfs block num %u at L0", inode->addr[i]);
            goto fail;
        }
        
        memcpy(buf + len, tmp, chunk_size);
        len += chunk_size;
    }
    
    
    if (!svfs_read_block(mount, tmp, inode->addr[10])) {
        sprintf(mount->error_str, "couldn't read svfs L1 block ptr %u", inode->addr[10]);
        goto fail;
    }
    else if (!svfs_read_level(mount, inode, buf, &len, (uint32_t*)tmp, 1))
        goto fail;
    
    if (!svfs_read_block(mount, tmp, inode->addr[11])) {
        sprintf(mount->error_str, "couldn't read svfs L2 block ptr %u", inode->addr[11]);
        goto fail;
    }
    else if (!svfs_read_level(mount, inode, buf, &len, (uint32_t*)tmp, 2))
        goto fail;
    
    if (!svfs_read_block(mount, tmp, inode->addr[12])) {
        sprintf(mount->error_str, "couldn't read svfs L3 block ptr %u", inode->addr[12]);
        goto fail;
    }
    else if (!svfs_read_level(mount, inode, buf, &len, (uint32_t*)tmp, 3))
        goto fail;
    
    p_free(tmp);
    return buf;
    
fail:
    p_free(tmp);
    p_free(buf);
    return NULL;
}

static svfs_t* svfs_mount(partition_t *part)
{
    assert(sizeof(svfs_superblock_t) == 512);
    
    uint32_t i;
    svfs_t *mount = p_alloc(part->pool, sizeof(svfs_t));
    mount->pool = part->pool;
    mount->error_str = part->error_str;
    mount->part = part;
    
    part_get_block(part, (uint8_t*)&mount->superblock, 1);
    
    fix_endian(mount->superblock.isize);
    fix_endian(mount->superblock.fsize);
    fix_endian(mount->superblock.nfree);
    for (i=0; i<50; i++) {
        fix_endian(mount->superblock.free[i]);
    }
    fix_endian(mount->superblock.ninode);
    for (i=0; i<100; i++) {
        fix_endian(mount->superblock.inode[i]);
    }
    fix_endian(mount->superblock.time);
    for (i=0; i<4; i++) {
        fix_endian(mount->superblock.dinfo[i]);
    }
    fix_endian(mount->superblock.tfree);
    fix_endian(mount->superblock.tinode);
    fix_endian(mount->superblock.state);
    fix_endian(mount->superblock.lasti);
    fix_endian(mount->superblock.nbehind);
    fix_endian(mount->superblock.magic);
    fix_endian(mount->superblock.type);
    
    if (mount->superblock.magic != 0xfd187e20) {
        sprintf(part->error_str, "Magic doesn't match svfs");
        goto fail;
    }
    
    // It is SVFS!
    
    const uint32_t type = mount->superblock.type;
    if ((type != 1) && (type != 2) && (type != 4) && (type != 8)) {
        sprintf(part->error_str, "Unknown SVFS type (%u)", type);
        goto fail;
    }
    
    mount->blocksize = 512 * type;
    
    return mount;
    
fail:
    if (mount) p_free(mount);
    
    return NULL;
}

typedef struct __attribute__ ((__packed__)) {
    uint16_t inum;
    char name[14];
} svfs_dir_entry_t;

svfs_inode_t* svfs_traverse_path(svfs_t *mount, const char *_path)
{
    uint32_t i;
    uint16_t inum = 2; // 2 == root
    svfs_inode_t *inode = p_alloc(mount->pool, sizeof(svfs_inode_t));
    char *path = p_alloc(mount->pool, strlen(_path)+1);
    strcpy(path, _path);
    
    if (!svfs_load_inode(mount, inode, inum))
        goto fail;
    
    char *last, *elem;
    for (elem = strtok_r(path, "/", &last);
         elem;
         elem = strtok_r(NULL, "/", &last)) {
        //slog("elem = [%s]\n", elem);
        const uint32_t num_entries = inode->size / 16;
        //slog("inode size = %u\n", inode->size);
        svfs_dir_entry_t *dir = (svfs_dir_entry_t*)svfs_read_inode_data(mount, inode);
        if (!dir)
            goto fail;
        
        for (i=0; i<num_entries; i++) {
            if (strncmp(dir[i].name, elem, 14) == 0) {
                inum = ntohs(dir[i].inum);
                if (!svfs_load_inode(mount, inode, inum)) {
                    p_free(dir);
                    goto fail;
                }
                break;
            }
        }
        p_free(dir);
        if (i == num_entries) {
            sprintf(mount->error_str, "'%s' in '%s' doesn't exist", elem, _path);
            goto fail;
        }
    }
    //slog("final inode size = %u\n", inode->size);
    p_free(path);
    return inode;
    
fail:
    p_free(inode);
    p_free(path);
    return NULL;
}

/* --- UFS stuff --- */
#pragma mark UFS stuff

typedef struct __attribute__ ((__packed__)) {
    uint32_t link;
    uint32_t rlink;
    uint32_t sblkno;
    uint32_t cblkno;
    uint32_t iblkno;
    uint32_t dblkno;
    uint32_t cgoffset;
    uint32_t cgmask;
    uint32_t time;
    uint32_t size;
    uint32_t dsize;
    uint32_t ncg;
    uint32_t bsize;
    uint32_t fsize;
    uint32_t frag;
    uint32_t minfree;
    uint32_t rotdelay;
    uint32_t rps;
    uint32_t bmask;
    uint32_t fmask;
    uint32_t bshift;
    uint32_t fshift;
    uint32_t maxcontig;
    uint32_t maxbpg;
    uint32_t fragshift;
    uint32_t fsbtodb;
    uint32_t sbsize;
    uint32_t csmask;
    uint32_t csshift;
    uint32_t nindir;
    uint32_t inopb;
    uint32_t nspf;
    uint32_t optim;
    uint32_t dummy[2];
    uint32_t state;
    uint32_t id[2];
    uint32_t csaddr;
    uint32_t cssize;
    uint32_t cgsize;
    uint32_t ntrak;
    uint32_t nsect;
    uint32_t spc;
    uint32_t ncyl;
    uint32_t cpg;
    uint32_t ipg;
    uint32_t fpg;
    uint32_t csum_ndir;
    uint32_t csum_nbfree;
    uint32_t csum_nifree;
    uint32_t csum_nffree;
    uint8_t fmod;
    uint8_t clean;
    uint8_t ronly;
    uint8_t flags;
    uint8_t fsmnt[500];
    uint8_t fname[6];
    uint8_t fpack[6];
    uint32_t cgrotor;
    uint32_t dummy2[32];
    uint32_t cpc;
    uint16_t postbl[32][8];
    uint32_t magic;
} ufs_superblock_t;

typedef struct __attribute__ ((__packed__)) {
    uint32_t link;
    uint32_t rlink;
    uint32_t time;
    uint32_t cgx;
    uint16_t ncyl;
    uint16_t niblk;
    uint32_t ndblk;
    uint32_t csum_ndir;
    uint32_t csum_nbfree;
    uint32_t csum_nifree;
    uint32_t csum_nffree;
    uint32_t rotor;
    uint32_t frotor;
    uint32_t irotor;
    uint32_t frsum[8];
    uint32_t btot[32];
    uint16_t b[32][8];
    uint8_t iused[256];
    uint32_t magic;
} ufs_cylinder_group_t;

typedef struct {
    char *error_str;
    alloc_pool_t *pool;
    partition_t *part;
    
    uint32_t frag_size;
    uint32_t block_size;
    uint32_t frag_per_block;
    
    ufs_superblock_t superblock;
    
    uint32_t num_groups;
    ufs_cylinder_group_t *groups;
} ufs_t;

typedef struct __attribute__ ((__packed__)) {
    uint16_t mode;
    uint16_t nlink;
    uint16_t uid;
    uint16_t gid;
    uint32_t size_hi; // UFS stores size as a uint64_t, but A/UX apparently only reads/writes
    uint32_t size; // to the low bits, so sometimes the hi bits contain garbage
    uint32_t atime;
    uint32_t dummy;
    uint32_t mtime;
    uint32_t dummy2;
    uint32_t ctime;
    uint32_t dummy3;
    uint32_t direct[12];
    uint32_t indirect[3];
    uint32_t flags;
    uint32_t blocks;
    uint32_t gen;
    uint32_t dummy4[4];
    
    
} ufs_inode_t;

/*
 * I truly don't understand the concept behind cgoffset/cgmask,
 * but this is apparently the algorithm for finding the fragment-offset
 * into a cylinder group.
 */
#define ufs_group_base(mount, num) ( \
    ((mount)->superblock.fpg * (num)) + \
    ((mount)->superblock.cgoffset * \
     ((num) & ~(mount)->superblock.cgmask)))

static uint8_t ufs_read_frag(ufs_t *mount, uint8_t *frag, uint32_t fragno)
{
    const uint32_t sectors_per_frag = mount->frag_size / 512;
    const uint32_t start_sector = fragno * sectors_per_frag;
    uint32_t i;
    
    for (i=0; i<sectors_per_frag; i++) {
        part_get_block(mount->part, &frag[i * 512], start_sector+i);
    }
    
    return 1;
}

static uint8_t ufs_read_block(ufs_t *mount, uint8_t *block, uint32_t blockno)
{
    uint32_t i;
    
    /* 
     * block numbers and fragment numbers are identical - they both refer
     * to fragment numbers. But if we're reading a "block", then we're reading
     * mount->frag_per_block fragments starting at that block address.
     */
    
    assert((blockno % mount->frag_per_block) == 0); // This had better align to a block boundary
    
    for (i=0; i<mount->frag_per_block; i++) {
        if (!ufs_read_frag(mount, block + i * mount->frag_size, blockno + i))
            return 0;
    }
    return 1;
}

static uint8_t ufs_load_cylinder_group(ufs_t *mount, uint32_t frag_offset, ufs_cylinder_group_t *group)
{
    uint32_t numfrags = sizeof(ufs_cylinder_group_t) / mount->frag_size;
    numfrags += ((sizeof(ufs_cylinder_group_t) % mount->frag_size) != 0);
    
    uint8_t *buf = p_alloc(mount->pool, (numfrags+1) * mount->frag_size);
    uint32_t i;
    
    for (i=0; i <= numfrags; i++)
        ufs_read_frag(mount, &buf[i * mount->frag_size], frag_offset + i);
    memcpy(group, buf, sizeof(ufs_cylinder_group_t));
    
    fix_endian(group->link);
    fix_endian(group->rlink);
    fix_endian(group->time);
    fix_endian(group->cgx);
    fix_endian(group->ncyl);
    fix_endian(group->niblk);
    fix_endian(group->ndblk);
    fix_endian(group->csum_ndir);
    fix_endian(group->csum_nbfree);
    fix_endian(group->csum_nifree);
    fix_endian(group->csum_nffree);
    fix_endian(group->rotor);
    fix_endian(group->frotor);
    fix_endian(group->irotor);
    for (i=0; i<8; i++)
        fix_endian(group->frsum[i]);
    for (i=0; i<(32*8); i++)
        fix_endian(group->b[i/8][i%8]);
    fix_endian(group->magic);
    
    p_free(buf);
    return 1;
fail:
    p_free(buf);
    return 0;
}

static uint8_t ufs_load_inode(ufs_t *mount, ufs_inode_t *inode, uint32_t inum)
{
    assert(sizeof(ufs_inode_t) == 128);
    
    /* Which cylinder group is this inode in? */
    const uint32_t group_num = inum / mount->superblock.ipg;
    
    /* Index of this inode in its cylinder group's inode table */
    const uint32_t group_ino_offset = inum % mount->superblock.ipg;
    
    /* Fragment address that contains inode */
    const uint32_t frag_addr = ufs_group_base(mount, group_num) +
                               mount->superblock.iblkno +
                               ((group_ino_offset * 128) / mount->frag_size);
    
    /* Byte offset into the fragment where the inode begins */
    const uint32_t frag_offset = (group_ino_offset * 128) % mount->frag_size;
    
    uint32_t i;
    uint8_t *buf = p_alloc(mount->pool, mount->frag_size);
    
    // slog("group_num = %u, ino_offset=%u, addr = 0x%08x, offset = 0x%08x\n", group_num, group_ino_offset, frag_addr, frag_offset);
    // slog("mount->superblock.iblkno = 0x%08x\n", mount->superblock.iblkno);
    
    if (!ufs_read_frag(mount, buf, frag_addr))
        goto fail;
    
    memcpy(inode, buf + frag_offset, 128);
    
    fix_endian(inode->mode);
    fix_endian(inode->nlink);
    fix_endian(inode->uid);
    fix_endian(inode->gid);
    fix_endian(inode->size_hi);
    fix_endian(inode->size);
    fix_endian(inode->atime);
    fix_endian(inode->mtime);
    fix_endian(inode->ctime);
    for (i=0; i<12; i++)
        fix_endian(inode->direct[i]);
    for (i=0; i<3; i++)
        fix_endian(inode->indirect[i]);
    fix_endian(inode->flags);
    fix_endian(inode->blocks);
    fix_endian(inode->gen);
    
    p_free(buf);
    return 1;
fail:
    if (buf)
        p_free(buf);
    return 0;
}

static uint8_t ufs_read_level(ufs_t *mount,
                              ufs_inode_t *inode,
                              uint8_t *buf,
                              uint64_t *len,
                              uint32_t indirect_blockno,
                              uint32_t level)
{
    if (inode->size <= *len)
        return 1;
    
    uint32_t *table = p_alloc(mount->pool, mount->block_size);
    uint8_t *block = NULL;
    
    const uint32_t num_pointers = mount->block_size / 4;
    uint32_t i;
    
    if (!ufs_read_block(mount, (uint8_t*)table, indirect_blockno))
        goto fail;
    
    // for (i=0; i<num_pointers; i++)
        // slog("%u 0x%08x\n", i, ntohl(table[i]));
    
    if (level == 1)
        block = p_alloc(mount->pool, mount->block_size);
    
    for (i=0; (i < num_pointers) && (inode->size > *len); i++) {
        const uint32_t blockno = ntohl(table[i]);
        
        if (level == 1) {
            // direct block
            uint64_t chunk_size = inode->size - *len;
            if (chunk_size > mount->block_size) chunk_size = mount->block_size;
            
            // Which block are we reading, and at which byte-offset into it does our data exist
            const uint32_t block_addr = (blockno / mount->frag_per_block) * mount->frag_per_block;
            const uint32_t block_offset = (blockno - block_addr) * mount->frag_size;
            
            slog("L%u: raw_blkno=0x%08x len=0x%08x blockno:0x%08x chunk_size=0x%08x\n", level-1, blockno, (uint32_t)*len, block_addr, (uint32_t)chunk_size);
            
            // If the chunk_size is a whole block, then we better be reading in a whole block
            if (chunk_size == mount->block_size) {
                // slog("block_offset = 0x%x\n", block_offset);
                assert(block_offset == 0);
            }
            
            if (!ufs_read_block(mount, block, block_addr))
                goto fail;
            
            memcpy(buf + *len, block + block_offset, chunk_size);
            (*len) += chunk_size;
        }
        else {
            // indirect block
            if (!ufs_read_level(mount, inode, buf, len, blockno, level-1))
                goto fail;
        }
    }
    
    if (block)
        p_free(block);
    p_free(table);
    return 1;
fail:
    if (block)
        p_free(block);
    p_free(table);
    return 0;
}

static uint8_t* ufs_read_inode_data(ufs_t *mount, ufs_inode_t *inode)
{
    uint32_t i, j;
    uint8_t *block = p_alloc(mount->pool, mount->block_size);
    uint8_t *buf = p_alloc(mount->pool, inode->size);
    uint64_t len = 0;
    
    /* Read in direct blocks */
    for (i=0; (i<12) && (len < inode->size); i++) {
        
        // How many bytes are we reading from this block?
        uint64_t chunk_size = inode->size - len;
        if (chunk_size > mount->block_size) chunk_size = mount->block_size;
        
        // Which block are we reading, and at which byte-offset into it does our data exist
        const uint32_t block_addr = (inode->direct[i] / mount->frag_per_block) * mount->frag_per_block;
        const uint32_t block_offset = (inode->direct[i] - block_addr) * mount->frag_size;
        
        // slog("block_addr=0x%08x, block_offset")
        
        // If the chunk_size is a whole block, then we better be reading in a whole block
        if (chunk_size == mount->block_size)
            assert(block_offset == 0);
        
        if (!ufs_read_block(mount, block, block_addr))
            goto fail;
        
        memcpy(buf + len, block + block_offset, chunk_size);

        len += chunk_size;
        // slog("direct block %u = 0x%08x\n", i, inode->direct[i]);
    }


    for (i=0; i<3; i++) {
        if (!ufs_read_level(mount, inode, buf, &len, inode->indirect[i], i+1))
            goto fail;
    }
    
    
    p_free(block);
    return buf;
fail:
    p_free(block);
    p_free(buf);
    return NULL;
}

typedef struct __attribute__ ((__packed__)) {
    uint32_t inum;
    uint16_t len;
    uint16_t namelen;
    char name[1];
} ufs_dir_t;

ufs_inode_t* ufs_traverse_path(ufs_t *mount, const char *_path)
{
    uint32_t i;
    uint32_t inum = 2; // 2 == root
    ufs_inode_t *inode = p_alloc(mount->pool, sizeof(ufs_inode_t));
    char *path = p_alloc(mount->pool, strlen(_path)+1);
    strcpy(path, _path);
    
    if (!ufs_load_inode(mount, inode, inum))
        goto fail;
    
    char *last, *elem;
    for (elem = strtok_r(path, "/", &last);
         elem;
         elem = strtok_r(NULL, "/", &last)) {
        
        uint32_t next_inum = 0;
        uint8_t *dir = ufs_read_inode_data(mount, inode);
        if (!dir)
            goto fail;
        
        for (i=0; inode->size; ) {
            ufs_dir_t *entry = (ufs_dir_t*)&dir[i];
            fix_endian(entry->inum);
            fix_endian(entry->len);
            fix_endian(entry->namelen);
            
            if (entry->inum == 0)
                break;
            
            if ((entry->namelen == strlen(elem)) &&
                (strncmp(elem, entry->name, entry->namelen) == 0)) {
                next_inum = entry->inum;
                break;
            }
            
            i += entry->len;
        }
        
        p_free(dir);
        
        if (next_inum == 0) {
            sprintf(mount->error_str, "'%s' in '%s' doesn't exist", elem, _path);
            goto fail;
        }
        
        inum = next_inum;
        if (!ufs_load_inode(mount, inode, inum))
            goto fail;
    }
    
    p_free(path);
    return inode;
    
fail:
    p_free(inode);
    p_free(path);
    return NULL;
}

static ufs_t* ufs_mount(partition_t *part)
{
    ufs_t *mount = p_alloc(part->pool, sizeof(ufs_t));
    uint8_t *buf = p_alloc(part->pool, 32 * 512);
    uint32_t i;
    
    mount->pool = part->pool;
    mount->part = part;
    mount->error_str = part->error_str;
    
    for (i=0; i<4; i++)
        part_get_block(part, &buf[i*512], 16 + i);
    memcpy(&mount->superblock, buf, sizeof(ufs_superblock_t));
    
    fix_endian(mount->superblock.link);
    fix_endian(mount->superblock.rlink);
    fix_endian(mount->superblock.sblkno);
    fix_endian(mount->superblock.cblkno);
    fix_endian(mount->superblock.iblkno);
    fix_endian(mount->superblock.dblkno);
    fix_endian(mount->superblock.cgoffset);
    fix_endian(mount->superblock.cgmask);
    fix_endian(mount->superblock.time);
    fix_endian(mount->superblock.size);
    fix_endian(mount->superblock.dsize);
    fix_endian(mount->superblock.ncg);
    fix_endian(mount->superblock.bsize);
    fix_endian(mount->superblock.fsize);
    fix_endian(mount->superblock.frag);
    fix_endian(mount->superblock.minfree);
    fix_endian(mount->superblock.rotdelay);
    fix_endian(mount->superblock.rps);
    fix_endian(mount->superblock.bmask);
    fix_endian(mount->superblock.fmask);
    fix_endian(mount->superblock.bshift);
    fix_endian(mount->superblock.fshift);
    fix_endian(mount->superblock.maxcontig);
    fix_endian(mount->superblock.maxbpg);
    fix_endian(mount->superblock.fragshift);
    fix_endian(mount->superblock.fsbtodb);
    fix_endian(mount->superblock.sbsize);
    fix_endian(mount->superblock.csmask);
    fix_endian(mount->superblock.csshift);
    fix_endian(mount->superblock.nindir);
    fix_endian(mount->superblock.inopb);
    fix_endian(mount->superblock.nspf);
    fix_endian(mount->superblock.optim);
    fix_endian(mount->superblock.state);
    fix_endian(mount->superblock.id[0]);
    fix_endian(mount->superblock.id[1]);
    fix_endian(mount->superblock.csaddr);
    fix_endian(mount->superblock.cssize);
    fix_endian(mount->superblock.cgsize);
    fix_endian(mount->superblock.ntrak);
    fix_endian(mount->superblock.nsect);
    fix_endian(mount->superblock.spc);
    fix_endian(mount->superblock.ncyl);
    fix_endian(mount->superblock.cpg);
    fix_endian(mount->superblock.ipg);
    fix_endian(mount->superblock.fpg);
    fix_endian(mount->superblock.csum_ndir);
    fix_endian(mount->superblock.csum_nbfree);
    fix_endian(mount->superblock.csum_nifree);
    fix_endian(mount->superblock.csum_nffree);
    fix_endian(mount->superblock.cgrotor);
    fix_endian(mount->superblock.cpc);
    for (i=0; i<(32*8); i++)
        fix_endian(mount->superblock.postbl[i/8][i%8]);
    fix_endian(mount->superblock.magic);
    
    
    if (mount->superblock.magic != 0x00011954) {
        sprintf(part->error_str, "Magic doesn't match ufs");
        goto fail;
    }
    
    // It is UFS!
    
    mount->frag_size = mount->superblock.fsize;
    mount->frag_per_block = mount->superblock.frag;
    mount->block_size = mount->frag_size * mount->frag_per_block;
    assert(mount->block_size == mount->superblock.bsize);
    
    mount->num_groups = mount->superblock.ncg;
    mount->groups = (ufs_cylinder_group_t*)p_alloc(mount->pool,
                                                   mount->num_groups * sizeof(ufs_cylinder_group_t));
    
    for (i=0; i<mount->num_groups; i++) {
        uint32_t group_base = ufs_group_base(mount, i) + mount->superblock.cblkno;
        ufs_load_cylinder_group(mount, group_base, &mount->groups[i]);
        if ((mount->groups[i].cgx != i) || (mount->groups[i].magic != 0x00090255)) {
            sprintf(mount->error_str, "bad cylinder group %u frag_offset=0x%x", i, group_base);
            goto fail;
        }
    }
    
    if (buf)
        p_free(buf);
    return mount;
fail:
    if (mount) {
        if (mount->groups)
            p_free(mount->groups);
        p_free(mount);
    }
    if (buf)
        p_free(buf);
    return NULL;
}



/* --- Public interfaces --- */
#pragma mark Public interfaces


uint8_t* shoebill_extract_kernel(const char *disk_path, const char *kernel_path, char *error_str, uint32_t *len)
{
    uint8_t *pool_data, *kernel_data = NULL;
    disk_t *disk;
    svfs_t *svfs_mount_obj;
    ufs_t *ufs_mount_obj;
    int32_t apm_part_num;
    
    strcpy(error_str, "");
    
    disk = open_disk(disk_path, error_str);
    if (!disk)
        goto done;
    
    apm_part_num = find_root_partition_number(disk, 0);
    if (apm_part_num == -1) {
        sprintf(error_str, "Couldn't find root partition");
        goto done;
    }
    slog("apm_part_num = %u\n", apm_part_num);
    
    svfs_mount_obj = svfs_mount(&disk->partitions[apm_part_num]);
    if (svfs_mount_obj) {
        svfs_inode_t *inode = svfs_traverse_path(svfs_mount_obj, kernel_path);
        if (!inode)
            goto done;
        
        pool_data = svfs_read_inode_data(svfs_mount_obj, inode);
        if (!pool_data)
            goto done;
        
        kernel_data = malloc(inode->size);
        memcpy(kernel_data, pool_data, inode->size);
        *len = inode->size;
        goto done;
    }
    
    ufs_mount_obj = ufs_mount(&disk->partitions[apm_part_num]);
    if (ufs_mount_obj) {
        ufs_inode_t *inode = ufs_traverse_path(ufs_mount_obj, kernel_path);
        if (!inode)
            goto done;
        
        pool_data = ufs_read_inode_data(ufs_mount_obj, inode);
        if (!pool_data)
            goto done;
        
        kernel_data = malloc(inode->size);
        memcpy(kernel_data, pool_data, inode->size);
        *len = inode->size;
        goto done;
    }
    
    sprintf(error_str, "I can read the partition map, but the filesystem doesn't seem to be UFS or SVFS");
    
done:
    if (strlen(error_str))
        slog("error: [%s]\n", error_str);
    if (disk)
        close_disk(disk);
    return kernel_data;
}

/*int main (int argc, char **argv)
{
    uint8_t *buf;
    uint32_t size;
    char error_str[1024];
    
    buf = shoebill_extract_kernel(argv[1], argv[2], error_str, &size);
    if (!buf)
        return 0;
    
    FILE *f = fopen("result", "wb");
    fwrite(buf, size, 1, f);
    fclose(f);
    
    return 0;
}*/



