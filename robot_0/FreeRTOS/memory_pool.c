#include "memory_pool.h"

MemPool_t system_mem_pool;

void MemPool_Init(MemPool_t *pool) {
    uint32_t i;
    
    for (i = 0; i < MEM_POOL_BLOCK_COUNT - 1; i++) {
        pool->blocks[i].next = &pool->blocks[i + 1];
    }
    pool->blocks[MEM_POOL_BLOCK_COUNT - 1].next = (MemBlock_t*)0;
    pool->free_list = &pool->blocks[0];
    pool->initialized = 1;
}

void* MemPool_Alloc(MemPool_t *pool) {
    void *ptr = (void*)0;
    
    if (pool->free_list != (MemBlock_t*)0) {
        ptr = (void*)pool->free_list;
        pool->free_list = pool->free_list->next;
    }
    
    if (ptr != (void*)0) {
        volatile uint32_t *p = (volatile uint32_t *)ptr;
        for (uint32_t i = 0; i < MEM_POOL_BLOCK_SIZE / 4; i++) {
            p[i] = 0xDEADBEEF;
        }
    }
    
    return ptr;
}

void MemPool_Free(MemPool_t *pool, void *ptr) {
    if (ptr == (void*)0) return;
    
    MemBlock_t *block = (MemBlock_t *)ptr;
    block->next = pool->free_list;
    pool->free_list = block;
    
    volatile uint32_t *p = (volatile uint32_t *)ptr;
    for (uint32_t i = 0; i < MEM_POOL_BLOCK_SIZE / 4; i++) {
        p[i] = 0xBAADF00D;
    }
}

uint32_t MemPool_GetFreeBlocks(MemPool_t *pool) {
    uint32_t count = 0;
    MemBlock_t *current = pool->free_list;
    while (current != (MemBlock_t*)0) {
        count++;
        current = current->next;
    }
    return count;
}

uint32_t MemPool_GetUsedBlocks(MemPool_t *pool) {
    return MEM_POOL_BLOCK_COUNT - MemPool_GetFreeBlocks(pool);
}
