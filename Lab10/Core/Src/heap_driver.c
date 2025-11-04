#include "heap_driver.h"
#include "main.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uintptr_t)0x20001000)
#define HEAP_SIZE         4096
#define BLOCK_SIZE        16
#define BLOCK_COUNT       (HEAP_SIZE / BLOCK_SIZE)

static uint8_t block_map[BLOCK_COUNT];  // 0 = free, 1 = used
static uint8_t *heap_base = (uint8_t *)HEAP_START_ADDR;

void heap_init(void)
{
    memset(block_map, 0, sizeof(block_map));  // Mark all blocks as free
}

void *heap_alloc(size_t size)
{
    if (size == 0 || size > HEAP_SIZE)
        return NULL;

    size_t needed_blocks = (size + BLOCK_SIZE - 1) / BLOCK_SIZE; // round up

    // Scan for contiguous free blocks
    size_t free_count = 0;
    size_t start_index = 0;

    for (size_t i = 0; i < BLOCK_COUNT; i++)
    {
        if (block_map[i] == 0)
        {
            if (free_count == 0)
                start_index = i;

            free_count++;
            if (free_count == needed_blocks)
            {
                // Found enough free blocks
                for (size_t j = start_index; j < start_index + needed_blocks; j++)
                    block_map[j] = 1; // mark as used

                return (void *)(heap_base + (start_index * BLOCK_SIZE));
            }
        }
        else
        {
            free_count = 0;
        }
    }

    return NULL; // no space
}

void heap_free(void *ptr)
{
    if (ptr == NULL)
        return;

    uintptr_t addr = (uintptr_t)ptr;

    // Check if address is inside the heap range
    if (addr < HEAP_START_ADDR || addr >= HEAP_START_ADDR + HEAP_SIZE)
        return;

    size_t block_index = (addr - HEAP_START_ADDR) / BLOCK_SIZE;

    // Free blocks until a free one is found or heap end is reached
    for (size_t i = block_index; i < BLOCK_COUNT; i++)
    {
        if (block_map[i] == 1)
            block_map[i] = 0;
        else
            break;
    }
}
