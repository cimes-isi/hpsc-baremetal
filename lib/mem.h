#ifndef MEM_H
#define MEM_H

#include <string.h>

void bzero(void *p, size_t size);

volatile void *vmem_set(volatile void *s, int c, unsigned n);

volatile void *vmem_cpy(volatile void *restrict dest, void *restrict src,
                        unsigned n);

void *mem_vcpy(void *restrict dest, volatile void *restrict src, unsigned n);

#endif // MEM_H
