#ifndef SHMEM_H
#define SHMEM_H

#include <unistd.h>

// size aligns with mailbox messages
#define SHMEM_MSG_SIZE 64

struct shmem;

struct shmem *shmem_open(void *addr);

void shmem_close(struct shmem *s);

// returns the number of bytes written
size_t shmem_send(struct shmem *s, void *msg, size_t sz);

uint32_t shmem_get_status(struct shmem *s);

// returns 0the number of bytes read
size_t shmem_recv(struct shmem *s, void *msg, size_t sz);

#endif // SHMEM_H
