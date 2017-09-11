/*
 * Demo module for allocation of contiguous dmable memory, user space API
 *
 * Author Davide Ciminaghi <ciminaghi@gnudd.com> Sept 2017
 */
#ifndef __UAPI_DMA_ALLOCATION_TEST_H__
#define __UAPI_DMA_ALLOCATION_TEST_H__

#define DMA_ALLOC_TEST_ALLOC _IOWR('T', 1, __u32 *)
#define DMA_ALLOC_TEST_FREE  _IOWR('T', 2, __u32)



#endif /* __UAPI_DMA_ALLOCATION_TEST_H__ */

