/*
 * static_allocator.h
 *
 *  Created on: Mar 29, 2020
 *      Author: javi
 *
 *  This memory allocator:
 *  - Does NOT free memory
 *  - Is not thread safe
 */

#ifndef MEMORY_STATIC_ALLOCATOR_H_
#define MEMORY_STATIC_ALLOCATOR_H_


/* Includes */
#include "main.h"


/* Public functions */

/**
 * Initializes static allocator module
 */
void stalloc_Init(void);

/**
 * Disables static allocator module
 */
void stalloc_Disable(void);

/**
 * Allocates memory that is word (4 bytes) aligned
 * @param sizeBytes, amount of memory to allocate [bytes]
 * @return pointer to allocated memory
 */
void *stalloc_AllocAlignWord(uint32_t sizeBytes);

#endif /* MEMORY_STATIC_ALLOCATOR_H_ */
