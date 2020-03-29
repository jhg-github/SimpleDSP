/*
 * static_allocator.c
 *
 *  Created on: Mar 29, 2020
 *      Author: javi
 *
 *  This memory allocator:
 *  - Does NOT free memory
 *  - Is not thread safe
 */


/* Includes */
#include "static_allocator.h"


/* Private defines */
#define STALLOC_BUFFER_SIZE_WORDS (5000)
#define STALLOC_BUFFER_SIZE_BYTES (STALLOC_BUFFER_SIZE_WORDS*4)


/* Private variables */
static struct stalloc_mod_tag { // static allocator module structure
  uint8_t buffer[STALLOC_BUFFER_SIZE_BYTES];  // buffer where to allocate memory
  bool isEnabled;                             // flag to mark that it is still possible to allocate memory
  uint32_t nextFree;                          // position of the next free memory
} stalloc_mod;


/* Public functions */

/**
 * Initializes static allocator module
 */
void stalloc_Init(void){
  stalloc_mod.isEnabled = true;
}

/**
 * Disables static allocator module
 */
void stalloc_Disable(void){
  stalloc_mod.isEnabled = false;
}

/**
 * Allocates memory that is word (4 bytes) aligned
 * @param sizeBytes, amount of memory to allocate [bytes]
 * @return pointer to allocated memory
 */
void *stalloc_AllocAlignWord(uint32_t sizeBytes){
   void *nextBlock;
   uint32_t sizeBytesWordAligned = ((sizeBytes + 3) / 4) * 4;
   if(!stalloc_mod.isEnabled || (sizeBytes == 0) || ((stalloc_mod.nextFree + sizeBytesWordAligned) > STALLOC_BUFFER_SIZE_BYTES)){
     nextBlock = NULL;
   } else {
     nextBlock = &stalloc_mod.buffer[stalloc_mod.nextFree];
     stalloc_mod.nextFree += sizeBytesWordAligned;
   }
   return nextBlock;
}


