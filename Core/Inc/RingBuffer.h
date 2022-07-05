#ifndef RINGBUFFER_H
#define RINGBUFFER_H


/* Error Types */
enum
{
  RB_ERR_OK,
  RB_ERR_OVERFLOW,
  RB_ERR_ALLOC
};

typedef struct
{
  uint8_t      *pData;                // Ring Buffer
  uint8_t       BytesPerSample;       // Size of each sample in ring buffer in bytes
  uint32_t      MaxNumSamples;        // Maximum number of samples in ring buffer
  uint32_t      readIdx;              // read index of ring buffer. Updated using consume function
  uint32_t      writeIdx;             // write index of ring buffer. Updated using feed function
  uint32_t      availableSamples;     // Current number of samples in ring buffer
} Ring_Buffer_t;


/*
 * Sets all struct memebers to zero and ring buff pointer to NULL
 */
void      RingBuffer_reset(Ring_Buffer_t *hRingBuff);

/*
 * Allocates ring buffer memory. Returns error code
 */
int32_t  RingBuffer_alloc(Ring_Buffer_t *hRingBuff);

/*
 * Frees memory associated with Ring Buffer
 */
void      RingBuffer_free(Ring_Buffer_t *hRingBuff);

/*
 * Adds new data to ring buffer. Returns error code
 */
int32_t  RingBuffer_feed(Ring_Buffer_t *hRingBuff, uint8_t *pData, int32_t numSamples);

/*
 * Read/Consume data from ring buffer. Return error code.
 */
uint8_t * RingBuffer_consume(Ring_Buffer_t *hRingBuff, uint32_t numSamples);


#endif // RINGBUFFER_H
