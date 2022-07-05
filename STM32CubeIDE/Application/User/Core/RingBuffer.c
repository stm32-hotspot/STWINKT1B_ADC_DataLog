#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "RingBuffer.h"

void RingBuffer_reset(Ring_Buffer_t *hRingBuff)
{
  memset(hRingBuff, 0, sizeof(Ring_Buffer_t));
  hRingBuff->pData = NULL;
}

int32_t RingBuffer_alloc(Ring_Buffer_t *hRingBuff)
{
  hRingBuff->pData = malloc((size_t)hRingBuff->BytesPerSample * hRingBuff->MaxNumSamples);

  // Allocation issue
  if(hRingBuff->pData == NULL)
  {
    return RB_ERR_ALLOC;
  }

  return RB_ERR_OK;
}

void RingBuffer_free(Ring_Buffer_t *hRingBuff)
{
  free(hRingBuff->pData);
}

int32_t  RingBuffer_feed(Ring_Buffer_t *hRingBuff, uint8_t *pData, int32_t numSamples)
{
  if( (pData != NULL) && (hRingBuff->pData != NULL) )
  {
    if ( (hRingBuff->availableSamples + numSamples) >  hRingBuff->MaxNumSamples )
    {
      return RB_ERR_OVERFLOW; // Overflow error
    }
    else
    {
      /* Feed new data into ring buffer */
      uint8_t * pDest = hRingBuff->pData + (hRingBuff->BytesPerSample * hRingBuff->writeIdx);
      memcpy(pDest, pData, (uint32_t)hRingBuff->BytesPerSample * numSamples);
      hRingBuff->writeIdx = (hRingBuff->writeIdx + numSamples) % hRingBuff->MaxNumSamples;
      hRingBuff->availableSamples += numSamples;
    }

  }
  else
  {
    return RB_ERR_ALLOC; // Unallocated buffer error
  }

  return RB_ERR_OK; // No error
}

uint8_t *RingBuffer_consume(Ring_Buffer_t *hRingBuff, uint32_t numSamples)
{
  uint8_t *pDest = NULL;

  /* Only perform the consume if there is enough available samples to consume, else return NULL */
  if (hRingBuff->availableSamples >= numSamples)
  {
    pDest = hRingBuff->pData + (hRingBuff->BytesPerSample * hRingBuff->readIdx);
    hRingBuff->readIdx = (hRingBuff->readIdx + numSamples) % hRingBuff->MaxNumSamples;
    hRingBuff->availableSamples -= numSamples;
  }

  return pDest;
}
