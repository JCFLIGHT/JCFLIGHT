#include "SDFAT.h"

uint32_t SdVolume::cacheBlockNumber_ = 0XFFFFFFFF;
cache_t SdVolume::cacheBuffer_;
Sd2Card *SdVolume::sdCard_;
uint8_t SdVolume::cacheDirty_ = 0;
uint32_t SdVolume::cacheMirrorBlock_ = 0;

uint8_t SdVolume::allocContiguous(uint32_t count, uint32_t *curCluster)
{
  uint32_t bgnCluster;
  uint8_t setStart;
  if (*curCluster)
  {
    bgnCluster = *curCluster + 1;
    setStart = false;
  }
  else
  {
    bgnCluster = allocSearchStart_;
    setStart = 1 == count;
  }
  uint32_t endCluster = bgnCluster;
  uint32_t fatEnd = clusterCount_ + 1;
  for (uint32_t n = 0;; n++, endCluster++)
  {
    if (n >= clusterCount_)
      return false;
    if (endCluster > fatEnd)
      bgnCluster = endCluster = 2;
    uint32_t f;
    if (!fatGet(endCluster, &f))
      return false;
    if (f != 0)
      bgnCluster = endCluster + 1;
    else if ((endCluster - bgnCluster + 1) == count)
      break;
  }
  if (!fatPutEOC(endCluster))
    return false;
  while (endCluster > bgnCluster)
  {
    if (!fatPut(endCluster - 1, endCluster))
      return false;
    endCluster--;
  }
  if (*curCluster != 0)
  {
    if (!fatPut(*curCluster, bgnCluster))
      return false;
  }
  *curCluster = bgnCluster;
  if (setStart)
    allocSearchStart_ = bgnCluster + 1;
  return true;
}

uint8_t SdVolume::cacheFlush(uint8_t blocking)
{
  if (cacheDirty_)
  {
    if (!sdCard_->writeBlock(cacheBlockNumber_, cacheBuffer_.data, blocking))
      return false;
    if (!blocking)
      return true;
    if (!cacheMirrorBlockFlush(blocking))
      return false;
    cacheDirty_ = 0;
  }
  return true;
}

uint8_t SdVolume::cacheMirrorBlockFlush(uint8_t blocking)
{
  if (cacheMirrorBlock_)
  {
    if (!sdCard_->writeBlock(cacheMirrorBlock_, cacheBuffer_.data, blocking))
      return false;
    cacheMirrorBlock_ = 0;
  }
  return true;
}

uint8_t SdVolume::cacheRawBlock(uint32_t blockNumber, uint8_t action)
{
  if (cacheBlockNumber_ != blockNumber)
  {
    if (!cacheFlush())
      return false;
    if (!sdCard_->readBlock(blockNumber, cacheBuffer_.data))
      return false;
    cacheBlockNumber_ = blockNumber;
  }
  cacheDirty_ |= action;
  return true;
}

uint8_t SdVolume::cacheZeroBlock(uint32_t blockNumber)
{
  if (!cacheFlush())
    return false;
  for (uint16_t i = 0; i < 512; i++)
    cacheBuffer_.data[i] = 0;
  cacheBlockNumber_ = blockNumber;
  cacheSetDirty();
  return true;
}

uint8_t SdVolume::chainSize(uint32_t cluster, uint32_t *size) const
{
  uint32_t s = 0;
  do
  {
    if (!fatGet(cluster, &cluster))
      return false;
    s += 512UL << clusterSizeShift_;
  } while (!isEOC(cluster));
  *size = s;
  return true;
}

uint8_t SdVolume::fatGet(uint32_t cluster, uint32_t *value) const
{
  if (cluster > (clusterCount_ + 1))
    return false;
  uint32_t lba = fatStartBlock_;
  lba += fatType_ == 16 ? cluster >> 8 : cluster >> 7;
  if (lba != cacheBlockNumber_)
    if (!cacheRawBlock(lba, CACHE_FOR_READ))
      return false;
  if (fatType_ == 16)
    *value = cacheBuffer_.fat16[cluster & 0XFF];
  else
    *value = cacheBuffer_.fat32[cluster & 0X7F] & FAT32MASK;
  return true;
}

uint8_t SdVolume::fatPut(uint32_t cluster, uint32_t value)
{
  if (cluster < 2)
    return false;
  if (cluster > (clusterCount_ + 1))
    return false;
  uint32_t lba = fatStartBlock_;
  lba += fatType_ == 16 ? cluster >> 8 : cluster >> 7;
  if (lba != cacheBlockNumber_)
    if (!cacheRawBlock(lba, CACHE_FOR_READ))
      return false;
  if (fatType_ == 16)
    cacheBuffer_.fat16[cluster & 0XFF] = value;
  else
    cacheBuffer_.fat32[cluster & 0X7F] = value;
  cacheSetDirty();
  if (fatCount_ > 1)
    cacheMirrorBlock_ = lba + blocksPerFat_;
  return true;
}

uint8_t SdVolume::freeChain(uint32_t cluster)
{
  allocSearchStart_ = 2;
  do
  {
    uint32_t next;
    if (!fatGet(cluster, &next))
      return false;
    if (!fatPut(cluster, 0))
      return false;
    cluster = next;
  } while (!isEOC(cluster));
  return true;
}

uint8_t SdVolume::init(Sd2Card *dev, uint8_t part)
{
  uint32_t volumeStartBlock = 0;
  sdCard_ = dev;
  if (part)
  {
    if (part > 4)
      return false;
    if (!cacheRawBlock(volumeStartBlock, CACHE_FOR_READ))
      return false;
    part_t *p = &cacheBuffer_.mbr.part[part - 1];
    if ((p->boot & 0X7F) != 0 || p->totalSectors < 100 || p->firstSector == 0)
      return false;
    volumeStartBlock = p->firstSector;
  }
  if (!cacheRawBlock(volumeStartBlock, CACHE_FOR_READ))
    return false;
  bpb_t *bpb = &cacheBuffer_.fbs.bpb;
  if (bpb->bytesPerSector != 512 || bpb->fatCount == 0 || bpb->reservedSectorCount == 0 || bpb->sectorsPerCluster == 0)
    return false;
  fatCount_ = bpb->fatCount;
  blocksPerCluster_ = bpb->sectorsPerCluster;
  clusterSizeShift_ = 0;
  while (blocksPerCluster_ != (1 << clusterSizeShift_))
  {
    if (clusterSizeShift_++ > 7)
      return false;
  }
  blocksPerFat_ = bpb->sectorsPerFat16 ? bpb->sectorsPerFat16 : bpb->sectorsPerFat32;
  fatStartBlock_ = volumeStartBlock + bpb->reservedSectorCount;
  rootDirEntryCount_ = bpb->rootDirEntryCount;
  rootDirStart_ = fatStartBlock_ + bpb->fatCount * blocksPerFat_;
  dataStartBlock_ = rootDirStart_ + ((32 * bpb->rootDirEntryCount + 511) / 512);
  uint32_t totalBlocks = bpb->totalSectors16 ? bpb->totalSectors16 : bpb->totalSectors32;
  clusterCount_ = totalBlocks - (dataStartBlock_ - volumeStartBlock);
  clusterCount_ >>= clusterSizeShift_;
  if (clusterCount_ < 4085)
    fatType_ = 12;
  else if (clusterCount_ < 65525)
    fatType_ = 16;
  else
  {
    rootDirStart_ = bpb->fat32RootCluster;
    fatType_ = 32;
  }
  return true;
}
