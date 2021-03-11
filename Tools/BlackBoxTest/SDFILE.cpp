#include "SDFAT.h"

void (*SdFile::dateTime_)(uint16_t *date, uint16_t *time) = NULL;
void (*SdFile::oldDateTime_)(uint16_t &date, uint16_t &time) = NULL;

uint8_t SdFile::addCluster()
{
  if (!vol_->allocContiguous(1, &curCluster_))
    return false;
  if (firstCluster_ == 0)
  {
    firstCluster_ = curCluster_;
    flags_ |= F_FILE_DIR_DIRTY;
  }
  flags_ |= F_FILE_CLUSTER_ADDED;
  return true;
}

uint8_t SdFile::addDirCluster(void)
{
  if (!addCluster())
    return false;
  uint32_t block = vol_->clusterStartBlock(curCluster_);
  for (uint8_t i = vol_->blocksPerCluster_; i != 0; i--)
    if (!SdVolume::cacheZeroBlock(block + i - 1))
      return false;
  fileSize_ += 512UL << vol_->clusterSizeShift_;
  return true;
}

dir_t *SdFile::cacheDirEntry(uint8_t action)
{
  if (!SdVolume::cacheRawBlock(dirBlock_, action))
    return NULL;
  return SdVolume::cacheBuffer_.dir + dirIndex_;
}

uint8_t SdFile::close(void)
{
  if (!sync())
    return false;
  type_ = FAT_FILE_TYPE_CLOSED;
  return true;
}

uint8_t SdFile::contiguousRange(uint32_t *bgnBlock, uint32_t *endBlock)
{
  if (firstCluster_ == 0)
    return false;
  for (uint32_t c = firstCluster_;; c++)
  {
    uint32_t next;
    if (!vol_->fatGet(c, &next))
      return false;
    if (next != (c + 1))
    {
      if (!vol_->isEOC(next))
        return false;
      *bgnBlock = vol_->clusterStartBlock(firstCluster_);
      *endBlock = vol_->clusterStartBlock(c) + vol_->blocksPerCluster_ - 1;
      return true;
    }
  }
}

uint8_t SdFile::createContiguous(SdFile *dirFile, const char *fileName, uint32_t size)
{
  if (size == 0)
    return false;
  if (!open(dirFile, fileName, O_CREAT | O_EXCL | O_RDWR))
    return false;
  uint32_t count = ((size - 1) >> (vol_->clusterSizeShift_ + 9)) + 1;
  if (!vol_->allocContiguous(count, &firstCluster_))
  {
    remove();
    return false;
  }
  fileSize_ = size;
  flags_ |= F_FILE_DIR_DIRTY;
  return sync();
}

uint8_t SdFile::dirEntry(dir_t *dir)
{
  if (!sync())
    return false;
  dir_t *p = cacheDirEntry(SdVolume::CACHE_FOR_READ);
  if (!p)
    return false;
  memcpy(dir, p, sizeof(dir_t));
  return true;
}

void SdFile::dirName(const dir_t &dir, char *name)
{
  uint8_t j = 0;
  for (uint8_t i = 0; i < 11; i++)
  {
    if (dir.name[i] == ' ')
      continue;
    if (i == 8)
      name[j++] = '.';
    name[j++] = dir.name[i];
  }
  name[j] = 0;
}

void SdFile::ls(uint8_t flags, uint8_t indent)
{
  dir_t *p;
  rewind();
  while ((p = readDirCache()))
  {
    if (p->name[0] == DIR_NAME_FREE)
      break;
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.')
      continue;
    if (!DIR_IS_FILE_OR_SUBDIR(p))
      continue;
    for (int8_t i = 0; i < indent; i++)
    {
      //Serial.print(' ');
    }
    printDirName(*p, flags & (LS_DATE | LS_SIZE) ? 14 : 0);
    if (flags & LS_DATE)
    {
      printFatDate(p->lastWriteDate);
      //Serial.print(' ');
      printFatTime(p->lastWriteTime);
    }
    if (!DIR_IS_SUBDIR(p) && (flags & LS_SIZE))
    {
      //Serial.print(' ');
      //Serial.print(p->fileSize);
    }
    //Serial.println();
    if ((flags & LS_R) && DIR_IS_SUBDIR(p))
    {
      uint16_t index = curPosition() / 32 - 1;
      SdFile s;
      if (s.open(this, index, O_READ))
        s.ls(flags, indent + 2);
      seekSet(32 * (index + 1));
    }
  }
}

uint8_t SdFile::make83Name(const char *str, uint8_t *name)
{
  uint8_t c;
  uint8_t n = 7;
  uint8_t i = 0;
  while (i < 11)
    name[i++] = ' ';
  i = 0;
  while ((c = *str++) != '\0')
  {
    if (c == '.')
    {
      if (n == 10)
      {
        return false;
      }
      n = 10;
      i = 8;
    }
    else
    {
      uint8_t b;
      PGM_P p = ProgramMemoryString("|<>^+=?/[];,*\"\\");
      while ((b = pgm_read_byte(p++)))
        if (b == c)
        {
          return false;
        }
      if (i > n || c < 0X21 || c > 0X7E)
        return false;
      name[i++] = c < 'a' || c > 'z' ? c : c + ('A' - 'a');
    }
  }
  return name[0] != ' ';
}

uint8_t SdFile::makeDir(SdFile *dir, const char *dirName)
{
  dir_t d;
  if (!open(dir, dirName, O_CREAT | O_EXCL | O_RDWR))
    return false;
  flags_ = O_READ;
  type_ = FAT_FILE_TYPE_SUBDIR;
  if (!addDirCluster())
    return false;
  if (!sync())
    return false;
  dir_t *p = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!p)
    return false;
  p->attributes = DIR_ATT_DIRECTORY;
  memcpy(&d, p, sizeof(d));
  for (uint8_t i = 1; i < 11; i++)
    d.name[i] = ' ';
  d.name[0] = '.';
  uint32_t block = vol_->clusterStartBlock(firstCluster_);
  if (!SdVolume::cacheRawBlock(block, SdVolume::CACHE_FOR_WRITE))
    return false;
  memcpy(&SdVolume::cacheBuffer_.dir[0], &d, sizeof(d));
  d.name[1] = '.';
  if (dir->isRoot())
  {
    d.firstClusterLow = 0;
    d.firstClusterHigh = 0;
  }
  else
  {
    d.firstClusterLow = dir->firstCluster_ & 0XFFFF;
    d.firstClusterHigh = dir->firstCluster_ >> 16;
  }
  memcpy(&SdVolume::cacheBuffer_.dir[1], &d, sizeof(d));
  curPosition_ = 2 * sizeof(d);
  return SdVolume::cacheFlush();
}

uint8_t SdFile::open(SdFile *dirFile, const char *fileName, uint8_t oflag)
{
  uint8_t dname[11];
  dir_t *p;
  if (isOpen())
    return false;
  if (!make83Name(fileName, dname))
    return false;
  vol_ = dirFile->vol_;
  dirFile->rewind();
  uint8_t emptyFound = false;
  while (dirFile->curPosition_ < dirFile->fileSize_)
  {
    uint8_t index = 0XF & (dirFile->curPosition_ >> 5);
    p = dirFile->readDirCache();
    if (p == NULL)
      return false;
    if (p->name[0] == DIR_NAME_FREE || p->name[0] == DIR_NAME_DELETED)
    {
      if (!emptyFound)
      {
        emptyFound = true;
        dirIndex_ = index;
        dirBlock_ = SdVolume::cacheBlockNumber_;
      }
      if (p->name[0] == DIR_NAME_FREE)
        break;
    }
    else if (!memcmp(dname, p->name, 11))
    {
      if ((oflag & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
        return false;
      return openCachedEntry(0XF & index, oflag);
    }
  }
  if ((oflag & (O_CREAT | O_WRITE)) != (O_CREAT | O_WRITE))
    return false;
  if (emptyFound)
  {
    p = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
    if (!p)
      return false;
  }
  else
  {
    if (dirFile->type_ == FAT_FILE_TYPE_ROOT16)
      return false;
    if (!dirFile->addDirCluster())
      return false;
    dirIndex_ = 0;
    p = SdVolume::cacheBuffer_.dir;
  }
  memset(p, 0, sizeof(dir_t));
  memcpy(p->name, dname, 11);
  if (dateTime_)
    dateTime_(&p->creationDate, &p->creationTime);
  else
  {
    p->creationDate = FAT_DEFAULT_DATE;
    p->creationTime = FAT_DEFAULT_TIME;
  }
  p->lastAccessDate = p->creationDate;
  p->lastWriteDate = p->creationDate;
  p->lastWriteTime = p->creationTime;
  if (!SdVolume::cacheFlush())
    return false;
  return openCachedEntry(dirIndex_, oflag);
}

uint8_t SdFile::open(SdFile *dirFile, uint16_t index, uint8_t oflag)
{
  if (isOpen())
    return false;
  if ((oflag & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
    return false;
  vol_ = dirFile->vol_;
  if (!dirFile->seekSet(32 * index))
    return false;
  dir_t *p = dirFile->readDirCache();
  if (p == NULL)
    return false;
  if (p->name[0] == DIR_NAME_FREE || p->name[0] == DIR_NAME_DELETED || p->name[0] == '.')
    return false;
  return openCachedEntry(index & 0XF, oflag);
}

uint8_t SdFile::openCachedEntry(uint8_t dirIndex, uint8_t oflag)
{
  dir_t *p = SdVolume::cacheBuffer_.dir + dirIndex;
  if (p->attributes & (DIR_ATT_READ_ONLY | DIR_ATT_DIRECTORY))
    if (oflag & (O_WRITE | O_TRUNC))
      return false;
  dirIndex_ = dirIndex;
  dirBlock_ = SdVolume::cacheBlockNumber_;
  firstCluster_ = (uint32_t)p->firstClusterHigh << 16;
  firstCluster_ |= p->firstClusterLow;
  if (DIR_IS_FILE(p))
  {
    fileSize_ = p->fileSize;
    type_ = FAT_FILE_TYPE_NORMAL;
  }
  else if (DIR_IS_SUBDIR(p))
  {
    if (!vol_->chainSize(firstCluster_, &fileSize_))
      return false;
    type_ = FAT_FILE_TYPE_SUBDIR;
  }
  else
    return false;
  flags_ = oflag & (O_ACCMODE | O_SYNC | O_APPEND);
  curCluster_ = 0;
  curPosition_ = 0;
  if (oflag & O_TRUNC)
    return truncate(0);
  return true;
}

uint8_t SdFile::openRoot(SdVolume *vol)
{
  if (isOpen())
    return false;
  if (vol->fatType() == 16)
  {
    type_ = FAT_FILE_TYPE_ROOT16;
    firstCluster_ = 0;
    fileSize_ = 32 * vol->rootDirEntryCount();
  }
  else if (vol->fatType() == 32)
  {
    type_ = FAT_FILE_TYPE_ROOT32;
    firstCluster_ = vol->rootDirStart();
    if (!vol->chainSize(firstCluster_, &fileSize_))
      return false;
  }
  else
    return false;
  vol_ = vol;
  flags_ = O_READ;
  curCluster_ = 0;
  curPosition_ = 0;
  dirBlock_ = 0;
  dirIndex_ = 0;
  return true;
}

void SdFile::printDirName(const dir_t &dir, uint8_t width)
{
  uint8_t w = 0;
  for (uint8_t i = 0; i < 11; i++)
  {
    if (dir.name[i] == ' ')
    {
      continue;
    }
    if (i == 8)
    {
      w++;
    }
    w++;
  }
  if (DIR_IS_SUBDIR(&dir))
  {
    w++;
  }
  while (w < width)
  {
    w++;
  }
}

void SdFile::printFatDate(uint16_t fatDate)
{
  printTwoDigits(FAT_MONTH(fatDate));
  printTwoDigits(FAT_DAY(fatDate));
}

void SdFile::printFatTime(uint16_t fatTime)
{
  printTwoDigits(FAT_HOUR(fatTime));
  printTwoDigits(FAT_MINUTE(fatTime));
  printTwoDigits(FAT_SECOND(fatTime));
}

void SdFile::printTwoDigits(uint8_t v)
{
  char str[3];
  str[0] = '0' + v / 10;
  str[1] = '0' + v % 10;
  str[2] = 0;
}

int16_t SdFile::read(void *buf, uint16_t nbyte)
{
  uint8_t *dst = reinterpret_cast<uint8_t *>(buf);
  if (!isOpen() || !(flags_ & O_READ))
    return -1;
  if (nbyte > (fileSize_ - curPosition_))
    nbyte = fileSize_ - curPosition_;
  uint16_t toRead = nbyte;
  while (toRead > 0)
  {
    uint32_t block;
    uint16_t offset = curPosition_ & 0X1FF;
    if (type_ == FAT_FILE_TYPE_ROOT16)
      block = vol_->rootDirStart() + (curPosition_ >> 9);
    else
    {
      uint8_t blockOfCluster = vol_->blockOfCluster(curPosition_);
      if (offset == 0 && blockOfCluster == 0)
      {
        if (curPosition_ == 0)
          curCluster_ = firstCluster_;
        else
        {
          if (!vol_->fatGet(curCluster_, &curCluster_))
            return -1;
        }
      }
      block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;
    }
    uint16_t n = toRead;
    if (n > (512 - offset))
      n = 512 - offset;
    if ((unbufferedRead() || n == 512) && block != SdVolume::cacheBlockNumber_)
    {
      if (!vol_->readData(block, offset, n, dst))
        return -1;
      dst += n;
    }
    else
    {
      if (!SdVolume::cacheRawBlock(block, SdVolume::CACHE_FOR_READ))
        return -1;
      uint8_t *src = SdVolume::cacheBuffer_.data + offset;
      uint8_t *end = src + n;
      while (src != end)
      {
        *dst++ = *src++;
      }
    }
    curPosition_ += n;
    toRead -= n;
  }
  return nbyte;
}

int8_t SdFile::readDir(dir_t *dir)
{
  int8_t n;
  if (!isDir() || (0X1F & curPosition_))
    return -1;
  while ((n = read(dir, sizeof(dir_t))) == sizeof(dir_t))
  {
    if (dir->name[0] == DIR_NAME_FREE)
      break;
    if (dir->name[0] == DIR_NAME_DELETED || dir->name[0] == '.')
      continue;
    if (DIR_IS_FILE_OR_SUBDIR(dir))
      return n;
  }
  return n < 0 ? -1 : 0;
}

dir_t *SdFile::readDirCache(void)
{
  if (!isDir())
    return NULL;
  uint8_t i = (curPosition_ >> 5) & 0XF;
  if (read() < 0)
    return NULL;
  curPosition_ += 31;
  return (SdVolume::cacheBuffer_.dir + i);
}

uint8_t SdFile::remove(void)
{
  if (!truncate(0))
    return false;
  dir_t *d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d)
    return false;
  d->name[0] = DIR_NAME_DELETED;
  type_ = FAT_FILE_TYPE_CLOSED;
  return SdVolume::cacheFlush();
}

uint8_t SdFile::remove(SdFile *dirFile, const char *fileName)
{
  SdFile file;
  if (!file.open(dirFile, fileName, O_WRITE))
    return false;
  return file.remove();
}

uint8_t SdFile::rmDir(void)
{
  if (!isSubDir())
    return false;
  rewind();
  while (curPosition_ < fileSize_)
  {
    dir_t *p = readDirCache();
    if (p == NULL)
      return false;
    if (p->name[0] == DIR_NAME_FREE)
      break;
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.')
      continue;
    if (DIR_IS_FILE_OR_SUBDIR(p))
      return false;
  }
  type_ = FAT_FILE_TYPE_NORMAL;
  flags_ |= O_WRITE;
  return remove();
}

uint8_t SdFile::rmRfStar(void)
{
  rewind();
  while (curPosition_ < fileSize_)
  {
    SdFile f;
    uint16_t index = curPosition_ / 32;
    dir_t *p = readDirCache();
    if (!p)
      return false;
    if (p->name[0] == DIR_NAME_FREE)
      break;
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.')
      continue;
    if (!DIR_IS_FILE_OR_SUBDIR(p))
      continue;
    if (!f.open(this, index, O_READ))
      return false;
    if (f.isSubDir())
    {
      if (!f.rmRfStar())
        return false;
    }
    else
    {
      f.flags_ |= O_WRITE;
      if (!f.remove())
        return false;
    }
    if (curPosition_ != (32u * (index + 1)))
      if (!seekSet(32u * (index + 1)))
        return false;
  }
  if (isRoot())
    return true;
  return rmDir();
}

uint8_t SdFile::seekSet(uint32_t pos)
{
  if (!isOpen() || pos > fileSize_)
    return false;
  if (type_ == FAT_FILE_TYPE_ROOT16)
  {
    curPosition_ = pos;
    return true;
  }
  if (pos == 0)
  {
    curCluster_ = 0;
    curPosition_ = 0;
    return true;
  }
  uint32_t nCur = (curPosition_ - 1) >> (vol_->clusterSizeShift_ + 9);
  uint32_t nNew = (pos - 1) >> (vol_->clusterSizeShift_ + 9);
  if (nNew < nCur || curPosition_ == 0)
    curCluster_ = firstCluster_;
  else
    nNew -= nCur;
  while (nNew--)
  {
    if (!vol_->fatGet(curCluster_, &curCluster_))
      return false;
  }
  curPosition_ = pos;
  return true;
}

uint8_t SdFile::sync(uint8_t blocking)
{
  if (!isOpen())
    return false;
  if (flags_ & F_FILE_DIR_DIRTY)
  {
    dir_t *d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
    if (!d)
      return false;
    if (!isDir())
      d->fileSize = fileSize_;
    d->firstClusterLow = firstCluster_ & 0XFFFF;
    d->firstClusterHigh = firstCluster_ >> 16;
    if (dateTime_)
    {
      dateTime_(&d->lastWriteDate, &d->lastWriteTime);
      d->lastAccessDate = d->lastWriteDate;
    }
    flags_ &= ~F_FILE_DIR_DIRTY;
  }
  if (!blocking)
    flags_ &= ~F_FILE_NON_BLOCKING_WRITE;
  return SdVolume::cacheFlush(blocking);
}

uint8_t SdFile::timestamp(uint8_t flags, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
  if (!isOpen() || year < 1980 || year > 2107 || month < 1 || month > 12 ||
      day < 1 || day > 31 || hour > 23 || minute > 59 || second > 59)
    return false;
  dir_t *d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d)
    return false;
  uint16_t dirDate = FAT_DATE(year, month, day);
  uint16_t dirTime = FAT_TIME(hour, minute, second);
  if (flags & T_ACCESS)
    d->lastAccessDate = dirDate;
  if (flags & T_CREATE)
  {
    d->creationDate = dirDate;
    d->creationTime = dirTime;
    d->creationTimeTenths = second & 1 ? 100 : 0;
  }
  if (flags & T_WRITE)
  {
    d->lastWriteDate = dirDate;
    d->lastWriteTime = dirTime;
  }
  SdVolume::cacheSetDirty();
  return sync();
}

uint8_t SdFile::truncate(uint32_t length)
{
  if (!isFile() || !(flags_ & O_WRITE))
    return false;
  if (length > fileSize_)
    return false;
  if (fileSize_ == 0)
    return true;
  uint32_t newPos = curPosition_ > length ? length : curPosition_;
  if (!seekSet(length))
    return false;
  if (length == 0)
  {
    if (!vol_->freeChain(firstCluster_))
      return false;
    firstCluster_ = 0;
  }
  else
  {
    uint32_t toFree;
    if (!vol_->fatGet(curCluster_, &toFree))
      return false;
    if (!vol_->isEOC(toFree))
    {
      if (!vol_->freeChain(toFree))
        return false;
      if (!vol_->fatPutEOC(curCluster_))
        return false;
    }
  }
  fileSize_ = length;
  flags_ |= F_FILE_DIR_DIRTY;
  if (!sync())
    return false;
  return seekSet(newPos);
}

size_t SdFile::write(const void *buf, uint16_t nbyte)
{
  const uint8_t *src = reinterpret_cast<const uint8_t *>(buf);
  uint16_t nToWrite = nbyte;
  uint8_t blocking = (flags_ & F_FILE_NON_BLOCKING_WRITE) == 0x00;
  if (!isFile() || !(flags_ & O_WRITE))
    goto writeErrorReturn;
  if ((flags_ & O_APPEND) && curPosition_ != fileSize_)
  {
    if (!seekEnd())
      goto writeErrorReturn;
  }
  while (nToWrite > 0)
  {
    uint8_t blockOfCluster = vol_->blockOfCluster(curPosition_);
    uint16_t blockOffset = curPosition_ & 0X1FF;
    if (blockOfCluster == 0 && blockOffset == 0)
    {
      if (curCluster_ == 0)
      {
        if (firstCluster_ == 0)
        {
          if (!addCluster())
            goto writeErrorReturn;
        }
        else
          curCluster_ = firstCluster_;
      }
      else
      {
        uint32_t next;
        if (!vol_->fatGet(curCluster_, &next))
          return false;
        if (vol_->isEOC(next))
        {
          if (!addCluster())
            goto writeErrorReturn;
        }
        else
          curCluster_ = next;
      }
    }
    uint16_t n = 512 - blockOffset;
    if (n > nToWrite)
      n = nToWrite;
    uint32_t block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;
    if (n == 512)
    {
      if (SdVolume::cacheBlockNumber_ == block)
        SdVolume::cacheBlockNumber_ = 0XFFFFFFFF;
      if (!vol_->writeBlock(block, src, blocking))
        goto writeErrorReturn;
      src += 512;
    }
    else
    {
      if (blockOffset == 0 && curPosition_ >= fileSize_)
      {
        if (!SdVolume::cacheFlush())
          goto writeErrorReturn;
        SdVolume::cacheBlockNumber_ = block;
        SdVolume::cacheSetDirty();
      }
      else
      {
        if (!SdVolume::cacheRawBlock(block, SdVolume::CACHE_FOR_WRITE))
          goto writeErrorReturn;
      }
      uint8_t *dst = SdVolume::cacheBuffer_.data + blockOffset;
      uint8_t *end = dst + n;
      while (dst != end)
        *dst++ = *src++;
    }
    nToWrite -= n;
    curPosition_ += n;
  }
  if (curPosition_ > fileSize_)
  {
    fileSize_ = curPosition_;
    flags_ |= F_FILE_DIR_DIRTY;
  }
  else if (dateTime_ && nbyte)
    flags_ |= F_FILE_DIR_DIRTY;

  if (flags_ & O_SYNC)
    if (!sync())
      goto writeErrorReturn;
  return nbyte;
writeErrorReturn:
  setWriteError();
  return 0;
}

size_t SdFile::write(uint8_t b)
{
  return write(&b, 1);
}

size_t SdFile::write(const char *str)
{
  return write(str, strlen(str));
}

void SdFile::write_P(PGM_P str)
{
  for (uint8_t c; (c = pgm_read_byte(str)); str++)
  {
    write(c);
  }
}

void SdFile::writeln_P(PGM_P str)
{
  write_P(str);
  println();
}

int SdFile::availableForWrite()
{
  if (!isFile() || !(flags_ & O_WRITE))
    return 0;
  if ((flags_ & O_APPEND) && curPosition_ != fileSize_)
    if (!seekEnd())
      return 0;
  if (vol_->isBusy())
    return 0;
  if (flags_ & F_FILE_CLUSTER_ADDED)
  {
    sync(0);
    flags_ &= ~F_FILE_CLUSTER_ADDED;
    return 0;
  }
  if (vol_->isCacheMirrorBlockDirty())
  {
    vol_->cacheMirrorBlockFlush(0);
    return 0;
  }
  flags_ |= F_FILE_NON_BLOCKING_WRITE;
  uint16_t blockOffset = curPosition_ & 0X1FF;
  uint16_t n = 512 - blockOffset;
  return n;
}
