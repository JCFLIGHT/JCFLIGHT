#ifndef SDFAT_h
#define SDFAT_h

#include "SDCARD.h"
#include "Print.h"

uint8_t const BOOTSIG0 = 0X55;
uint8_t const BOOTSIG1 = 0XAA;

struct partitionTable
{
  uint8_t boot;
  uint8_t beginHead;
  unsigned beginSector : 6;
  unsigned beginCylinderHigh : 2;
  uint8_t beginCylinderLow;
  uint8_t type;
  uint8_t endHead;
  unsigned endSector : 6;
  unsigned endCylinderHigh : 2;
  uint8_t endCylinderLow;
  uint32_t firstSector;
  uint32_t totalSectors;
} __attribute__((packed));

typedef struct partitionTable part_t;

struct masterBootRecord
{
  uint8_t codeArea[440];
  uint32_t diskSignature;
  uint16_t usuallyZero;
  part_t part[4];
  uint8_t mbrSig0;
  uint8_t mbrSig1;
} __attribute__((packed));

typedef struct masterBootRecord mbr_t;

struct biosParmBlock
{
  uint16_t bytesPerSector;
  uint8_t sectorsPerCluster;
  uint16_t reservedSectorCount;
  uint8_t fatCount;
  uint16_t rootDirEntryCount;
  uint16_t totalSectors16;
  uint8_t mediaType;
  uint16_t sectorsPerFat16;
  uint16_t sectorsPerTrtack;
  uint16_t headCount;
  uint32_t hidddenSectors;
  uint32_t totalSectors32;
  uint32_t sectorsPerFat32;
  uint16_t fat32Flags;
  uint16_t fat32Version;
  uint32_t fat32RootCluster;
  uint16_t fat32FSInfo;
  uint16_t fat32BackBootBlock;
  uint8_t fat32Reserved[12];
} __attribute__((packed));

typedef struct biosParmBlock bpb_t;

struct fat32BootSector
{
  uint8_t jmpToBootCode[3];
  char oemName[8];
  bpb_t bpb;
  uint8_t driveNumber;
  uint8_t reserved1;
  uint8_t bootSignature;
  uint32_t volumeSerialNumber;
  char volumeLabel[11];
  char fileSystemType[8];
  uint8_t bootCode[420];
  uint8_t bootSectorSig0;
  uint8_t bootSectorSig1;
} __attribute__((packed));

uint16_t const FAT16EOC = 0XFFFF;
uint16_t const FAT16EOC_MIN = 0XFFF8;
uint32_t const FAT32EOC = 0X0FFFFFFF;
uint32_t const FAT32EOC_MIN = 0X0FFFFFF8;
uint32_t const FAT32MASK = 0X0FFFFFFF;

typedef struct fat32BootSector fbs_t;

struct directoryEntry
{
  uint8_t name[11];
  uint8_t attributes;
  uint8_t reservedNT;
  uint8_t creationTimeTenths;
  uint16_t creationTime;
  uint16_t creationDate;
  uint16_t lastAccessDate;
  uint16_t firstClusterHigh;
  uint16_t lastWriteTime;
  uint16_t lastWriteDate;
  uint16_t firstClusterLow;
  uint32_t fileSize;
} __attribute__((packed));

typedef struct directoryEntry dir_t;

uint8_t const DIR_NAME_0XE5 = 0X05;
uint8_t const DIR_NAME_DELETED = 0XE5;
uint8_t const DIR_NAME_FREE = 0X00;
uint8_t const DIR_ATT_READ_ONLY = 0X01;
uint8_t const DIR_ATT_HIDDEN = 0X02;
uint8_t const DIR_ATT_SYSTEM = 0X04;
uint8_t const DIR_ATT_VOLUME_ID = 0X08;
uint8_t const DIR_ATT_DIRECTORY = 0X10;
uint8_t const DIR_ATT_ARCHIVE = 0X20;
uint8_t const DIR_ATT_LONG_NAME = 0X0F;
uint8_t const DIR_ATT_LONG_NAME_MASK = 0X3F;
uint8_t const DIR_ATT_DEFINED_BITS = 0X3F;
static inline uint8_t DIR_IS_LONG_NAME(const dir_t *dir)
{
  return (dir->attributes & DIR_ATT_LONG_NAME_MASK) == DIR_ATT_LONG_NAME;
}

uint8_t const DIR_ATT_FILE_TYPE_MASK = (DIR_ATT_VOLUME_ID | DIR_ATT_DIRECTORY);

static inline uint8_t DIR_IS_FILE(const dir_t *dir)
{
  return (dir->attributes & DIR_ATT_FILE_TYPE_MASK) == 0;
}

static inline uint8_t DIR_IS_SUBDIR(const dir_t *dir)
{
  return (dir->attributes & DIR_ATT_FILE_TYPE_MASK) == DIR_ATT_DIRECTORY;
}

static inline uint8_t DIR_IS_FILE_OR_SUBDIR(const dir_t *dir)
{
  return (dir->attributes & DIR_ATT_VOLUME_ID) == 0;
}

#define ALLOW_DEPRECATED_FUNCTIONS 1

class SdVolume;

uint8_t const LS_DATE = 1;
uint8_t const LS_SIZE = 2;
uint8_t const LS_R = 4;
uint8_t const O_READ = 0X01;
uint8_t const O_RDONLY = O_READ;
uint8_t const O_WRITE = 0X02;
uint8_t const O_WRONLY = O_WRITE;
uint8_t const O_RDWR = (O_READ | O_WRITE);
uint8_t const O_ACCMODE = (O_READ | O_WRITE);
uint8_t const O_APPEND = 0X04;
uint8_t const O_SYNC = 0X08;
uint8_t const O_CREAT = 0X10;
uint8_t const O_EXCL = 0X20;
uint8_t const O_TRUNC = 0X40;
uint8_t const T_ACCESS = 1;
uint8_t const T_CREATE = 2;
uint8_t const T_WRITE = 4;
uint8_t const FAT_FILE_TYPE_CLOSED = 0;
uint8_t const FAT_FILE_TYPE_NORMAL = 1;
uint8_t const FAT_FILE_TYPE_ROOT16 = 2;
uint8_t const FAT_FILE_TYPE_ROOT32 = 3;
uint8_t const FAT_FILE_TYPE_SUBDIR = 4;
uint8_t const FAT_FILE_TYPE_MIN_DIR = FAT_FILE_TYPE_ROOT16;

static inline uint16_t FAT_DATE(uint8_t day, uint8_t month, uint16_t year)
{
  return (year - 1980) << 9 | month << 5 | day;
}

static inline uint16_t FAT_YEAR(uint16_t fatDate)
{
  return 1980 + (fatDate >> 9);
}

static inline uint8_t FAT_MONTH(uint16_t fatDate)
{
  return (fatDate >> 5) & 0XF;
}

static inline uint8_t FAT_DAY(uint16_t fatDate)
{
  return fatDate & 0X1F;
}

static inline uint16_t FAT_TIME(uint8_t hour, uint8_t minute, uint8_t second)
{
  return hour << 11 | minute << 5 | second >> 1;
}

static inline uint8_t FAT_HOUR(uint16_t fatTime)
{
  return fatTime >> 11;
}

static inline uint8_t FAT_MINUTE(uint16_t fatTime)
{
  return (fatTime >> 5) & 0X3F;
}

static inline uint8_t FAT_SECOND(uint16_t fatTime)
{
  return 2 * (fatTime & 0X1F);
}

uint16_t const FAT_DEFAULT_DATE = ((2000 - 1980) << 9) | (1 << 5) | 1;
uint16_t const FAT_DEFAULT_TIME = (1 << 11);

class SdFile : public Print
{
public:
  SdFile(void) : type_(FAT_FILE_TYPE_CLOSED) {}
  void clearUnbufferedRead(void)
  {
    flags_ &= ~F_FILE_UNBUFFERED_READ;
  }
  uint8_t close(void);
  uint8_t timestamp(uint8_t flag, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
  uint8_t contiguousRange(uint32_t *bgnBlock, uint32_t *endBlock);
  uint8_t createContiguous(SdFile *dirFile, const char *fileName, uint32_t size);
  uint32_t curCluster(void) const
  {
    return curCluster_;
  }
  uint32_t curPosition(void) const
  {
    return curPosition_;
  }
  static void dateTimeCallback(
      void (*dateTime)(uint16_t *Date, uint16_t *Time))
  {
    dateTime_ = dateTime;
  }
  static void dateTimeCallbackCancel(void)
  {
    dateTime_ = 0;
  }
  uint32_t dirBlock(void) const
  {
    return dirBlock_;
  }
  uint8_t dirEntry(dir_t *dir);
  uint8_t dirIndex(void) const
  {
    return dirIndex_;
  }
  static void dirName(const dir_t &dir, char *name);
  uint32_t fileSize(void) const
  {
    return fileSize_;
  }
  uint32_t firstCluster(void) const
  {
    return firstCluster_;
  }
  uint8_t isDir(void) const
  {
    return type_ >= FAT_FILE_TYPE_MIN_DIR;
  }
  uint8_t isFile(void) const
  {
    return type_ == FAT_FILE_TYPE_NORMAL;
  }
  uint8_t isOpen(void) const
  {
    return type_ != FAT_FILE_TYPE_CLOSED;
  }
  uint8_t isSubDir(void) const
  {
    return type_ == FAT_FILE_TYPE_SUBDIR;
  }
  uint8_t isRoot(void) const
  {
    return type_ == FAT_FILE_TYPE_ROOT16 || type_ == FAT_FILE_TYPE_ROOT32;
  }
  void ls(uint8_t flags = 0, uint8_t indent = 0);
  uint8_t makeDir(SdFile *dir, const char *dirName);
  uint8_t open(SdFile *dirFile, uint16_t index, uint8_t oflag);
  uint8_t open(SdFile *dirFile, const char *fileName, uint8_t oflag);
  uint8_t openRoot(SdVolume *vol);
  static void printDirName(const dir_t &dir, uint8_t width);
  static void printFatDate(uint16_t fatDate);
  static void printFatTime(uint16_t fatTime);
  static void printTwoDigits(uint8_t v);
  int16_t read(void)
  {
    uint8_t b;
    return read(&b, 1) == 1 ? b : -1;
  }
  int16_t read(void *buf, uint16_t nbyte);
  int8_t readDir(dir_t *dir);
  static uint8_t remove(SdFile *dirFile, const char *fileName);
  uint8_t remove(void);
  void rewind(void)
  {
    curPosition_ = curCluster_ = 0;
  }
  uint8_t rmDir(void);
  uint8_t rmRfStar(void);
  uint8_t seekCur(uint32_t pos)
  {
    return seekSet(curPosition_ + pos);
  }
  uint8_t seekEnd(void)
  {
    return seekSet(fileSize_);
  }
  uint8_t seekSet(uint32_t pos);
  void setUnbufferedRead(void)
  {
    if (isFile())
    {
      flags_ |= F_FILE_UNBUFFERED_READ;
    }
  }
  uint8_t sync(uint8_t blocking = 1);
  uint8_t type(void) const
  {
    return type_;
  }
  uint8_t truncate(uint32_t size);
  uint8_t unbufferedRead(void) const
  {
    return flags_ & F_FILE_UNBUFFERED_READ;
  }
  SdVolume *volume(void) const
  {
    return vol_;
  }
  size_t write(uint8_t b);
  size_t write(const void *buf, uint16_t nbyte);
  size_t write(const char *str);
  void write_P(PGM_P str);
  void writeln_P(PGM_P str);
  int availableForWrite(void);
#if ALLOW_DEPRECATED_FUNCTIONS
  uint8_t contiguousRange(uint32_t &bgnBlock, uint32_t &endBlock)
  {
    return contiguousRange(&bgnBlock, &endBlock);
  }
  uint8_t createContiguous(SdFile &dirFile, const char *fileName, uint32_t size)
  {
    return createContiguous(&dirFile, fileName, size);
  }
  static void dateTimeCallback(
      void (*dateTime)(uint16_t &Date, uint16_t &Time))
  {
    oldDateTime_ = dateTime;
    dateTime_ = dateTime ? oldToNew : 0;
  }
  uint8_t dirEntry(dir_t &dir)
  {
    return dirEntry(&dir);
  }
  uint8_t makeDir(SdFile &dir, const char *dirName)
  {
    return makeDir(&dir, dirName);
  }
  uint8_t open(SdFile &dirFile, const char *fileName, uint8_t oflag)
  {
    return open(&dirFile, fileName, oflag);
  }
  uint8_t open(SdFile &dirFile, const char *fileName)
  {
    return open(dirFile, fileName, O_RDWR);
  }
  uint8_t open(SdFile &dirFile, uint16_t index, uint8_t oflag)
  {
    return open(&dirFile, index, oflag);
  }
  uint8_t openRoot(SdVolume &vol)
  {
    return openRoot(&vol);
  }
  int8_t readDir(dir_t &dir)
  {
    return readDir(&dir);
  }
  static uint8_t remove(SdFile &dirFile, const char *fileName)
  {
    return remove(&dirFile, fileName);
  }

private:
  static void (*oldDateTime_)(uint16_t &date, uint16_t &time);
  static void oldToNew(uint16_t *date, uint16_t *time)
  {
    uint16_t d;
    uint16_t t;
    oldDateTime_(d, t);
    *date = d;
    *time = t;
  }
#endif
private:
  static uint8_t const F_OFLAG = (O_ACCMODE | O_APPEND | O_SYNC);
  static uint8_t const F_FILE_NON_BLOCKING_WRITE = 0X10;
  static uint8_t const F_FILE_CLUSTER_ADDED = 0X20;
  static uint8_t const F_FILE_UNBUFFERED_READ = 0X40;
  static uint8_t const F_FILE_DIR_DIRTY = 0X80;
#if ((F_FILE_NON_BLOCKING_WRITE | F_FILE_CLUSTER_ADDED | F_FILE_UNBUFFERED_READ | F_FILE_DIR_DIRTY) & F_OFLAG)
#error flags_ bits conflict
#endif
  uint8_t flags_;
  uint8_t type_;
  uint32_t curCluster_;
  uint32_t curPosition_;
  uint32_t dirBlock_;
  uint8_t dirIndex_;
  uint32_t fileSize_;
  uint32_t firstCluster_;
  SdVolume *vol_;
  uint8_t addCluster(void);
  uint8_t addDirCluster(void);
  dir_t *cacheDirEntry(uint8_t action);
  static void (*dateTime_)(uint16_t *date, uint16_t *time);
  static uint8_t make83Name(const char *str, uint8_t *name);
  uint8_t openCachedEntry(uint8_t cacheIndex, uint8_t oflags);
  dir_t *readDirCache(void);
};

union cache_t
{
  uint8_t data[512];
  uint16_t fat16[256];
  uint32_t fat32[128];
  dir_t dir[16];
  mbr_t mbr;
  fbs_t fbs;
};

class SdVolume
{
public:
  SdVolume(void) : allocSearchStart_(2), fatType_(0) {}
  static uint8_t *cacheClear(void)
  {
    cacheFlush();
    cacheBlockNumber_ = 0XFFFFFFFF;
    return cacheBuffer_.data;
  }
  uint8_t init(Sd2Card *dev)
  {
    return init(dev, 1) ? true : init(dev, 0);
  }
  uint8_t init(Sd2Card *dev, uint8_t part);
  uint8_t blocksPerCluster(void) const
  {
    return blocksPerCluster_;
  }
  uint32_t blocksPerFat(void) const
  {
    return blocksPerFat_;
  }
  uint32_t clusterCount(void) const
  {
    return clusterCount_;
  }
  uint8_t clusterSizeShift(void) const
  {
    return clusterSizeShift_;
  }
  uint32_t dataStartBlock(void) const
  {
    return dataStartBlock_;
  }
  uint8_t fatCount(void) const
  {
    return fatCount_;
  }
  uint32_t fatStartBlock(void) const
  {
    return fatStartBlock_;
  }
  uint8_t fatType(void) const
  {
    return fatType_;
  }
  uint32_t rootDirEntryCount(void) const
  {
    return rootDirEntryCount_;
  }
  uint32_t rootDirStart(void) const
  {
    return rootDirStart_;
  }
  static Sd2Card *sdCard(void)
  {
    return sdCard_;
  }
#if ALLOW_DEPRECATED_FUNCTIONS
  uint8_t init(Sd2Card &dev)
  {
    return init(&dev);
  }
  uint8_t init(Sd2Card &dev, uint8_t part)
  {
    return init(&dev, part);
  }
#endif

private:
  friend class SdFile;
  static uint8_t const CACHE_FOR_READ = 0;
  static uint8_t const CACHE_FOR_WRITE = 1;
  static cache_t cacheBuffer_;
  static uint32_t cacheBlockNumber_;
  static Sd2Card *sdCard_;
  static uint8_t cacheDirty_;
  static uint32_t cacheMirrorBlock_;
  uint32_t allocSearchStart_;
  uint8_t blocksPerCluster_;
  uint32_t blocksPerFat_;
  uint32_t clusterCount_;
  uint8_t clusterSizeShift_;
  uint32_t dataStartBlock_;
  uint8_t fatCount_;
  uint32_t fatStartBlock_;
  uint8_t fatType_;
  uint16_t rootDirEntryCount_;
  uint32_t rootDirStart_;
  uint8_t allocContiguous(uint32_t count, uint32_t *curCluster);
  uint8_t blockOfCluster(uint32_t position) const
  {
    return (position >> 9) & (blocksPerCluster_ - 1);
  }
  uint32_t clusterStartBlock(uint32_t cluster) const
  {
    return dataStartBlock_ + ((cluster - 2) << clusterSizeShift_);
  }
  uint32_t blockNumber(uint32_t cluster, uint32_t position) const
  {
    return clusterStartBlock(cluster) + blockOfCluster(position);
  }
  static uint8_t cacheFlush(uint8_t blocking = 1);
  static uint8_t cacheMirrorBlockFlush(uint8_t blocking);
  static uint8_t cacheRawBlock(uint32_t blockNumber, uint8_t action);
  static void cacheSetDirty(void)
  {
    cacheDirty_ |= CACHE_FOR_WRITE;
  }
  static uint8_t cacheZeroBlock(uint32_t blockNumber);
  uint8_t chainSize(uint32_t beginCluster, uint32_t *size) const;
  uint8_t fatGet(uint32_t cluster, uint32_t *value) const;
  uint8_t fatPut(uint32_t cluster, uint32_t value);
  uint8_t fatPutEOC(uint32_t cluster)
  {
    return fatPut(cluster, 0x0FFFFFFF);
  }
  uint8_t freeChain(uint32_t cluster);
  uint8_t isEOC(uint32_t cluster) const
  {
    return cluster >= (fatType_ == 16 ? FAT16EOC_MIN : FAT32EOC_MIN);
  }
  uint8_t readBlock(uint32_t block, uint8_t *dst)
  {
    return sdCard_->readBlock(block, dst);
  }
  uint8_t readData(uint32_t block, uint16_t offset,
                   uint16_t count, uint8_t *dst)
  {
    return sdCard_->readData(block, offset, count, dst);
  }
  uint8_t writeBlock(uint32_t block, const uint8_t *dst, uint8_t blocking = 1)
  {
    return sdCard_->writeBlock(block, dst, blocking);
  }
  uint8_t isBusy(void)
  {
    return sdCard_->isBusy();
  }
  uint8_t isCacheMirrorBlockDirty(void)
  {
    return (cacheMirrorBlock_ != 0);
  }
};
#endif
