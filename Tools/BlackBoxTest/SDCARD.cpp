#include "SDCARD.h"
#include <SPI.h>

static SPISettings Settings;

uint8_t _SPI3_SCK;   //SCLK
uint8_t _SPI3_MOSI;  //MOSI
uint8_t _SPI3_MISO;  //MISO
uint8_t CHIP_SELECT; //CS

static void SPI_Send(uint8_t b)
{
  SPI.transfer(b);
}

static uint8_t SPI_Rec(void)
{
  return SPI.transfer(0xFF);
}

uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg)
{
  readEnd();
  chipSelectLow();
  waitNotBusy(300);
  SPI_Send(cmd | 0x40);
  for (int8_t s = 24; s >= 0; s -= 8)
    SPI_Send(arg >> s);
  uint8_t crc = 0XFF;
  if (cmd == CMD0)
    crc = 0X95;
  if (cmd == CMD8)
    crc = 0X87;
  SPI_Send(crc);
  for (uint8_t i = 0; ((status_ = SPI_Rec()) & 0X80) && i != 0XFF; i++)
    ;
  return status_;
}

uint32_t Sd2Card::cardSize(void)
{
  csd_t csd;
  if (!readCSD(&csd))
    return 0;
  if (csd.v1.csd_ver == 0)
  {
    uint8_t read_bl_len = csd.v1.read_bl_len;
    uint16_t c_size = (csd.v1.c_size_high << 10) | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
    uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1) | csd.v1.c_size_mult_low;
    return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
  }
  else if (csd.v2.csd_ver == 1)
  {
    uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16) | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
    return (c_size + 1) << 10;
  }
  else
  {
    error(SD_CARD_ERROR_BAD_CSD);
    return 0;
  }
}

static uint8_t chip_select_asserted = 0;
void Sd2Card::chipSelectHigh(void)
{
  digitalWrite(chipSelectPin_, HIGH);
  if (chip_select_asserted)
  {
    chip_select_asserted = 0;
    SPI.endTransaction();
  }
}

void Sd2Card::chipSelectLow(void)
{
  if (!chip_select_asserted)
  {
    chip_select_asserted = 1;
    SPI.beginTransaction(Settings);
  }
  digitalWrite(chipSelectPin_, LOW);
}

uint8_t Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock)
{
  if (!eraseSingleBlockEnable())
  {
    error(SD_CARD_ERROR_ERASE_SINGLE_BLOCK);
    goto fail;
  }
  if (type_ != SD_CARD_TYPE_SDHC)
  {
    firstBlock <<= 9;
    lastBlock <<= 9;
  }
  if (cardCommand(CMD32, firstBlock) || cardCommand(CMD33, lastBlock) || cardCommand(CMD38, 0))
  {
    error(SD_CARD_ERROR_ERASE);
    goto fail;
  }
  if (!waitNotBusy(SD_ERASE_TIMEOUT))
  {
    error(SD_CARD_ERROR_ERASE_TIMEOUT);
    goto fail;
  }
  chipSelectHigh();
  return true;
fail:
  chipSelectHigh();
  return false;
}

uint8_t Sd2Card::eraseSingleBlockEnable(void)
{
  csd_t csd;
  return readCSD(&csd) ? csd.v1.erase_blk_en : 0;
}

uint8_t Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin)
{
  errorCode_ = inBlock_ = partialBlockRead_ = type_ = 0;
  chipSelectPin_ = chipSelectPin;
  unsigned int t0 = millis();
  uint32_t arg;
  pinMode(chipSelectPin_, OUTPUT);
  digitalWrite(chipSelectPin_, HIGH);
  SPI.begin();
  Settings = SPISettings(250000, MSBFIRST, SPI_MODE0);
  for (uint8_t i = 0; i < 10; i++)
    SPI_Send(0XFF);
  SPI.endTransaction();
  chipSelectLow();
  while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE)
  {
    unsigned int d = millis() - t0;
    if (d > SD_INIT_TIMEOUT)
    {
      error(SD_CARD_ERROR_CMD0);
      goto fail;
    }
  }
  if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND))
    type(SD_CARD_TYPE_SD1);
  else
  {
    for (uint8_t i = 0; i < 4; i++)
      status_ = SPI_Rec();
    if (status_ != 0XAA)
    {
      error(SD_CARD_ERROR_CMD8);
      goto fail;
    }
    type(SD_CARD_TYPE_SD2);
  }
  arg = type() == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;
  while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE)
  {
    unsigned int d = millis() - t0;
    if (d > SD_INIT_TIMEOUT)
    {
      error(SD_CARD_ERROR_ACMD41);
      goto fail;
    }
  }
  if (type() == SD_CARD_TYPE_SD2)
  {
    if (cardCommand(CMD58, 0))
    {
      error(SD_CARD_ERROR_CMD58);
      goto fail;
    }
    if ((SPI_Rec() & 0XC0) == 0XC0)
      type(SD_CARD_TYPE_SDHC);
    for (uint8_t i = 0; i < 3; i++)
      SPI_Rec();
  }
  chipSelectHigh();
  return true;
fail:
  chipSelectHigh();
  return false;
}

void Sd2Card::partialBlockRead(uint8_t value)
{
  readEnd();
  partialBlockRead_ = value;
}

uint8_t Sd2Card::readBlock(uint32_t block, uint8_t *dst)
{
  return readData(block, 0, 512, dst);
}

uint8_t Sd2Card::readData(uint32_t block, uint16_t offset, uint16_t count, uint8_t *dst)
{
  if (count == 0)
    return true;
  if ((count + offset) > 512)
    goto fail;
  if (!inBlock_ || block != block_ || offset < offset_)
  {
    block_ = block;
    if (type() != SD_CARD_TYPE_SDHC)
      block <<= 9;
    if (cardCommand(CMD17, block))
    {
      error(SD_CARD_ERROR_CMD17);
      goto fail;
    }
    if (!waitStartBlock())
      goto fail;
    offset_ = 0;
    inBlock_ = 1;
  }
  for (; offset_ < offset; offset_++)
    SPI_Rec();
  for (uint16_t i = 0; i < count; i++)
    dst[i] = SPI_Rec();
  offset_ += count;
  if (!partialBlockRead_ || offset_ >= 512)
    readEnd();
  return true;
fail:
  chipSelectHigh();
  return false;
}

void Sd2Card::readEnd(void)
{
  if (inBlock_)
  {
    while (offset_++ < 514)
      SPI_Rec();
    chipSelectHigh();
    inBlock_ = 0;
  }
}

uint8_t Sd2Card::readRegister(uint8_t cmd, void *buf)
{
  uint8_t *dst = reinterpret_cast<uint8_t *>(buf);
  if (cardCommand(cmd, 0))
  {
    error(SD_CARD_ERROR_READ_REG);
    goto fail;
  }
  if (!waitStartBlock())
    goto fail;
  for (uint16_t i = 0; i < 16; i++)
    dst[i] = SPI_Rec();
  SPI_Rec();
  SPI_Rec();
  chipSelectHigh();
  return true;
fail:
  chipSelectHigh();
  return false;
}

uint8_t Sd2Card::setSpiClock(uint32_t clock)
{
  Settings = SPISettings(clock, MSBFIRST, SPI_MODE0);
  return true;
}

uint8_t Sd2Card::waitNotBusy(unsigned int timeoutMillis)
{
  unsigned int t0 = millis();
  unsigned int d;
  do
  {
    if (SPI_Rec() == 0XFF)
      return true;
    d = millis() - t0;
  } while (d < timeoutMillis);
  return false;
}

uint8_t Sd2Card::waitStartBlock(void)
{
  unsigned int t0 = millis();
  while ((status_ = SPI_Rec()) == 0XFF)
  {
    unsigned int d = millis() - t0;
    if (d > SD_READ_TIMEOUT)
    {
      error(SD_CARD_ERROR_READ_TIMEOUT);
      goto fail;
    }
  }
  if (status_ != DATA_START_BLOCK)
  {
    error(SD_CARD_ERROR_READ);
    goto fail;
  }
  return true;
fail:
  chipSelectHigh();
  return false;
}

uint8_t Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t *src, uint8_t blocking)
{
  if (blockNumber == 0)
  {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
  if (type() != SD_CARD_TYPE_SDHC)
    blockNumber <<= 9;
  if (cardCommand(CMD24, blockNumber))
  {
    error(SD_CARD_ERROR_CMD24);
    goto fail;
  }
  if (!writeData(DATA_START_BLOCK, src))
    goto fail;
  if (blocking)
  {
    if (!waitNotBusy(SD_WRITE_TIMEOUT))
    {
      error(SD_CARD_ERROR_WRITE_TIMEOUT);
      goto fail;
    }
    if (cardCommand(CMD13, 0) || SPI_Rec())
    {
      error(SD_CARD_ERROR_WRITE_PROGRAMMING);
      goto fail;
    }
  }
  chipSelectHigh();
  return true;
fail:
  chipSelectHigh();
  return false;
}

uint8_t Sd2Card::writeData(const uint8_t *src)
{
  if (!waitNotBusy(SD_WRITE_TIMEOUT))
  {
    error(SD_CARD_ERROR_WRITE_MULTIPLE);
    chipSelectHigh();
    return false;
  }
  return writeData(WRITE_MULTIPLE_TOKEN, src);
}

uint8_t Sd2Card::writeData(uint8_t token, const uint8_t *src)
{
  SPI_Send(token);
  for (uint16_t i = 0; i < 512; i++)
    SPI_Send(src[i]);
  SPI_Send(0xff);
  SPI_Send(0xff);
  status_ = SPI_Rec();
  if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED)
  {
    error(SD_CARD_ERROR_WRITE);
    chipSelectHigh();
    return false;
  }
  return true;
}

uint8_t Sd2Card::writeStart(uint32_t blockNumber, uint32_t eraseCount)
{
  if (blockNumber == 0)
  {
    error(SD_CARD_ERROR_WRITE_BLOCK_ZERO);
    goto fail;
  }
  if (cardAcmd(ACMD23, eraseCount))
  {
    error(SD_CARD_ERROR_ACMD23);
    goto fail;
  }
  if (type() != SD_CARD_TYPE_SDHC)
    blockNumber <<= 9;
  if (cardCommand(CMD25, blockNumber))
  {
    error(SD_CARD_ERROR_CMD25);
    goto fail;
  }
  return true;
fail:
  chipSelectHigh();
  return false;
}

uint8_t Sd2Card::writeStop(void)
{
  if (!waitNotBusy(SD_WRITE_TIMEOUT))
    goto fail;
  SPI_Send(STOP_TRAN_TOKEN);
  if (!waitNotBusy(SD_WRITE_TIMEOUT))
    goto fail;
  chipSelectHigh();
  return true;
fail:
  error(SD_CARD_ERROR_STOP_TRAN);
  chipSelectHigh();
  return false;
}

uint8_t Sd2Card::isBusy(void)
{
  chipSelectLow();
  byte b = SPI_Rec();
  chipSelectHigh();
  return (b != 0XFF);
}
