#ifndef SDINFO_h
#define SDINFO_h
#include <stdint.h>

uint8_t const CMD0 = 0X00;
uint8_t const CMD8 = 0X08;
uint8_t const CMD9 = 0X09;
uint8_t const CMD10 = 0X0A;
uint8_t const CMD13 = 0X0D;
uint8_t const CMD17 = 0X11;
uint8_t const CMD24 = 0X18;
uint8_t const CMD25 = 0X19;
uint8_t const CMD32 = 0X20;
uint8_t const CMD33 = 0X21;
uint8_t const CMD38 = 0X26;
uint8_t const CMD55 = 0X37;
uint8_t const CMD58 = 0X3A;
uint8_t const ACMD23 = 0X17;
uint8_t const ACMD41 = 0X29;
uint8_t const R1_READY_STATE = 0X00;
uint8_t const R1_IDLE_STATE = 0X01;
uint8_t const R1_ILLEGAL_COMMAND = 0X04;
uint8_t const DATA_START_BLOCK = 0XFE;
uint8_t const STOP_TRAN_TOKEN = 0XFD;
uint8_t const WRITE_MULTIPLE_TOKEN = 0XFC;
uint8_t const DATA_RES_MASK = 0X1F;
uint8_t const DATA_RES_ACCEPTED = 0X05;

typedef struct CID
{
  uint8_t mid;
  char oid[2];
  char pnm[5];
  unsigned prv_m : 4;
  unsigned prv_n : 4;
  uint32_t psn;
  unsigned mdt_year_high : 4;
  unsigned reserved : 4;
  unsigned mdt_month : 4;
  unsigned mdt_year_low : 4;
  unsigned always1 : 1;
  unsigned crc : 7;
} cid_t;

typedef struct CSDV1
{
  unsigned reserved1 : 6;
  unsigned csd_ver : 2;
  uint8_t taac;
  uint8_t nsac;
  uint8_t tran_speed;
  uint8_t ccc_high;
  unsigned read_bl_len : 4;
  unsigned ccc_low : 4;
  unsigned c_size_high : 2;
  unsigned reserved2 : 2;
  unsigned dsr_imp : 1;
  unsigned read_blk_misalign : 1;
  unsigned write_blk_misalign : 1;
  unsigned read_bl_partial : 1;
  uint8_t c_size_mid;
  unsigned vdd_r_curr_max : 3;
  unsigned vdd_r_curr_min : 3;
  unsigned c_size_low : 2;
  unsigned c_size_mult_high : 2;
  unsigned vdd_w_cur_max : 3;
  unsigned vdd_w_curr_min : 3;
  unsigned sector_size_high : 6;
  unsigned erase_blk_en : 1;
  unsigned c_size_mult_low : 1;
  unsigned wp_grp_size : 7;
  unsigned sector_size_low : 1;
  unsigned write_bl_len_high : 2;
  unsigned r2w_factor : 3;
  unsigned reserved3 : 2;
  unsigned wp_grp_enable : 1;
  unsigned reserved4 : 5;
  unsigned write_partial : 1;
  unsigned write_bl_len_low : 2;
  unsigned reserved5 : 2;
  unsigned file_format : 2;
  unsigned tmp_write_protect : 1;
  unsigned perm_write_protect : 1;
  unsigned copy : 1;
  unsigned file_format_grp : 1;
  unsigned always1 : 1;
  unsigned crc : 7;
} csd1_t;

typedef struct CSDV2
{
  unsigned reserved1 : 6;
  unsigned csd_ver : 2;
  uint8_t taac;
  uint8_t nsac;
  uint8_t tran_speed;
  uint8_t ccc_high;
  unsigned read_bl_len : 4;
  unsigned ccc_low : 4;
  unsigned reserved2 : 4;
  unsigned dsr_imp : 1;
  unsigned read_blk_misalign : 1;
  unsigned write_blk_misalign : 1;
  unsigned read_bl_partial : 1;
  unsigned reserved3 : 2;
  unsigned c_size_high : 6;
  uint8_t c_size_mid;
  uint8_t c_size_low;
  unsigned sector_size_high : 6;
  unsigned erase_blk_en : 1;
  unsigned reserved4 : 1;
  unsigned wp_grp_size : 7;
  unsigned sector_size_low : 1;
  unsigned write_bl_len_high : 2;
  unsigned r2w_factor : 3;
  unsigned reserved5 : 2;
  unsigned wp_grp_enable : 1;
  unsigned reserved6 : 5;
  unsigned write_partial : 1;
  unsigned write_bl_len_low : 2;
  unsigned reserved7 : 2;
  unsigned file_format : 2;
  unsigned tmp_write_protect : 1;
  unsigned perm_write_protect : 1;
  unsigned copy : 1;
  unsigned file_format_grp : 1;
  unsigned always1 : 1;
  unsigned crc : 7;
} csd2_t;

union csd_t
{
  csd1_t v1;
  csd2_t v2;
};
#endif
