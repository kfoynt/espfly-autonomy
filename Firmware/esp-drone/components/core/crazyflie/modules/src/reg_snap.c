/*
 * Register snapshots for the take-off reset investigation: after a software
 * reset taken on USB, first take-offs brown out until the chip is power-cycled
 * (battery replug or the XIAO RST button). These copies show which retained
 * chip state differs between such a boot and a clean power-on boot.
 */
#include "reg_snap.h"

#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "soc/soc.h"
#include "soc/reg_base.h"
#include "soc/regi2c_bbpll.h"
#include "soc/regi2c_ulp.h"
#include "soc/regi2c_dig_reg.h"
#include "soc/regi2c_saradc.h"
#include "esp_private/regi2c_ctrl.h"

typedef struct {
  const char *name;
  uint32_t base;
  uint16_t first;   /* first word offset (bytes) */
  uint16_t words;
} regBlock_t;

/* Only read-side-effect-free ranges: the USB-Serial-JTAG EP1 FIFO at +0x00
 * would pop console bytes, and the analog I2C master's command registers
 * below +0x40 are skipped. */
static const regBlock_t blocks[] = {
  {"rtccntl", DR_REG_RTCCNTL_BASE, 0x000, 128},
  {"rtcio", DR_REG_RTCIO_BASE, 0x000, 128},
  {"sens", DR_REG_SENS_BASE, 0x000, 128},
  {"iomux", DR_REG_IO_MUX_BASE, 0x000, 64},
  {"usbjtag", DR_REG_USB_SERIAL_JTAG_BASE, 0x004, 32},
  {"anamst", 0x6000E000, 0x040, 4},
};
#define BLOCK_COUNT (sizeof(blocks) / sizeof(blocks[0]))

typedef struct {
  const char *name;
  uint8_t block;
  uint8_t regs;
} anaBlock_t;

static const anaBlock_t anaBlocks[] = {
  {"ulp_bod", I2C_ULP, 12},
  {"dig_reg", I2C_DIG_REG, 12},
  {"bbpll", I2C_BBPLL, 12},
  {"sar_adc", I2C_SAR_ADC, 12},
};
#define ANA_BLOCK_COUNT (sizeof(anaBlocks) / sizeof(anaBlocks[0]))

/* 2 x 2.1 KB, in PSRAM (internal RAM is short): a console diagnostic only. */
static regSnap_t *bootSnap;
static regSnap_t *nowSnap;

static regSnap_t *snapAlloc(void)
{
  regSnap_t *s = heap_caps_calloc(1, sizeof(regSnap_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return s ? s : heap_caps_calloc(1, sizeof(regSnap_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

static void take(regSnap_t *s)
{
  int k = 0;
  s->takenUs = (uint32_t)esp_timer_get_time();
  for (int b = 0; b < BLOCK_COUNT; b++) {
    for (int i = 0; i < blocks[b].words && k < REG_SNAP_WORDS; i++) {
      s->w[k++] = REG_READ(blocks[b].base + blocks[b].first + 4 * i);
    }
  }
  s->words = k;
  k = 0;
  /* The SAR ADC analog block only answers while its I2C clock gate is open. */
  regi2c_saradc_enable();
  for (int b = 0; b < ANA_BLOCK_COUNT; b++) {
    for (int r = 0; r < anaBlocks[b].regs && k < REG_SNAP_ANA_BYTES; r++) {
      s->ana[k++] = regi2c_ctrl_read_reg(anaBlocks[b].block, 1, r);
    }
  }
  regi2c_saradc_disable();
}

void regSnapBoot(void)
{
  if (bootSnap == NULL) {
    bootSnap = snapAlloc();
  }
  if (bootSnap != NULL) {
    take(bootSnap);
  }
}

static void printSnap(void (*line)(const char *fmt, ...), const char *tag, const regSnap_t *s)
{
  int k = 0;
  for (int b = 0; b < BLOCK_COUNT; b++) {
    for (int i = 0; i < blocks[b].words; i += 8) {
      char buf[8 * 9 + 1];
      int n = 0;
      for (int j = i; j < i + 8 && j < blocks[b].words; j++) {
        n += snprintf(buf + n, sizeof(buf) - n, " %08x", (unsigned)s->w[k + j]);
      }
      line("R %s %s +%03x%s", tag, blocks[b].name, (unsigned)(blocks[b].first + 4 * i), buf);
    }
    k += blocks[b].words;
  }
  k = 0;
  for (int b = 0; b < ANA_BLOCK_COUNT; b++) {
    char buf[REG_SNAP_ANA_BYTES * 3 + 1];
    int n = 0;
    for (int r = 0; r < anaBlocks[b].regs; r++) {
      n += snprintf(buf + n, sizeof(buf) - n, " %02x", s->ana[k++]);
    }
    line("A %s %s 0x%02x%s", tag, anaBlocks[b].name, anaBlocks[b].block, buf);
  }
}

void regSnapPrint(void (*line)(const char *fmt, ...))
{
  if (nowSnap == NULL) {
    nowSnap = snapAlloc();
  }
  if (bootSnap == NULL || nowSnap == NULL) {
    line("E no memory for the register snapshots");
    return;
  }
  take(nowSnap);
  uint32_t blkMajor = 0, ocode = 0;
  esp_efuse_read_field_blob(ESP_EFUSE_BLK_VERSION_MAJOR, &blkMajor, ESP_EFUSE_BLK_VERSION_MAJOR[0]->bit_count);
  esp_efuse_read_field_blob(ESP_EFUSE_OCODE, &ocode, ESP_EFUSE_OCODE[0]->bit_count);
  line("E blkVerMajor=%u efuseOcode=0x%02x bootUs=%u nowUs=%u", (unsigned)blkMajor, (unsigned)ocode,
       (unsigned)bootSnap->takenUs, (unsigned)nowSnap->takenUs);
  printSnap(line, "boot", bootSnap);
  printSnap(line, "now", nowSnap);
}
