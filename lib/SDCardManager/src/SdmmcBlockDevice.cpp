#include "SdmmcBlockDevice.h"

#ifdef XTEINK_X4_PRO

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "sdmmc_cmd.h"

namespace {
// 1-bit SDMMC, slot 1, through the S3 GPIO matrix.
constexpr gpio_num_t SD_CLK = GPIO_NUM_41;
constexpr gpio_num_t SD_CMD = GPIO_NUM_42;
constexpr gpio_num_t SD_D0 = GPIO_NUM_40;
// Active-LOW card enable. The OEM mountSD pulses it HIGH->LOW before each attempt and
// runs the card with it held LOW; holding it HIGH breaks every block read (0x107).
constexpr int SD_EN = 5;
}  // namespace

bool SdmmcBlockDevice::begin() {
  for (const gpio_num_t pin : {SD_CLK, SD_CMD, SD_D0}) gpio_hold_dis(pin);

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = SDMMC_FREQ_DEFAULT;  // same as freeink-sdk, hardware-validated

  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 1;
  slot.clk = SD_CLK;
  slot.cmd = SD_CMD;
  slot.d0 = SD_D0;
  slot.d1 = GPIO_NUM_NC;
  slot.d2 = GPIO_NUM_NC;
  slot.d3 = GPIO_NUM_NC;
  slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  if (sdmmc_host_init() != ESP_OK) return false;
  if (sdmmc_host_init_slot(host.slot, &slot) != ESP_OK) {
    sdmmc_host_deinit();
    return false;
  }

  auto* card = static_cast<sdmmc_card_t*>(calloc(1, sizeof(sdmmc_card_t)));
  // SDMMC DMA needs an internal, DMA-capable bounce buffer; 8 sectors bounds it at 4 KiB.
  _dmaBuffer = static_cast<uint8_t*>(heap_caps_malloc(kSectorSize * kMaxTransferSectors, MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
  if (!card || !_dmaBuffer) {
    free(card);
    heap_caps_free(_dmaBuffer);
    _dmaBuffer = nullptr;
    sdmmc_host_deinit();
    return false;
  }

  gpio_hold_dis(static_cast<gpio_num_t>(SD_EN));
  pinMode(SD_EN, OUTPUT);

  // Retry the whole mount, init AND a real sector-0 read, power-cycling the enable each
  // time, so the card is only published once genuine block I/O works.
  esp_err_t err = ESP_FAIL;
  for (int attempt = 0; attempt < 4; attempt++) {
    digitalWrite(SD_EN, HIGH);
    delay(80);
    digitalWrite(SD_EN, LOW);
    delay(120);
    err = sdmmc_card_init(&host, card);
    if (err != ESP_OK && card->csd.capacity == 0) continue;
    err = sdmmc_read_sectors(card, _dmaBuffer, 0, 1);
    if (err == ESP_OK) break;
  }
  if (err != ESP_OK) {
    if (Serial) Serial.printf("[%lu] [SD] SDMMC mount failed: %s\n", millis(), esp_err_to_name(err));
    heap_caps_free(_dmaBuffer);
    _dmaBuffer = nullptr;
    free(card);
    sdmmc_host_deinit();
    return false;
  }
  _card = card;
  return true;
}

void SdmmcBlockDevice::end() {
  if (_card) {
    free(_card);
    _card = nullptr;
    sdmmc_host_deinit();
  }
  if (_dmaBuffer) {
    heap_caps_free(_dmaBuffer);
    _dmaBuffer = nullptr;
  }
}

bool SdmmcBlockDevice::readSectors(Sector_t sector, uint8_t* dst, size_t ns) {
  if (!_card || !dst || ns == 0) return false;
  while (ns > 0) {
    const size_t count = ns > kMaxTransferSectors ? kMaxTransferSectors : ns;
    const size_t bytes = count * kSectorSize;
    if (sdmmc_read_sectors(static_cast<sdmmc_card_t*>(_card), _dmaBuffer, sector, count) != ESP_OK) return false;
    memcpy(dst, _dmaBuffer, bytes);
    sector += count;
    dst += bytes;
    ns -= count;
  }
  return true;
}

bool SdmmcBlockDevice::writeSectors(Sector_t sector, const uint8_t* src, size_t ns) {
  if (!_card || !src || ns == 0) return false;
  while (ns > 0) {
    const size_t count = ns > kMaxTransferSectors ? kMaxTransferSectors : ns;
    const size_t bytes = count * kSectorSize;
    memcpy(_dmaBuffer, src, bytes);
    if (sdmmc_write_sectors(static_cast<sdmmc_card_t*>(_card), _dmaBuffer, sector, count) != ESP_OK) return false;
    sector += count;
    src += bytes;
    ns -= count;
  }
  return true;
}

Sector_t SdmmcBlockDevice::sectorCount() {
  return _card ? static_cast<Sector_t>(static_cast<sdmmc_card_t*>(_card)->csd.capacity) : 0;
}

#endif  // XTEINK_X4_PRO
