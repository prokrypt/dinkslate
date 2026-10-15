#pragma once

// Native ESP-IDF SDMMC block device for SdFat, for the Xteink X4 Pro whose SD slot is
// wired for 1-bit SDMMC (the card is silent to SPI-mode CMD0). Ported from
// freeink-sdk's SdmmcBlockDevice (MIT, FreeInk SDK Contributors), hardware-validated
// there on the X4 Pro. Needs SdFat built with -DUSE_BLOCK_DEVICE_INTERFACE=1.

#ifdef XTEINK_X4_PRO

#include <SdFat.h>

class SdmmcBlockDevice : public FsBlockDeviceInterface {
 public:
  bool begin();
  void end() override;

  bool isBusy() override { return false; }
  bool readSector(Sector_t sector, uint8_t* dst) override { return readSectors(sector, dst, 1); }
  bool readSectors(Sector_t sector, uint8_t* dst, size_t ns) override;
  bool writeSector(Sector_t sector, const uint8_t* src) override { return writeSectors(sector, src, 1); }
  bool writeSectors(Sector_t sector, const uint8_t* src, size_t ns) override;
  Sector_t sectorCount() override;
  bool syncDevice() override { return true; }

 private:
  static constexpr size_t kSectorSize = 512;
  static constexpr size_t kMaxTransferSectors = 8;

  // sdmmc_card_t is a typedef of an anonymous struct and can't be forward-declared.
  void* _card = nullptr;
  uint8_t* _dmaBuffer = nullptr;
};

#endif  // XTEINK_X4_PRO
