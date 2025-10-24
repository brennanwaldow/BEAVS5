#ifndef SDFAT_H
#define SDFAT_H

#include <cstddef>
#include <stdint.h>
#include <string>

#define O_RDONLY 0X00
#define O_WRONLY 0X01
#define O_APPEND 0X08
#define O_CREAT 0x10
#define O_WRITE O_WRONLY

#define SD_SCK_MHZ(maxMhz) (1000000UL * (maxMhz))

typedef std::string String;
typedef uint8_t SdCsPin_t;

typedef uint8_t oflag_t;

const uint8_t SHARED_SPI = 0;
const uint8_t DEDICATED_SPI = 1;

class SdSpiConfig {
public:
  SdSpiConfig(SdCsPin_t cs, uint8_t opt, uint32_t maxSpeed)
      : csPin(cs), options(opt), maxSck(maxSpeed) {}
  explicit SdSpiConfig(SdCsPin_t cs) : csPin(cs) {}

  const SdCsPin_t csPin;
  const uint8_t options = SHARED_SPI;
  const uint32_t maxSck = SD_SCK_MHZ(50);
};

class FsFile {
  size_t println(const String &str);
  bool close();
};

class SdFs {
  bool begin(SdCsPin_t csPin);

  bool mkdir(const String &path, bool pFlag = true);
  bool exists(const String &path);

  FsFile open(const String &path, oflag_t oflag = O_RDONLY);
};
#endif
