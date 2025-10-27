#ifndef SDFAT_H
#define SDFAT_H

#include "misc.h"
#include <cstddef>
#include <filesystem>
#include <memory>
#include <stdint.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define O_RDONLY 0X00
#define O_WRONLY 0X01
#define O_APPEND 0X08
#define O_CREAT 0x10
#define O_WRITE O_WRONLY

#define SD_SCK_MHZ(maxMhz) (1000000UL * (maxMhz))

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

// TODO: Maybe use string stream
struct File_s {
  std::vector<String> lines;
};

// Less restrictive than normal FAT file
class FsFile {
private:
  FsFile(std::shared_ptr<File_s> file) : file(file) {}

  std::shared_ptr<File_s> file;

public:
  FsFile() : file(nullptr) {}

  size_t println(const String &str);
  bool close();

  friend class SdFs;
};

// Less restrictive than normal FAT
class SdFs {
private:
  bool began = false;

  std::unordered_map<std::filesystem::path, std::shared_ptr<File_s>> files;
  std::unordered_set<std::filesystem::path> dirs;

  bool mkdir(const std::filesystem::path &path, bool pFlag);
  bool exists(const std::filesystem::path &path) const;

public:
  bool begin(SdSpiConfig spiConfig);

  bool mkdir(const String &path, bool pFlag = true);
  bool exists(const String &path) const;

  FsFile open(const String &path, oflag_t oflag = O_RDONLY);
};
#endif
