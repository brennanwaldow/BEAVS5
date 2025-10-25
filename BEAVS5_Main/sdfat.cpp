#include "sdfat.h"

#include <cassert>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <unordered_map>

size_t FsFile::println(const String &str) {
  if (file == nullptr) {
    return 0;
  }

  file->lines.push_back(str);

  // \r\n I think is the correct ending
  return str.length() + 2;
}

bool FsFile::close() {
  file = nullptr;
  return true;
}

bool SdFs::begin(SdSpiConfig spiConfig) {
  assert(!began);
  assert(spiConfig.csPin == 17);
  assert(spiConfig.options == DEDICATED_SPI);
  assert(spiConfig.maxSck == SD_SCK_MHZ(50));

  began = true;
  return true;
}

bool SdFs::mkdir(const String &path_str, bool pFlag) {
  assert(began);

  std::filesystem::path path(path_str.c_str());

  return mkdir(path, pFlag);
}

bool SdFs::mkdir(const std::filesystem::path &path, bool pFlag) {
  if (exists(path)) {
    return false;
  }

  if (path.has_parent_path()) {
    if (!dirs.contains(path.parent_path())) {
      if (!pFlag || !mkdir(path.parent_path(), true)) {
        return false;
      }
    }
  }

  dirs.insert(path);

  return true;
}

bool SdFs::exists(const String &path_str) const {
  assert(began);

  std::filesystem::path path(path_str.c_str());

  return exists(path);
}

bool SdFs::exists(const std::filesystem::path &path) const {
  return dirs.contains(path) || files.contains(path);
}

FsFile SdFs::open(const String &path_str, oflag_t oflag) {
  assert(began);

  // Currently files are only writable
  if (!(oflag & O_WRONLY)) {
    return FsFile();
  }

  std::filesystem::path path(path_str.c_str());

  if (dirs.contains(path)) {
    return FsFile();
  }

  auto index = files.find(path);
  if (index == files.end()) {
    if (oflag & O_CREAT) {
      std::shared_ptr<File_s> file;
      files.insert({path, file});
      return file;
    } else {
      return FsFile();
    }
  }

  return index->second;
}
