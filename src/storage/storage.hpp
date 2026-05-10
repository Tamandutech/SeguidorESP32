#ifndef STORAGE_HPP
#define STORAGE_HPP

#include <atomic>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include "esp_err.h"
#include "esp_vfs_fat.h"

class Storage {
public:
  static Storage *getInstance();

  bool is_mounted();

  esp_err_t mount_storage(std::string mount_path);

  esp_err_t list_files();

  esp_err_t save_data(std::string file_path, char *data_buffer,
                      size_t data_size, const char *mode = "wb");

  esp_err_t load_data(std::string file_path, char *data_buffer,
                      size_t data_size);

  esp_err_t load_data(std::string file_path, char **data_buffer,
                      size_t *file_size);

  esp_err_t delete_data(std::string file_path);

  template <typename DataType>
  esp_err_t write(DataType data_value, std::string file_path);

  template <typename DataType>
  esp_err_t read(DataType &data_value, std::string file_path);

  template <typename ElementType>
  esp_err_t write_vector(const std::vector<ElementType> &vector_data,
                         std::string file_path);

  template <typename ElementType>
  esp_err_t read_vector(std::vector<ElementType> &vector_data,
                        std::string file_path);

  bool file_exists(std::string file_path);

private:
  static std::atomic<Storage *> instance;
  static std::mutex             instance_mutex;
  static std::string            logger_tag;

  wl_handle_t                      wear_leveling_handle;
  std::string                      mount_point;
  esp_vfs_fat_sdmmc_mount_config_t fat_mount_config;
  bool                             mount_status;
  mutable std::mutex               fs_mutex;

  Storage();
};

#endif // STORAGE_HPP
