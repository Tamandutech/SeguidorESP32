#ifndef STORAGE_HPP
#define STORAGE_HPP

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

class Storage {
public:
  static Storage *getInstance() {
    ESP_LOGD("Storage", "Acquiring instance...");

    Storage *storage_instance = instance.load(std::memory_order_acquire);
    if(!storage_instance) {
      ESP_LOGD("Storage", "Instance does not exist, creating...");

      std::lock_guard<std::mutex> instance_lock(instance_mutex);
      storage_instance = instance.load(std::memory_order_relaxed);
      if(!storage_instance) {
        storage_instance = new Storage();
        instance.store(storage_instance, std::memory_order_release);
      }
    }

    return storage_instance;
  };

  esp_err_t mount_storage(std::string mount_path) {
    if(is_mounted()) {
      ESP_LOGE(logger_tag.c_str(),
               "FATFS already mounted at %s. To mount at %s, first unmount the "
               "previous path.",
               mount_point.c_str(), mount_path.c_str());
      return ESP_FAIL;
    }

    mount_point = mount_path.at(0) == '/' ? mount_path : "/" + mount_path;

    esp_err_t mount_result = esp_vfs_fat_spiflash_mount_rw_wl(
        mount_point.c_str(), "storage", &fat_mount_config,
        &wear_leveling_handle);
    if(mount_result != ESP_OK) {
      ESP_LOGE(logger_tag.c_str(), "Failed to mount FATFS (%s)",
               esp_err_to_name(mount_result));
      mount_status = false;
      return ESP_FAIL;
    } else {
      ESP_LOGI(logger_tag.c_str(), "FATFS mounted successfully at %s",
               mount_point.c_str());
      mount_status = true;
      return ESP_OK;
    }
  }

  esp_err_t list_files() {
    if(!is_mounted()) return ESP_FAIL;

    DIR           *directory_handle;
    struct dirent *directory_entry;

    directory_handle = opendir(mount_point.c_str());
    if(directory_handle) {
      ESP_LOGD(logger_tag.c_str(),
               "Listing directory %s:", mount_point.c_str());
      while((directory_entry = readdir(directory_handle)) != NULL) {
        ESP_LOGI(logger_tag.c_str(), "File: %s", directory_entry->d_name);
      }
      closedir(directory_handle);
    }

    return ESP_OK;
  }

  esp_err_t save_data(std::string file_path, char *data_buffer,
                      size_t data_size, const char *mode = "wb") {
    if(!is_mounted()) return ESP_FAIL;

    std::string full_path   = mount_point + "/" + file_path;
    FILE       *file_handle = fopen(full_path.c_str(), mode);

    if(file_handle == NULL) {
      ESP_LOGE(logger_tag.c_str(), "Failed to open file %s for writing",
               file_path.c_str());
      return ESP_FAIL;
    }

    fwrite(data_buffer, data_size, 1, file_handle);
    ESP_LOGD(logger_tag.c_str(), "Written %s, %zu bytes", file_path.c_str(),
             data_size);

    fclose(file_handle);

    return ESP_OK;
  }

  esp_err_t load_data(std::string file_path, char *data_buffer,
                      size_t data_size) {
    if(!is_mounted()) return ESP_FAIL;

    std::string full_path   = mount_point + "/" + file_path;
    FILE       *file_handle = fopen(full_path.c_str(), "r");

    if(file_handle == NULL) {
      ESP_LOGE(logger_tag.c_str(), "Failed to open file %s for reading",
               file_path.c_str());
      return ESP_FAIL;
    }

    fread(data_buffer, data_size, 1, file_handle);

    ESP_LOGD(logger_tag.c_str(), "Read %s, %zu bytes", file_path.c_str(),
             data_size);

    fclose(file_handle);

    return ESP_OK;
  }

  esp_err_t load_data(std::string file_path, char **data_buffer,
                      size_t *file_size) {
    if(!is_mounted()) return ESP_FAIL;

    std::string full_path   = mount_point + "/" + file_path;
    FILE       *file_handle = fopen(full_path.c_str(), "rb");

    if(file_handle == NULL) {
      ESP_LOGE(logger_tag.c_str(), "Failed to open file %s for reading",
               file_path.c_str());
      return ESP_FAIL;
    }

    fseek(file_handle, 0, SEEK_END);
    (*file_size) = ftell(file_handle);

    free(*data_buffer);
    *data_buffer = (char *)malloc((*file_size) * sizeof(char));

    fseek(file_handle, 0, SEEK_SET);

    fread(*data_buffer, *file_size, 1, file_handle);

    ESP_LOGD(logger_tag.c_str(), "Read %s, %zu bytes", file_path.c_str(),
             *file_size);

    fclose(file_handle);

    return ESP_OK;
  }

  esp_err_t delete_data(std::string file_path) {
    if(!is_mounted()) return ESP_FAIL;

    std::string full_path = mount_point + "/" + file_path;
    if(remove(full_path.c_str()) != 0) {
      ESP_LOGE(logger_tag.c_str(), "Failed to remove file %s",
               file_path.c_str());
      return ESP_FAIL;
    }

    ESP_LOGI(logger_tag.c_str(), "Removed %s", file_path.c_str());
    return ESP_OK;
  }

  // Template methods for convenience
  template <typename DataType>
  esp_err_t write(DataType data_value, std::string file_path) {
    return save_data(file_path, reinterpret_cast<char *>(&data_value),
                     sizeof(DataType));
  }

  template <typename DataType>
  esp_err_t read(DataType &data_value, std::string file_path) {
    return load_data(file_path, reinterpret_cast<char *>(&data_value),
                     sizeof(DataType));
  }

  // Vector serialization helpers
  template <typename ElementType>
  esp_err_t write_vector(const std::vector<ElementType> &vector_data,
                        std::string file_path) {
    if(!is_mounted()) return ESP_FAIL;

    std::string full_path   = mount_point + "/" + file_path;
    FILE       *file_handle = fopen(full_path.c_str(), "wb");

    if(file_handle == NULL) {
      ESP_LOGE(logger_tag.c_str(), "Failed to open file %s for writing",
               file_path.c_str());
      return ESP_FAIL;
    }

    size_t vector_size = vector_data.size();
    fwrite(&vector_size, sizeof(size_t), 1, file_handle);
    if(vector_size > 0) {
      fwrite(vector_data.data(), sizeof(ElementType), vector_size, file_handle);
    }

    ESP_LOGD(logger_tag.c_str(), "Written vector %s, %zu elements",
             file_path.c_str(), vector_size);

    fclose(file_handle);
    return ESP_OK;
  }

  template <typename ElementType>
  esp_err_t read_vector(std::vector<ElementType> &vector_data,
                        std::string file_path) {
    if(!is_mounted()) return ESP_FAIL;

    std::string full_path   = mount_point + "/" + file_path;
    FILE       *file_handle = fopen(full_path.c_str(), "rb");

    if(file_handle == NULL) {
      ESP_LOGD(logger_tag.c_str(), "File %s does not exist, using defaults",
               file_path.c_str());
      return ESP_FAIL;
    }

    size_t vector_size = 0;
    fread(&vector_size, sizeof(size_t), 1, file_handle);

    if(vector_size > 0) {
      vector_data.resize(vector_size);
      fread(vector_data.data(), sizeof(ElementType), vector_size, file_handle);
    } else {
      vector_data.clear();
    }

    ESP_LOGD(logger_tag.c_str(), "Read vector %s, %zu elements",
             file_path.c_str(), vector_size);

    fclose(file_handle);
    return ESP_OK;
  }

  // Check if file exists
  bool file_exists(std::string file_path) {
    if(!is_mounted()) return false;

    std::string full_path   = mount_point + "/" + file_path;
    FILE       *file_handle = fopen(full_path.c_str(), "r");
    if(file_handle != NULL) {
      fclose(file_handle);
      return true;
    }
    return false;
  }

private:
  static std::atomic<Storage *> instance;
  static std::mutex             instance_mutex;
  static std::string            logger_tag;

  wl_handle_t                      wear_leveling_handle;
  std::string                      mount_point;
  esp_vfs_fat_sdmmc_mount_config_t fat_mount_config;
  bool                             mount_status;

  Storage() {
    wear_leveling_handle = WL_INVALID_HANDLE;
    mount_point          = "/data";
    logger_tag           = "Storage";

    fat_mount_config.format_if_mount_failed = true;
    fat_mount_config.max_files              = 5;
    fat_mount_config.allocation_unit_size   = CONFIG_WL_SECTOR_SIZE;
    mount_status                            = false;
  }

  bool is_mounted() {
    if(!mount_status) {
      ESP_LOGD(logger_tag.c_str(), "FATFS not mounted.");
    }

    return mount_status;
  }
};

// Static member definitions
std::atomic<Storage *> Storage::instance;
std::mutex             Storage::instance_mutex;
std::string            Storage::logger_tag;

#endif // STORAGE_HPP
