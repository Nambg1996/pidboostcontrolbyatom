#include <esp_spi_flash.h>
#include <string.h>

const int FLASH_SIZE = 10 * 6; // 10 strings of 6 characters
const int FLASH_SECTOR = 1;
const int FLASH_ADDRESS = 0x100000; // starting address of the flash sector
const char *values[10] = {
  "cd:09:fe:8f:7f:1a",
  "cd:09:fe:8f:7f:1b",
  "cd:09:fe:8f:7f:1c",
  "cd:09:fe:8f:7f:1d",
  "cd:09:fe:8f:7f:1e",
  "cd:09:fe:8f:7f:1f",
  "cd:09:fe:8f:7f:1g",
  "cd:09:fe:8f:7f:1h",
  "cd:09:fe:8f:7f:1j",
  "cd:09:fe:8f:7f:1k"
};

void saveValueToFlash() {
  // Erase the flash sector
  esp_err_t result = spi_flash_erase_sector(FLASH_SECTOR);
  if (result != ESP_OK) {
    Serial.println("Error erasing flash sector");
    return;
  }

  // Write the values to the flash
  for (int i = 0; i < 10; i++) {
    result = spi_flash_write(FLASH_ADDRESS + i * 6, (uint32_t *)values[i], 6);
    if (result != ESP_OK) {
      Serial.println("Error writing to flash");
      return;
    }
  }
}

void getFlashMemory() {
  char buffer[6];
  for (int i = 0; i < 10; i++) {
    spi_flash_read(FLASH_ADDRESS + i * 6, (uint32_t *)buffer, 6);
    Serial.println(buffer);
  }
}
