idf_component_register(
  SRCS "main.c"
  REQUIRES esp_wifi esp_event esp_netif nvs_flash sht4x
)
