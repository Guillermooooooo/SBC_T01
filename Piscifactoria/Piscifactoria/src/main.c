#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include <string.h>
#include "nvs_flash.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "ssd1306.h"
#include "esp_spiffs.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_sntp.h"
#include "esp_ota_ops.h"
#include "esp_sleep.h"

#define OTA_URL "https://gitlab.etsisi.upm.es/bq0063/sbc/-/raw/main/firmware.bin?ref_type=heads&inline=false"
#define GPIO_DS18B20_0       (CONFIG_ONE_WIRE_GPIO)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD        (5000)   // milliseconds
#define WIFI_SSID      "jorgevg"
#define WIFI_PASS      "sbc12345678"
#define SERVER_URL     "http://demo.thingsboard.io/api/v1/v6tDz1vYMWbEdRY1pyHk/telemetry"
#define SENSOR_CHANNEL ADC1_CHANNEL_0 // Cambiar si usas otro pin ADC (G34)
#define OFFSET 0.0 // Ajuste de calibración, en mV
#define SCREEN_WIDTH 128
#define LIQUID_LEVEL_SENSOR_PIN GPIO_NUM_32  // Pin del sensor
#define PUMP_CONTROL_PIN GPIO_NUM_25         // Pin para controlar la bomba
#define HEATER_CONTROL_PIN GPIO_NUM_33         // Pin para controlar la bomba

#define FONT_WIDTH 6  
#define ADC_WIDTH ADC_WIDTH_BIT_12    // Resolución de 12 bits (0-4095)
#define DEFAULT_VREF 1100             // Vref por defecto en mV
#define OTA_UPDATE_HOUR 3

SSD1306_t dev;
static const char *TAG = "Piscifactoria";

void save_firmware_version(const char *version) {
    FILE* f = fopen("/data/firmware_version.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Error al abrir el archivo para guardar la versión");
        return;
    }
    fprintf(f, "Versión del firmware: %s", version);
    fclose(f);
}
void log_firmware_version() {
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    if (running_partition == NULL) {
        ESP_LOGE(TAG, "No se pudo obtener la partición actual");
        return;
    }
       char text[24];
    memset(text, 0x20, sizeof(text));
    snprintf(text, sizeof(text), " V.%s", running_partition->label );

    ssd1306_display_text(&dev, 7, text, 24, false);
}
esp_err_t init_spiffs() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/data",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}
static void event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Conectando a WiFi...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGE(TAG, "WiFi desconectado. Reintentando...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conexión WiFi establecida, IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}
static void wifi_init() {
    ESP_LOGI(TAG, "Inicializando NVS para WiFi...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS no tiene páginas libres o es una versión nueva, re-inicializando...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Inicializando WiFi...");
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}
void configSEN0204(){
        gpio_config_t sensor_config = {
        .pin_bit_mask = (1ULL << LIQUID_LEVEL_SENSOR_PIN), // Máscara del pin
        .mode = GPIO_MODE_INPUT,                          // Modo de entrada
        .pull_up_en = GPIO_PULLUP_DISABLE,                // Sin resistencia pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,            // Sin resistencia pull-down
        .intr_type = GPIO_INTR_DISABLE                    // Sin interrupciones
    };
    gpio_config(&sensor_config);

    // Configuración del pin de la bomba como salida
    esp_rom_gpio_pad_select_gpio(PUMP_CONTROL_PIN); // Habilitamos el pin
    gpio_set_direction(PUMP_CONTROL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP_CONTROL_PIN, 0); // Aseguramos que la bomba esté apagada inicialmente

}
void configHeater(){
    // Configuración del pin de la bomba como salida
    esp_rom_gpio_pad_select_gpio(HEATER_CONTROL_PIN); // Habilitamos el pin
    gpio_set_direction(HEATER_CONTROL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(HEATER_CONTROL_PIN, 0); // Aseguramos que la bomba esté apagada inicialmente

}
void send_temperature_value(float temperature_value) {
    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    ESP_LOGI(TAG, "ENTRANDO A HTTP");
    char json_data[50];
    sprintf(json_data, "{\"temp\":%.1f}", temperature_value);
    ESP_LOGI(TAG, "JSON");
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_data, strlen(json_data));
     ESP_LOGI(TAG, "HEADER");
    esp_err_t err = esp_http_client_perform(client);
     ESP_LOGI(TAG, "err");
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}
void send_SEN0165(float oxidacion_value) {
    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    ESP_LOGI(TAG, "ENTRANDO A HTTP");
    char json_data[50];
    sprintf(json_data, "{\"oxidacion\":%.1f}", oxidacion_value);
    ESP_LOGI(TAG, "JSON");
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_data, strlen(json_data));
     ESP_LOGI(TAG, "HEADER");
    esp_err_t err = esp_http_client_perform(client);
     ESP_LOGI(TAG, "err");
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}
void send_SEN0204_value(int sensor) {
    esp_http_client_config_t config = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    ESP_LOGI(TAG, "ENTRANDO A HTTP");
    char json_data[50];
    sprintf(json_data, "{\"sen0204\":%d}", sensor);
    ESP_LOGI(TAG, "JSON");
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_data, strlen(json_data));
     ESP_LOGI(TAG, "HEADER");
    esp_err_t err = esp_http_client_perform(client);
     ESP_LOGI(TAG, "err");
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}
void handle_ds18b20(DS18B20_Info *device, int *sample_count, int *errors_count) {
    ds18b20_convert_all(device->bus);
    ds18b20_wait_for_conversion(device);

    float readings = 0;
    DS18B20_ERROR errors = ds18b20_read_temp(device, &readings);

    printf("\nTemperature readings (degrees C): sample %d\n", ++(*sample_count));
    if (errors != DS18B20_OK) {
        ++(*errors_count);
    }
    printf("  1: %.1f    %d errors\n", readings, *errors_count);

    // Mostrar en OLED
    char text[24];
    memset(text, 0x20, sizeof(text));
    snprintf(text, sizeof(text), "   TEMP: %.2lf", readings);
    ssd1306_display_text(&dev, 0, text, 24, false);
    if(readings<20.0){
         printf("Temperatura baja. Activando calentador...\n");
        gpio_set_level(HEATER_CONTROL_PIN, 0); // Encender la bomba
    }else{
        printf("Temperatura normal\n");
        gpio_set_level(HEATER_CONTROL_PIN, 1); // Encender la bomba
    }
       

    // Enviar valor de temperatura
    send_temperature_value(readings);
}
void handleSEN0204(){
    int sensor_state = gpio_get_level(LIQUID_LEVEL_SENSOR_PIN);
    send_SEN0204_value(sensor_state);
    if (sensor_state == 0) {
    printf("Nivel de agua muy bajo. Activando bomba...\n");
       gpio_set_level(PUMP_CONTROL_PIN, 0); // Encender la bomba
    } else {
        printf("Nivel de agua correcto. Apagando bomba...\n");
        gpio_set_level(PUMP_CONTROL_PIN, 1); // Apagar la bomba
    }
    char text[24];
    memset(text, 0x20, sizeof(text));
    snprintf(text, sizeof(text), " N.AGUA: %s", (sensor_state == 0) ? "BAJO" : "NORMAL");

    ssd1306_display_text(&dev, 2, text, 24, false);
    
}
void handleSEN0165(){

    int raw_value = adc1_get_raw(SENSOR_CHANNEL); // Leer ADC
    printf("VALOR RAW ---> %d" , raw_value);
    float voltage = (raw_value / 4095.0) * 3300; // Convertir a mV
    float orpValue = voltage + OFFSET; // Aplicar calibración

    char text[24];
    if (orpValue > 700) {
        int len = strlen("Alta");
        int padding =  (12+len) / 2; // Calcula el número de espacios a la izquierda
        memset(text, 0x20, sizeof(text)); // Rellena todo con espacios
        snprintf(text + padding, sizeof(text) - padding, "Alta"); // Inserta el texto centrado
        ssd1306_display_text(&dev, 4, text, 24, false);
        
        memset(text, 0x20, sizeof(text));
        len = strlen("oxidacion");
        padding = len / 2;
        snprintf(text + padding, sizeof(text) - padding, "oxidacion");
        ssd1306_display_text(&dev, 5, text, 24, false);
        
    } else if (orpValue > 400) {
        int len = strlen("Optimo");
        int padding = (12+len) / 2;
        memset(text, 0x20, sizeof(text));
        snprintf(text + padding, sizeof(text) - padding, "Optimo");
        ssd1306_display_text(&dev, 4, text, 24, false);
             memset(text, 0x20, sizeof(text));
        len = strlen(" ");
        padding = (12+len) / 2;
        snprintf(text + padding, sizeof(text) - padding, " ");
        ssd1306_display_text(&dev, 5, text, 24, false);
        
    } else if (orpValue > 200) {
        int len = strlen("Moderada");
        int padding = ( len) / 2;
        memset(text, 0x20, sizeof(text));
        snprintf(text + padding, sizeof(text) - padding, "Moderada");
        ssd1306_display_text(&dev, 4, text, 24, false);
        
        memset(text, 0x20, sizeof(text));
        len = strlen("oxidacion");
        padding = ( len) / 2;
        snprintf(text + padding, sizeof(text) - padding, "oxidacion");
        ssd1306_display_text(&dev, 5, text, 24, false);
    } else if (orpValue > 0) {
        int len = strlen("Riesgo");
        int padding = ( len+5) / 2;
        memset(text, 0x20, sizeof(text));
        snprintf(text + padding, sizeof(text) - padding, "Riesgo");
        ssd1306_display_text(&dev, 4, text, 24, false);
        memset(text, 0x20, sizeof(text));
        len = strlen(" ");
        padding = (12+len) / 2;
        snprintf(text + padding, sizeof(text) - padding, " ");
        ssd1306_display_text(&dev, 5, text, 24, false);
    
        
} else {
    int len = strlen("Reduccion");
    int padding = (len) / 2;
    memset(text, 0x20, sizeof(text));
    snprintf(text + padding, sizeof(text) - padding, "Reduccion");
    ssd1306_display_text(&dev, 4, text, 24, false);
    
    memset(text, 0x20, sizeof(text));
    len = strlen("alta");
    padding = (10+len) / 2;
    snprintf(text + padding, sizeof(text) - padding, "alta");
    ssd1306_display_text(&dev, 5, text, 24, false);
}

    send_SEN0165(orpValue);
}
void configOLED(){
      #if CONFIG_I2C_INTERFACE
    ESP_LOGI(TAG, "INTERFACE is i2c");
    ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
    ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
    ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    #endif // CONFIG_I2C_INTERFACE

    #if CONFIG_SPI_INTERFACE
        ESP_LOGI(TAG, "INTERFACE is SPI");
        ESP_LOGI(TAG, "CONFIG_MOSI_GPIO=%d", CONFIG_MOSI_GPIO);
        ESP_LOGI(TAG, "CONFIG_SCLK_GPIO=%d", CONFIG_SCLK_GPIO);
        ESP_LOGI(TAG, "CONFIG_CS_GPIO=%d", CONFIG_CS_GPIO);
        ESP_LOGI(TAG, "CONFIG_DC_GPIO=%d", CONFIG_DC_GPIO);
        ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
        spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO);
    #endif // CONFIG_SPI_INTERFACE

    #if CONFIG_SSD1306_128x64
        ESP_LOGI(TAG, "Panel is 128x64");
        ssd1306_init(&dev, 128, 64);
        int maxPage = 8;
    #endif // CONFIG_SSD1306_128x64

    #if CONFIG_SSD1306_128x32
        ESP_LOGI(TAG, "Panel is 128x32");
        ssd1306_init(&dev, 128, 32);
        int maxPage = 4;
    #endif // CONFIG_SSD1306_128x32

    // Inicializar el ADC
    //init_adc();

    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
}
void back_to_factory() {
    esp_partition_iterator_t pi;
    const esp_partition_t* factory;
    esp_err_t err;

    pi = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, "factory");
    if (pi == NULL) {
        ESP_LOGE(TAG, "No se encontró la partición de fábrica");
        return;
    }

    factory = esp_partition_get(pi);
    esp_partition_iterator_release(pi);

    err = esp_ota_set_boot_partition(factory);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar la partición de arranque de fábrica");
    } else {
        ESP_LOGI(TAG, "Partición de arranque configurada a 'factory'");
        esp_restart();
    }
}
void perform_ota_update() {
    // Inicializa SPIFFS
    esp_err_t ret = init_spiffs();
    if (ret != ESP_OK) {
        return; // Maneja el error según sea necesario
    }

    // Abrir el archivo del certificado
    FILE *f = fopen("/data/ca_cert.pem", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open certificate file");
        return;
    }

    fseek(f, 0, SEEK_END);
    long length = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *cert_buf = malloc(length + 1);
    fread(cert_buf, 1, length, f);
    cert_buf[length] = '\0'; 
    fclose(f);

    // Configura el cliente HTTP para OTA
   esp_http_client_config_t config = {
        .url = OTA_URL,
        .cert_pem = cert_buf,
        
    };
    esp_https_ota_config_t configOTA={
        .http_config=&config
    };
    esp_err_t ota_res = esp_https_ota(&configOTA);
    if (ota_res == ESP_OK) {
        ESP_LOGI(TAG, "OTA update performed successfully");
        
        // Guardar la versión del firmware
        save_firmware_version("v1.0.2");
        log_firmware_version();
        
        // Confirmar la partición de arranque para que se quede con el nuevo firmware
        ota_res = esp_ota_mark_app_valid_cancel_rollback();
        if (ota_res == ESP_OK) {
            ESP_LOGI(TAG, "Nuevo firmware confirmado exitosamente");
            esp_restart(); 
        } else {
            ESP_LOGE(TAG, "Error al confirmar el nuevo firmware (%s)", esp_err_to_name(ota_res));
        }
    } else {
        ESP_LOGE(TAG, "OTA update failed (%s)", esp_err_to_name(ota_res));
    }

    free(cert_buf);
    esp_vfs_spiffs_unregister(NULL);
}
void initialize_sntp() {
    ESP_LOGI(TAG, "Iniciando SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
}
bool check_update_time() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year < (2022 - 1900)) {
        ESP_LOGI(TAG, "Esperando sincronización con NTP...");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    return (timeinfo.tm_hour == OTA_UPDATE_HOUR && timeinfo.tm_min == 0);
}

_Noreturn void app_main()
{
    adc1_config_width(ADC_WIDTH_BIT_12); // Configurar ADC de 12 bits
    adc1_config_channel_atten(SENSOR_CHANNEL, ADC_ATTEN_DB_11); // Atenuación de entrada
    configOLED();
    configSEN0204();
    configHeater();
    esp_log_level_set("*", ESP_LOG_INFO);

    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code
    OneWireBus_ROMCode device_rom_codes = {0};
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_devices, rom_code_s);
        device_rom_codes= search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    OneWireBus_ROMCode rom_code;
    owb_status status = owb_read_rom(owb, &rom_code);
    if (status == OWB_STATUS_OK)
        {
            char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
            owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
            printf("Single device %s present\n", rom_code_s);
    }
    else
        {
            printf("An error occurred reading ROM code: %d", status);
        }    
    DS18B20_Info * device = {0};
    DS18B20_Info * ds18b20_info = ds18b20_malloc(); 
    device = ds18b20_info;
    
    ds18b20_init_solo(ds18b20_info, owb);         
    bool parasitic_power = false;
    ds18b20_check_for_parasite_power(owb, &parasitic_power);
    if (parasitic_power) {
        printf("Parasitic-powered devices detected");
    }

    // In parasitic-power mode, devices cannot indicate when conversions are complete,
    // so waiting for a temperature conversion must be done by waiting a prescribed duration
    owb_use_parasitic_power(owb, parasitic_power);


#ifdef CONFIG_ENABLE_STRONG_PULLUP_GPIO
    // An external pull-up circuit is used to supply extra current to OneWireBus devices
    // during temperature conversions.
    owb_use_strong_pullup_gpio(owb, CONFIG_STRONG_PULLUP_GPIO);
#endif

    // OTA 

    log_firmware_version();

    int errors_count = {0};
    int sample_count = 0;
    if (num_devices > 0)
    {
        TickType_t last_wake_time = xTaskGetTickCount();
    
    // Declarar la variable estática fuera del bucle while
    static TickType_t last_sleep_time = 0;

    while (1) {
        wifi_init();
        vTaskDelay(10000 / portTICK_PERIOD_MS);  // Espera 10 segundos

        handle_ds18b20(device, &sample_count, &errors_count);
        handleSEN0204();
        handleSEN0165();
        
        if (check_update_time()) {
            ESP_LOGI(TAG, "ACTUALIZANDO OTA...");
            perform_ota_update();
        }

        // Comprobación para entrar en suspensión cada 5 segundos
        TickType_t current_time = xTaskGetTickCount(); // Tiempo actual en ticks

        if ((current_time - last_sleep_time) >= (5000 / portTICK_PERIOD_MS)) { 
            ESP_LOGI(TAG, "Entrando en modo de suspensión profunda...");
            esp_sleep_enable_timer_wakeup(5 * 1000000);  // 5 segundos en microsegundos
            esp_deep_sleep_start();
            
            last_sleep_time = current_time; // Actualizamos el último tiempo de suspensión
        }
    }

    }
    else
    {
        printf("\nNo DS18B20 devices detected!\n");
    }

    ds18b20_free(&device);
    owb_uninitialize(owb);
    printf("Restarting now.\n");
    fflush(stdout);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_restart();
}