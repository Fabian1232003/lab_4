
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "nvs_flash.h"

#include "driver/gpio.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_rom_sys.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "driver/ledc.h"

uint32_t uids_validos[] = {
    0xCBD43207
};
//////////////////lcd en modo 4 bits con PCF8574//////////.///
 
uint64_t now, last, lastm =0;

uint8_t hora=0;

#define MISO 19
#define MOSI 16
#define clk 4 
#define CS 17
static spi_device_handle_t spi_dev;

#define led_azul 21

#define TDX_PIN  1   // TX0 hacia el PC (USB)
#define RDX_PIN  3   // RX0 desde el PC (USB)
#define led_r 22
#define led_v 2
#define buzzer 33

bool acceso= false;
bool rojo = false;

uint64_t last_rf, lastsec=0;

#define PCF8574_ADDR        0x20
#define I2C_MASTER_SCL_IO           32
#define I2C_MASTER_SDA_IO           26
#define I2C_PORT             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          470000

uint8_t uid[4]={0};
void ets_delay_us(uint32_t us);


void ds1307_write(uint8_t reg, uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
}

uint8_t ds1307_read(uint8_t reg){
    

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd); // repeated start
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &hora, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    return hora;

    
}
uint8_t dec_to_bcd(uint8_t val){
    return ((val / 10) << 4) | (val % 10);
}

void set_time_ds1307(uint8_t seg, uint8_t min, uint8_t hora, uint8_t dia, uint8_t mes, uint8_t año){
    ds1307_write(0x00, dec_to_bcd(seg));
    ds1307_write(0x01, dec_to_bcd(min));
    ds1307_write(0x02, dec_to_bcd(hora)); // bit 6 en 0 para formato 24 horas
    ds1307_write(0x04, dec_to_bcd(dia));
    ds1307_write(0x05, dec_to_bcd(mes));
    ds1307_write(0x06, dec_to_bcd(año));
}

 uint8_t bcd_to_dec(uint8_t val){
        return ((val >> 4) * 10) + (val & 0x0F);
    }

char* ver_hora(void){
    static char hora_str[16];

    uint8_t sec_bcd  = ds1307_read(0x00);
    uint8_t min_bcd  = ds1307_read(0x01);
    uint8_t hora_bcd = ds1307_read(0x02);

    uint8_t sec  = bcd_to_dec(sec_bcd & 0x7F);
    uint8_t min  = bcd_to_dec(min_bcd & 0x7F);
    uint8_t hora = bcd_to_dec(hora_bcd & 0x3F); // modo 24h

    sprintf(hora_str, "%02d:%02d:%02d", hora, min, sec);

    return hora_str;
}




void pcf_write(uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCF8574_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
}
void lcd_write(uint8_t data,uint8_t rs){
    // mascaras y demas para que funcione el nibbable
    uint8_t r = rs ? 0x80 : 0x00;

    uint8_t msb = (data >> 4) | r;
    pcf_write(msb);
    esp_rom_delay_us(1);
    pcf_write(msb | 0x20);
    esp_rom_delay_us(1);
    pcf_write(msb);
    esp_rom_delay_us(10);

    uint8_t lsb = (data & 0x0F) | r;
    pcf_write(lsb);
    esp_rom_delay_us(1);
    pcf_write(lsb | 0x20);
    esp_rom_delay_us(1);
    pcf_write(lsb);
    esp_rom_delay_us(10);
    
    
}

void reiniciar(void){
    vTaskDelay(pdMS_TO_TICKS(50));

    lcd_write(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write(0x03, 0);
    esp_rom_delay_us(150);

    lcd_write(0x03, 0);
    esp_rom_delay_us(150);

    lcd_write(0x02, 0);
    esp_rom_delay_us(150);

    lcd_write(0x28,0); // 4-bit, 2 lines, 5x8


    esp_rom_delay_us(50);

    lcd_write(0x08,0);
    esp_rom_delay_us(50);

    lcd_write(0x01,0); // reinicia pantalla y cursor
    
    esp_rom_delay_us(2000);
    lcd_write(0x06,0);// cursor icremental a la derecha no se mueve pantalla
    esp_rom_delay_us(50);
    lcd_write(0x0E,0);// display on, cursor on, blink off
    esp_rom_delay_us(50);

    lcd_write(0x02, 0);   // return home
    esp_rom_delay_us(2000);
    lcd_write(0x80, 0);   // DDRAM al inicio
    esp_rom_delay_us(2000);

}

void escribir_bloques_8(char *str, bool arriba) {
    uint8_t cmd_linea = arriba ? 0x80 : 0xC0;

    // 1. Posicionar el cursor al inicio de la línea correspondiente
    if (arriba) {
        lcd_write(0x02, 0);   // Return home
        vTaskDelay(pdMS_TO_TICKS(2)); // Tiempo físico para que el cursor regrese
    }
    lcd_write(cmd_linea, 0);   // DDRAM al inicio de la línea (0x80 o 0xC0)
    esp_rom_delay_us(200);

    // Preparar el buffer seguro
    char data_buffer[64];
    snprintf(data_buffer, sizeof(data_buffer), "%s", str);
    int longitud = strlen(data_buffer);

    // --- BLOQUE 1: Primeras 8 letras (Índices 0 al 7) ---
    for (int i = 0; i < longitud && i < 8; i++) {
        lcd_write(data_buffer[i], 1);
        esp_rom_delay_us(100); // Pequeño delay obligatorio entre caracteres
    }

    // 🔥 EL RESPIRO CRÍTICO: Si el texto sigue, le damos tiempo al LCD
    if (longitud > 8) {
        vTaskDelay(pdMS_TO_TICKS(3)); // 3 milisegundos para que el LCD vacíe su buffer interno
        
        // --- BLOQUE 2: Siguientes 8 letras (Índices 8 al 15) ---
        for (int i = 8; i < longitud && i < 16; i++) {
            lcd_write(data_buffer[i], 1);
            esp_rom_delay_us(100);
        }
    }
}

void escribir(char *str , bool arriba){
     // reinicia pantalla y cursor
    if (arriba ){
        lcd_write(0x02, 0);   // return home
        esp_rom_delay_us(200);
        lcd_write(0x80, 0);   // DDRAM al inicio
        esp_rom_delay_us(50);
        char data_buffer[64];
        sprintf( data_buffer,  "%s" ,str);

        for (int i =0;i<strlen(data_buffer);i++){
            lcd_write(data_buffer[i],1);
            
        }
    }
    else {
        esp_rom_delay_us(200);
        lcd_write(0xC0,0);// segunda linea 
        char data_buffer[64];
        sprintf( data_buffer,  "%s" ,str);

        for (int i =0;i<strlen(data_buffer);i++){
            lcd_write(data_buffer[i],1);
            
        }



    }
}

void rf_write(uint8_t address, uint8_t com) {
    uint8_t tx[2];

    tx[0] = ((address << 1) & 0x7E) ;   // bit7=0 write, bits6:1=addr, bit0=0
    tx[1] = com;

    spi_transaction_t t = {
        .flags = 0,
        .length = 16,
        .tx_buffer = tx
    };

    spi_device_transmit(spi_dev, &t);
}

uint8_t rf_read(uint8_t address)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = ((address << 1) & 0x7E) | 0x80;
    tx[1] = 0x00;

    spi_transaction_t t = {
        .flags = 0,
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(spi_dev, &t);

    return rx[1];
}
void iniciar_rf(void){
    rf_write(0x01, 0x0F);   // SoftReset
    vTaskDelay(pdMS_TO_TICKS(50));

    rf_write(0x01, 0x00);   // Idle
    rf_write(0x0A, 0x80);   // Flush FIFO

    rf_write(0x2A, 0x8D);   // TModeReg
    rf_write(0x2B, 0x3E);   // TPrescalerReg
    rf_write(0x2D, 30);     // TReloadRegL
    rf_write(0x2C, 0);      // TReloadRegH

    rf_write(0x15, 0x40);   // TxASKReg
    rf_write(0x11, 0x3D);   // ModeReg

    rf_write(0x26, 0x70);   // RFCfgReg, ganancia del receptor

    uint8_t v = rf_read(0x14);
    rf_write(0x14, v | 0x03);  // Activar TX1RFEn y TX2RFEn
}
void leer_rfid(void ){
    rf_write(0x0A,0x80); // reiniciar fifo 
    
    rf_write(0x09,0x26); // REQA en FIFO

    rf_write(0x0D,0x07); // 7 bits válidos
    rf_write(0x01,0x0C); // Transceive
    rf_write(0x0D,0x87); // StartSend = 1 y conserva los 7 bits

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t cantidad = rf_read(0x0A) & 0x7F; // bytes en FIFO después del REQA

    if (cantidad < 2) {
        uid[0] = 0x00;
        uid[1] = 0x00;
        uid[2] = 0x00;
        uid[3] = 0x00;
        rf_write(0x01, 0x00); // Idle
        return;
    }

    rf_write(0x01, 0x00);   // Idle
    rf_write(0x0A, 0x80);   // Limpiar FIFO
    
    rf_write(0x09, 0x93);   // anticollision nivel 1
    rf_write(0x09, 0x20);   // NVB

    rf_write(0x0D, 0x00);   // 8 bits
    rf_write(0x01, 0x0C);   // Transceive
    rf_write(0x0D, 0x80);   // StartSend

    vTaskDelay(pdMS_TO_TICKS(10));

    cantidad = rf_read(0x0A) & 0x7F; // bytes en FIFO

    if (cantidad >= 5) {
        uid[0] = rf_read(0x09); // UID byte 0
        uid[1] = rf_read(0x09); // UID byte 1
        uid[2] = rf_read(0x09); // UID byte 2
        uid[3] = rf_read(0x09); // UID byte 3
        rf_read(0x09);          // BCC (descartado)
    } else {
        uid[0] = 0x00;
        uid[1] = 0x00;
        uid[2] = 0x00;
        uid[3] = 0x00;
    }

    rf_write(0x01, 0x00); // Idle
}

#define DEVICE_NAME "PanelHMI"

static const char *TAG = "ESP32_NUS_fabian";

static uint8_t own_addr_type; // guarda la direeccion del dispositivo al anunciarse o conectarse 
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;// identificador de la conexion de blutoth activa 
static uint16_t nus_tx_val_handle; // handel de tx de la cracteristica de NUS 
static bool notify_enabled = false; // para saber si el usuario se suscribio a notify


/*
 * UUIDs oficiales del Nordic UART Service:
 * Service: 6e400001-b5a3-f393-e0a9-e50e24dcca9e
 * RX:      6e400002-b5a3-f393-e0a9-e50e24dcca9e
 * TX:      6e400003-b5a3-f393-e0a9-e50e24dcca9e
 *
 * En NimBLE se escriben en orden little-endian.
 */
static const ble_uuid128_t nus_service_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

static const ble_uuid128_t nus_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

static const ble_uuid128_t nus_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);


// revisa si el cliente se suscribio o no esta conectado 
static void nus_send_response(const char *msg)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !notify_enabled) {
        ESP_LOGW(TAG, "Cliente no conectado o no suscrito a TX Notify");
        return;
    }

    // convierte el string a un nbuf para enviarlo por BLE
    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));

    if (om == NULL) {
        ESP_LOGE(TAG, "No se pudo crear buffer BLE");
        return;
    }
    // envia la notificacion al cliente el buff creado arriba 
    int rc = ble_gatts_notify_custom(conn_handle, nus_tx_val_handle, om);

    if (rc != 0) {
        ESP_LOGE(TAG, "Error enviando notificacion: %d", rc);
    }
}

// con hanled e squine le envio os datos attr es el atributo accedido  ctxt es el contexto de acceso gatt aqui biene el mbuf
static int nus_rx_access_cb(uint16_t conn_handle_param,
                            uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt,
                            void *arg)

{
    //
    char data[17];
    uint16_t len = 0;
    // aqui recibimos el buuf copiamos maximo 63 bytes del mbuf y colocamos la cantidad de datos copiados en len 

    int rc = ble_hs_mbuf_to_flat(ctxt->om, data, sizeof(data) - 1, &len);

    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    data[len] = '\0'; // ahora data es un string C válido

    ESP_LOGI(TAG, "Dato recibido por BLE: %s", data);


    if (len > 0 && acceso) {
        lcd_write(0x01,0); // reinicia pantalla y cursor
        gpio_set_level(led_azul, 1);
        escribir(data, true);
        escribir(ver_hora(), false );

    }
    return 0;
}

    // char buf[32];
    // int valor = 1234;
    // snprintf(buf, sizeof(buf), "VAL=%d\r\n", valor);
    // nus_send_response(buf);

    // uint8_t vec[4] = { 0x01, 0x02, 0xAA, 0x55 };
    // nus_send_response_bin(vec, sizeof(vec)); 

// no hace nada es para que nibble no se buggee 
static int nus_tx_access_cb(uint16_t conn_handle_param,
                            uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt,
                            void *arg)
{
    return 0;
}

// api ble de NUS 

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &nus_rx_uuid.u,
                .access_cb = nus_rx_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &nus_tx_uuid.u,
                .access_cb = nus_tx_access_cb,
                .val_handle = &nus_tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0,
            }
        },
    },
    {
        0,
    }
};


static void ble_app_advertise(void);


// eventos de conexion y desconesion del BLE 
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Cliente conectado");
        } else {
            ESP_LOGW(TAG, "Conexion fallida, reanudando advertising");
            ble_app_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Cliente desconectado");
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        notify_enabled = false;
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == nus_tx_val_handle) {
            notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "TX Notify: %s", notify_enabled ? "ON" : "OFF");
        }
        return 0;
// evento de advertising completo para volver a anunciarse
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_app_advertise();
        return 0;

    default:
        return 0;
    }
}

//configuras y arrnacas el advertaising BLE 

static void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;
    struct ble_gap_adv_params adv_params;

    int rc;

    // Advertising principal: solo flags + UUID del servicio
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.uuids128 = (ble_uuid128_t *)&nus_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
  // cargas campos al advertaising del paquete
    rc = ble_gap_adv_set_fields(&fields);

    if (rc != 0) {
        ESP_LOGE(TAG, "Error configurando advertising: %d", rc);
        return;
    }

    // Scan response: nombre del dispositivo
    memset(&rsp_fields, 0, sizeof(rsp_fields));

    // por el espacoio limitado hay que mandar request para obtener elnombrre del dispositivo 
    const char *name = ble_svc_gap_device_name();

    rsp_fields.name = (uint8_t *)name;
    rsp_fields.name_len = strlen(name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);

    if (rc != 0) {
        ESP_LOGE(TAG, "Error configurando scan response: %d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
// modo de conexion y modo de discovery del advertising general y cualqueira se puede coenctar 
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // arrancas el advertaising con todo 
    rc = ble_gap_adv_start(
        own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        ble_gap_event_cb,
        NULL
    );

    if (rc != 0) {
        ESP_LOGE(TAG, "Error iniciando advertising: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising iniciado como %s", DEVICE_NAME);
    }
}

// aqui ya cuadraron la dirreccion y se puede arrancar el advertising y guarda la direccion en own_addr_type para usarla en el advertising y en la conexion
static void ble_app_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);

    if (rc != 0) {
        ESP_LOGE(TAG, "Error obteniendo direccion BLE: %d", rc);
        return;
    }

    ble_app_advertise();
}


// log para diagnosticar por si se renicia el BLE por algun motivo y se pueda volver a arrancar el advertising
static void ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "Reset BLE, reason=%d", reason);
}
// para que no corra en el app main sino en una task dedicada 
static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main() {

    i2c_config_t conf1 = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = 14,
    .scl_io_num = 12,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,  // 100 kHz
    };
    i2c_param_config(I2C_NUM_1, &conf1);
    i2c_driver_install(I2C_NUM_1, conf1.mode, 0, 0, 0);

    spi_bus_config_t bus ={
        .mosi_io_num = MOSI,
        .sclk_io_num = clk,
        .miso_io_num = MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num=CS,
        .queue_size=1
    };
    spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    spi_bus_add_device(SPI2_HOST, &dev ,&spi_dev);

    
    i2c_config_t config = { 
 
        // El ESP32 va a ser el MAESTRO: genera SCL, START, STOP 
        .mode = I2C_MODE_MASTER, 
        .sda_io_num = I2C_MASTER_SDA_IO,   // Pin físico para la línea de datos 
        .scl_io_num = I2C_MASTER_SCL_IO,   // Pin físico para la línea de reloj 
        .sda_pullup_en = GPIO_PULLUP_ENABLE, 
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ, 
    }; 
    i2c_param_config(I2C_PORT, &config); 
    i2c_driver_install(I2C_PORT, config.mode, 0, 0, 0); 

    
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    gpio_set_direction(led_azul, GPIO_MODE_OUTPUT);
    gpio_set_level(led_azul, 0);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(nimble_port_init());

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_svc_gap_device_name_set(DEVICE_NAME);

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svr_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svr_svcs));

    nimble_port_freertos_init(ble_host_task);

    reiniciar();
    vTaskDelay(pdMS_TO_TICKS(600));
    iniciar_rf();
    vTaskDelay(pdMS_TO_TICKS(600));
    lcd_write(0x01,0); // reinicia pantalla y cursor
    escribir("Panel bloqueado", true);
    vTaskDelay(pdMS_TO_TICKS(100));
    escribir("Acerque credencial", false);

    timer_config_t timer_config1 = {
        .divider=80,
        .counter_dir=TIMER_COUNT_UP,
        .alarm_en=TIMER_ALARM_DIS,
        .auto_reload=false 
    };
    timer_init(TIMER_GROUP_0,TIMER_0,&timer_config1);
    timer_set_counter_value(TIMER_GROUP_0,TIMER_0,0);
    timer_start(TIMER_GROUP_0,TIMER_0);

    gpio_config_t out1 ={
   
     .pin_bit_mask = (1ULL << led_r)|(1ULL << led_v) | (1ULL << buzzer)|(1ULL << led_azul),
     .mode = GPIO_MODE_OUTPUT,
     .pull_up_en = GPIO_PULLUP_DISABLE,
     .pull_down_en = GPIO_PULLDOWN_DISABLE,
     .intr_type = GPIO_INTR_DISABLE
    };
    
    
    gpio_config(&out1);

    
    gpio_set_level(led_v,0);
    gpio_set_level(buzzer,0);
    gpio_set_level(led_azul,0);
    bool listo = false;
    bool cam=true;
    bool rojo = true ;  
    bool cerrar= false;
    bool ready = false;
    bool activo = false;
    
    set_time_ds1307(3, 30, 3, 5, 31, 26);

   
    

   


    while (1) {
        timer_get_counter_value(TIMER_GROUP_0,TIMER_0,&now);
        

        if (now - last_rf >= 2100000) {
            last_rf = now;
        
            leer_rfid();

            uint32_t ac = ((uint32_t)uid[0] << 24) |
                        ((uint32_t)uid[1] << 16) |
                        ((uint32_t)uid[2] << 8)  |
                        ((uint32_t)uid[3]);
            ESP_LOGI(TAG, "ac = 0x%08" PRIX32, ac);
        
                
                
            int total = sizeof(uids_validos) / sizeof(uids_validos[0]);

            for (int i = 0; i < total; i++) {
                if (ac == uids_validos[i]) {
                    if (!acceso) {
                        acceso = true;
                        lcd_write(0x01,0);
                        escribir("Acceso concedido", true);
                        escribir(ver_hora(), false);
                        gpio_set_level(led_r,0);
                        last = now;
                        gpio_set_level(buzzer,1);
                        gpio_set_level(led_v,1);
                        cerrar=false;
                        ready = false;
                    }
                    else if (ready && !cerrar) {
                        gpio_set_level(buzzer,1);
                        cerrar=true;
                        gpio_set_level(led_azul,0);
                        gpio_set_level(led_r,1);
                        lcd_write(0x01,0); // reinicia pantalla y cursor
                        escribir("Panel bloqueado", true);
                        
                        escribir("Acerque credencial", false);
                        last=now;
                        acceso=false;
                        activo=false;

                    }
                    break;
                }
                else if ((ac != 0x00000000) && i == total - 1) {
                    acceso = false;
                    lcd_write(0x01,0);
                    escribir("Acceso denegado", true);
                    escribir("UID no registrado", false);
                    last = now;
                    gpio_set_level(led_r,1);
                    gpio_set_level(led_v,0);
                    gpio_set_level(buzzer,1);
                    rojo = false;
                    lastm = now;
                    listo = false;
                    gpio_set_level(led_azul,0); 
                    activo= false;
                }
            } 
        }          
        if (activo && (now-lastsec >= 1000000)){
            lastsec=now;
            escribir(ver_hora(), false);
        }
        if ((acceso || cerrar) && (now -last >=500000)){
            gpio_set_level(buzzer,0);
        }
        if ((acceso && (now -last >=1000000))&& !ready){
            gpio_set_level(led_v,0);
            lcd_write(0x01,0);
            escribir("Sin mensaje", true);
            escribir(ver_hora(), false);
            gpio_set_level(led_azul,1);
            ready = true;
            activo= true;


        }
       

        if (!acceso && (now-lastm>= 300000) && !rojo ){
            cam = !cam;

            gpio_set_level(led_r,cam );
            lastm=now;
        }



        if (!acceso && (now-last>=2000000)&& !listo ){
            lcd_write(0x01,0);
            escribir("Panel bloqueado", true);
            escribir("Acerque credencial", false);
            gpio_set_level(buzzer,0);
            gpio_set_level(led_r,1);
            rojo=true;
            listo=true;
            

        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
        
    }

}