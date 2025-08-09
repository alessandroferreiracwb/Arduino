#include <WiFi.h>
#include "lvgl.h"
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

/* Configurações de Hardware e LVGL */
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];
TFT_eSPI tft = TFT_eSPI();

// Seus valores de calibração obtidos
const int touchMinX = 430;
const int touchMaxX = 3700;
const int touchMinY = 600;
const int touchMaxY = 3570;

/* Protótipos de funções */
void lv_tick_isr();
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, unsigned char *color_p);
void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data);

void create_wifi_setup_screen();
void connect_button_handler(lv_event_t *e);
void rescan_button_handler(lv_event_t *e);
void dropdown_event_handler(lv_event_t *e);
void keyboard_event_handler(lv_event_t *e);
void scan_and_populate_dropdown();

lv_obj_t *wifi_dropdown;
lv_obj_t *password_field;
lv_obj_t *status_label;
lv_obj_t *selected_ssid_label; // Novo label para o SSID selecionado
lv_obj_t *kb;
lv_obj_t *screen;

void setup() {
    Serial.begin(115200);

    /* Inicializa o timer para o LVGL */
    hw_timer_t *timer = NULL;
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &lv_tick_isr, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);

    /* Inicializa o SPI para o touchscreen */
    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    touchscreen.begin(touchscreenSPI);
    touchscreen.setRotation(2);

    /* Inicializa o display TFT_eSPI */
    tft.init();
    tft.setRotation(1);
    
    // A chamada setTouch() foi removida, o mapeamento será feito na my_touchpad_read().

    /* Inicializa LVGL */
    lv_init();

    /* Inicializa o display do LVGL */
    static lv_display_t *disp;
    disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270);
    
    /* Inicializa o input do LVGL (touchscreen) */
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touchpad_read);

    /* Cria a tela de configuração de Wi-Fi */
    create_wifi_setup_screen();
}

void loop() {
    lv_timer_handler();
    delay(5);
}

/* Funções de callback para o LVGL */
void IRAM_ATTR lv_tick_isr() {
    lv_tick_inc(1);
}

void my_disp_flush(lv_display_t *disp, const lv_area_t *area, unsigned char *color_p) {
    tft.pushImage(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (uint16_t *)color_p);
    lv_display_flush_ready(disp);
}

void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
    if (touchscreen.touched()) {
        TS_Point p = touchscreen.getPoint();
        data->state = LV_INDEV_STATE_PRESSED;
        
        // Mapeia os valores de calibração do seu touch para as dimensões da tela
        data->point.x = map(p.x, touchMinX, touchMaxX, 1, SCREEN_WIDTH);
        data->point.y = map(p.y, touchMinY, touchMaxY, 1, SCREEN_HEIGHT);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/* Funções da interface de Configuração Wi-Fi */
void create_wifi_setup_screen() {
    screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen, lv_color_make(0xFF, 0xFF, 0xFF), LV_PART_MAIN);
    
    lv_obj_t *ssid_label = lv_label_create(screen);
    lv_label_set_text(ssid_label, "WiFi SSID:");
    lv_obj_set_style_text_color(ssid_label, lv_color_make(0x00, 0x00, 0x00), LV_PART_MAIN);
    lv_obj_set_style_text_font(ssid_label, &lv_font_montserrat_14, 0);
    lv_obj_align(ssid_label, LV_ALIGN_TOP_LEFT, 10, 10);

    // Novo label para o SSID selecionado
    selected_ssid_label = lv_label_create(screen);
    lv_label_set_text(selected_ssid_label, "Nenhum");
    lv_obj_set_style_text_color(selected_ssid_label, lv_color_make(0x00, 0x00, 0x00), LV_PART_MAIN);
    lv_obj_set_style_text_font(selected_ssid_label, &lv_font_montserrat_14, 0);
    lv_obj_align_to(selected_ssid_label, ssid_label, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

    wifi_dropdown = lv_dropdown_create(screen);
    lv_obj_set_width(wifi_dropdown, lv_obj_get_width(screen) - 20);
    lv_dropdown_set_text(wifi_dropdown, "Procurando redes...");
    lv_obj_align_to(wifi_dropdown, ssid_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);
    lv_obj_set_style_bg_color(wifi_dropdown, lv_color_make(0xE0, 0xE0, 0xE0), LV_PART_MAIN);
    lv_obj_set_style_text_color(wifi_dropdown, lv_color_make(0x00, 0x00, 0x00), LV_PART_MAIN);
    lv_obj_add_event_cb(wifi_dropdown, dropdown_event_handler, LV_EVENT_ALL, NULL);

    lv_obj_t *password_label = lv_label_create(screen);
    lv_label_set_text(password_label, "WiFi Password");
    lv_obj_set_style_text_color(password_label, lv_color_make(0x00, 0x00, 0x00), LV_PART_MAIN);
    lv_obj_set_style_text_font(password_label, &lv_font_montserrat_14, 0);
    lv_obj_align_to(password_label, wifi_dropdown, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    
    password_field = lv_textarea_create(screen);
    lv_obj_set_width(password_field, lv_obj_get_width(screen) - 20);
    lv_obj_set_height(password_field, 35);
    lv_textarea_set_password_mode(password_field, true);
    lv_textarea_set_placeholder_text(password_field, "Digite a senha");
    lv_obj_align_to(password_field, password_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);
    lv_obj_set_style_bg_color(password_field, lv_color_make(0xE0, 0xE0, 0xE0), LV_PART_MAIN);
    lv_obj_set_style_text_color(password_field, lv_color_make(0x00, 0x00, 0x00), LV_PART_MAIN);
    lv_obj_add_event_cb(password_field, keyboard_event_handler, LV_EVENT_ALL, NULL);

    lv_obj_t *connect_button = lv_button_create(screen);
    lv_obj_set_width(connect_button, 100);
    lv_obj_set_style_bg_color(connect_button, lv_color_make(0xDD, 0xDD, 0xDD), LV_PART_MAIN);
    lv_obj_set_style_border_color(connect_button, lv_color_make(0x80, 0x80, 0x80), LV_PART_MAIN);
    lv_obj_align(connect_button, LV_ALIGN_BOTTOM_LEFT, 10, -50);
    lv_obj_add_event_cb(connect_button, connect_button_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label_connect = lv_label_create(connect_button);
    lv_label_set_text(btn_label_connect, "Connect");
    lv_obj_center(btn_label_connect);
    lv_obj_set_style_text_color(btn_label_connect, lv_color_make(0x00, 0x00, 0x00), LV_PART_MAIN);

    lv_obj_t *rescan_button = lv_button_create(screen);
    lv_obj_set_width(rescan_button, 100);
    lv_obj_set_style_bg_color(rescan_button, lv_color_make(0xDD, 0xDD, 0xDD), LV_PART_MAIN);
    lv_obj_set_style_border_color(rescan_button, lv_color_make(0x80, 0x80, 0x80), LV_PART_MAIN);
    lv_obj_align(rescan_button, LV_ALIGN_BOTTOM_RIGHT, -10, -50);
    lv_obj_add_event_cb(rescan_button, rescan_button_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label_rescan = lv_label_create(rescan_button);
    lv_label_set_text(btn_label_rescan, "Re-Scan");
    lv_obj_center(btn_label_rescan);
    lv_obj_set_style_text_color(btn_label_rescan, lv_color_make(0x00, 0x00, 0x00), LV_PART_MAIN);

    status_label = lv_label_create(screen);
    lv_obj_align_to(status_label, connect_button, LV_ALIGN_OUT_TOP_MID, 0, -10);
    lv_label_set_text(status_label, "");
    lv_obj_set_style_text_color(status_label, lv_color_make(0x00, 0x00, 0x00), LV_PART_MAIN);
    
    kb = lv_keyboard_create(screen);
    lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    
    lv_scr_load(screen);
    scan_and_populate_dropdown();
}

void dropdown_event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = (lv_obj_t*) lv_event_get_target(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        char buf[64];
        lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
        lv_textarea_set_text(password_field, "");
        lv_label_set_text(selected_ssid_label, buf); // <--- CORREÇÃO AQUI
        Serial.print("Rede selecionada na tela: ");
        Serial.println(buf);
    }
}

void keyboard_event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);

    if (code == LV_EVENT_FOCUSED) {
        lv_keyboard_set_textarea(kb, ta);
        lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
    if (code == LV_EVENT_DEFOCUSED) {
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
}

void connect_button_handler(lv_event_t *e) {
    char ssid_buf[64];
    lv_dropdown_get_selected_str(wifi_dropdown, ssid_buf, sizeof(ssid_buf));
    const char *ssid = ssid_buf;
    const char *password = lv_textarea_get_text(password_field);

    Serial.println("------------------------------------");
    Serial.println("Tentando conectar com as credenciais:");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Senha: ");
    Serial.println(password);
    Serial.println("------------------------------------");

    if (strlen(ssid) > 0) {
        lv_label_set_text(status_label, "Conectando...");
        
        WiFi.begin(ssid, password);
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            attempts++;
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            lv_label_set_text(status_label, "Conectado!");
            lv_obj_set_style_text_color(status_label, lv_color_make(0x00, 0x80, 0x00), LV_PART_MAIN);
            Serial.println("\nConectado!");
            Serial.println(WiFi.localIP());
        } else {
            lv_label_set_text(status_label, "Falha na conexao!");
            lv_obj_set_style_text_color(status_label, lv_color_make(0xFF, 0x00, 0x00), LV_PART_MAIN);
            Serial.println("\nFalha na conexao.");
        }
    } else {
        lv_label_set_text(status_label, "Selecione uma rede!");
        lv_obj_set_style_text_color(status_label, lv_color_make(0xFF, 0x00, 0x00), LV_PART_MAIN);
    }
}

void rescan_button_handler(lv_event_t *e) {
    lv_label_set_text(status_label, "Escaneando redes...");
    lv_dropdown_set_text(wifi_dropdown, "Procurando redes...");
    scan_and_populate_dropdown();
}

void scan_and_populate_dropdown() {
    lv_obj_add_state(wifi_dropdown, LV_STATE_DISABLED);
    lv_dropdown_set_options(wifi_dropdown, "");
    lv_label_set_text(status_label, "Escaneando...");
    
    int n = WiFi.scanNetworks();
    Serial.println("Scan completo.");

    if (n == 0) {
        lv_label_set_text(status_label, "Nenhuma rede encontrada.");
    } else {
        String ssid_list;
        for (int i = 0; i < n; ++i) {
            ssid_list += WiFi.SSID(i);
            if (i < n - 1) {
                ssid_list += "\n";
            }
            delay(10);
        }
        lv_dropdown_set_options(wifi_dropdown, ssid_list.c_str());
        lv_dropdown_set_text(wifi_dropdown, "Selecione uma rede");
        lv_label_set_text(status_label, "");
    }
    lv_obj_clear_state(wifi_dropdown, LV_STATE_DISABLED);
    WiFi.scanDelete();
}