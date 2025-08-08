/*
Fernbedienung für die Steuerung des Inglenook Siding mit DCC-Zentrale "PicoLo"
Läuft auf Cheap Yellow Display ESP32-2432S028R via ESP-Now, Empfänger: ESP32-C3 Supermicro mit UART-Anbindung an RP2040 o. RP2350
Die Kommunikation ist unverschlüsselt

- Autopairing 
- Kommandostructur:
  Präambel (<<<) {Kommando}# Suffix (>>>)
  Jedes Kommando für Lok und Zubehör wird mit # beendet und besteht aus:
  L für Lok
  W für Weiche
  P für PoM - Lok
  A für PoM - Accessory

  Kommandos für das Layout::
  QUIT : Layout und FB ausschalten
  RESET: Layout auf Grundstellung bringen
  EMERG: Layout Nothalt

  z. B. <<<L3,V0,F3#>>>: Lok mit Adresse 3 Vorwärts, Fahrstufe 0, Funktion 3 umschalten
        <<<QUIT>>> Layout und Fernbedienung ausschalten
*/

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>

enum PAIRING_STATUS {
  UNKNOWN,
  NOT_PAIRED,
  PAIR_REQUEST,
  PAIR_REQUESTED,
  PAIRED,
};

PAIRING_STATUS pairing_status = NOT_PAIRED;

enum MSG_TYPE {
  PAIRING,
  DATA,
};

MSG_TYPE msg_type = PAIRING;

typedef struct {
  String name;
  int address;
  byte speedsteps;
  bool invers;
} LOCO_TYPE;

typedef struct {
  String name;
  String index;
  String command;
} ROUTE_TYPE;

typedef struct {
  uint8_t msg_type;
  uint8_t mac[6];
  int length;
  char txt[224];
} MESSAGE_TYPE;

const char* VERSION[] = { "0", "5&", "20250808.1610" };

#define FONT_SMALL &lv_font_montserrat_10
#define FONT_H3 &lv_font_montserrat_20
#define FONT_H1 &lv_font_montserrat_24

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

// Touchscreen pins
const uint8_t XPT2046_IRQ = 36;   // T_IRQ
const uint8_t XPT2046_MOSI = 32;  // T_DIN
const uint8_t XPT2046_MISO = 39;  // T_OUT
const uint8_t XPT2046_CLK = 25;   // T_CLK
const uint8_t XPT2046_CS = 33;    // T_CS
const int SD_CS = 5;

const int REMOTE_BOARD = 1;
const uint32_t SD_TIMEOUT = 5e3;
const uint32_t PAIRING_FREQ = 3e3;

uint8_t peerAddress[6];
char peer_name[64];
uint8_t myAddress[6];
uint8_t broadcastAddress[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
esp_now_peer_info_t peer;

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

LOCO_TYPE locos[64];
ROUTE_TYPE routes[3];

MESSAGE_TYPE outgoing;
MESSAGE_TYPE incoming;
uint32_t pairing_timeout = 0L;

const char* filename = "/meine_lokomotiven.json";  // loco file
int loco_cnt = 0;
int route_cnt = 0;

LOCO_TYPE* active_loco = NULL;

// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;

// If logging is enabled, it will inform the user about what is happening in the library
void log_print(lv_log_level_t level, const char* buf) {
  LV_UNUSED(level);
  // Serial.println(buf);
  // Serial.flush();
}

// Create display and screen objects
lv_display_t* disp;
lv_obj_t* tabview;
lv_obj_t* tab_loco;
lv_obj_t* tab_shunt;
lv_obj_t* tab_fn;
lv_obj_t* tab_pom;
lv_obj_t* tab_info;
lv_obj_t* tab_shunt_label;
lv_obj_t* tab_pom_label;
lv_obj_t* tab_fn_label;
lv_obj_t *img1, *img2, *img3;
lv_obj_t *shunt_fn_f0, *shunt_fn_f3, *dir_reversed, *stop, *dir_forward;
lv_obj_t *label_stop, *label_shunt_fn_f0, *label_shunt_fn_f3, *label_forward, *label_reversed;
lv_obj_t* touch_area_route_1;
lv_obj_t* touch_area_route_2;
lv_obj_t* touch_area_route_3;
lv_obj_t* label_fn;
lv_obj_t* label_tacho;
lv_obj_t* slider;
lv_obj_t* tacho;
lv_obj_t* loco_list;
lv_obj_t* address_spinbox_addr;
lv_obj_t* address_btn_addr_dec;
lv_obj_t* address_btn_addr_inc;
lv_obj_t* address_btn_shift_left;
lv_obj_t* address_btn_shift_right;
lv_obj_t* address_btn_ok;

lv_obj_t* pom_spinbox_addr;
lv_obj_t* pom_btn_addr_dec;
lv_obj_t* pom_btn_addr_inc;
lv_obj_t* pom_spinbox_cv;
lv_obj_t* pom_btn_cv_dec;
lv_obj_t* pom_btn_cv_inc;
lv_obj_t* pom_spinbox_value;
lv_obj_t* pom_btn_value_dec;
lv_obj_t* pom_btn_value_inc;
lv_obj_t* pom_btn_send_multi;
lv_obj_t* pom_btn_send_accessory;
lv_obj_t* mbox_route_error;
lv_obj_t* btn_restart;
lv_obj_t* btn_refresh;
lv_obj_t* btn_power;
lv_obj_t* espnow_label;
lv_obj_t* layout_label = NULL;

lv_obj_t* btnm1;
const char* btnm_map[] = { "L", "1", "2", "3", "4", "\n",
                           "5", "6", "7", "8", "\n",
                           "9", "10", "11", "12", "" };

static lv_obj_t* currentItem = NULL;

lv_style_t style_transp;
lv_style_t style_bg_lightblue;
lv_style_t style_bg_yellow;
lv_style_t style_header_1;
lv_style_t style_header_3;
lv_style_t style_small;

char temp_label[128];
char temp_cmd[128];
int char_cnt;
bool forward = false;
bool reversed = false;
int speed;


void OnDataSent(const esp_now_send_info_t* info, esp_now_send_status_t status) {
}

void OnDataRecv(uint8_t* mac_addr, uint8_t* incomingData, uint8_t length) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.print(length);
  Serial.print(" Bytes received, ");
  Serial.print("Msg Type ");
  Serial.print(incoming.msg_type);
  Serial.print(", Sender MAC: ");
  snprintf(temp_label, sizeof(temp_label), "%X:%X:%X:%X:%X:%X, ",
           *(incoming.mac + 0),
           *(incoming.mac + 1),
           *(incoming.mac + 2),
           *(incoming.mac + 3),
           *(incoming.mac + 4),
           *(incoming.mac + 5));
  memcpy(peerAddress, incoming.mac, 6);
  memset(peer_name, 0, sizeof(peer_name));
  memcpy(peer_name, incoming.txt, min(sizeof(peer_name), strlen(incoming.txt)));
  Serial.print(temp_label);
  Serial.print(" Command length: ");
  Serial.print(incoming.length);
  Serial.print(" Bytes, Command: ");
  Serial.println(incoming.txt);
  if (incoming.msg_type == PAIRING) {
    esp_now_del_peer(broadcastAddress);
    memcpy(peer.peer_addr, peerAddress, 6);
    peer.channel = 0;
    peer.encrypt = false;
    memcpy(broadcastAddress, peerAddress, 6);
    esp_now_add_peer(&peer);
    pairing_status = PAIRED;
  }
  else if (incoming.msg_type == DATA) {  // Get loco state if applicable
  }
}

// Loads the locos from a file
// returns count of locos
void loadLocosAndRoutesFromJson(const char* filename) {
  LOCO_TYPE this_loco;  // temp vars
  ROUTE_TYPE this_route;
  // Open file for reading
  File file = SD.open(filename);
  // Allocate a temporary JsonDocument
  JsonDocument locodoc;
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(locodoc, file);
  // if (error) {
  //   Serial.print(F("Failed to read file "));
  //   Serial.println(filename);
  // }

  JsonArray loco_list = locodoc["locos"];
  for (JsonVariant loco : loco_list) {
    if (loco_cnt < (sizeof(locos) / sizeof(locos[0]) - 1)) {
      // locos[i++] = this_loco;
      locos[loco_cnt++] = {
        loco["name"].as<String>(),
        loco["address"].as<int>(),
        loco["speedsteps"].as<byte>(),
        loco["invers"].as<bool>(),
      };
    }
  }
  locos[loco_cnt++] = {
    "Standard", 3, 128, 0
  };

  JsonArray route_list = locodoc["turnouts"];

  for (JsonVariant route : route_list) {
    if (route_cnt < sizeof(routes) / sizeof(routes[0])) {
      routes[route_cnt++] = {
        route["name"].as<String>(),
        route["short"].as<String>(),
        route["command"].as<String>(),
      };
    }
  }

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}


// Get the Touchscreen data
void touchscreen_read(lv_indev_t* indev, lv_indev_data_t* data) {
  // Checks if Touchscreen was touched, and prints X, Y and Pressure (Z)
  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();
    // Calibrate Touchscreen points with map function to the correct width and height
    x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
    y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);
    z = p.z;

    data->state = LV_INDEV_STATE_PRESSED;

    // Set the coordinates
    data->point.x = x;
    data->point.y = y;

    // Print Touchscreen info about X, Y and Pressure (Z) on the Serial Monitor
    // Serial.print("X = ");
    // Serial.print(x);
    // Serial.print(" | Y = ");
    // Serial.print(y);
    // Serial.print(" | Pressure = ");
    // Serial.print(z);
    // Serial.println();
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void halt() {
    // stop last active loco
  if (active_loco != NULL) {
    char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "<<<L%d,H", active_loco->address);
    for (int i = 0; i < sizeof(btnm_map) / sizeof(btnm_map[0]); i++) {
      if (lv_buttonmatrix_has_button_ctrl(btnm1, i, LV_BUTTONMATRIX_CTRL_CHECKED)) {
        char_cnt = snprintf(temp_cmd + strlen(temp_cmd), sizeof(temp_cmd), ",F%d", i);
      }
    }
    char_cnt = snprintf(temp_cmd + strlen(temp_cmd), sizeof(temp_cmd), "#>>>");
  }
  active_loco = NULL;
}


LOCO_TYPE* activate_loco(LOCO_TYPE* new_loco) {
  lv_arc_set_value(tacho, 0);
  lv_label_set_text_fmt(label_tacho, "%d %%", 0);
  forward = false;
  reversed = false;
  speed = 0;
  lv_obj_remove_state(dir_reversed, LV_STATE_CHECKED);
  lv_obj_remove_state(dir_forward, LV_STATE_CHECKED);
  lv_obj_remove_state(shunt_fn_f0, LV_STATE_CHECKED);
  lv_obj_remove_state(shunt_fn_f3, LV_STATE_CHECKED);
  lv_buttonmatrix_clear_button_ctrl_all(btnm1, LV_BUTTONMATRIX_CTRL_CHECKED);
  if (new_loco != NULL) {
    char_cnt = snprintf(temp_label, sizeof(temp_label), "%s", new_loco->name.c_str());
    lv_label_set_text(tab_shunt_label, temp_label);
    lv_label_set_text(tab_fn_label, temp_label);
  } else {
    lv_label_set_text(tab_shunt_label, "");
    lv_label_set_text(tab_fn_label, "");
  }

  lv_spinbox_set_value(pom_spinbox_addr, (int32_t)new_loco->address);
  return new_loco;
}

static void pom_btn_send_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* obj = lv_event_get_target_obj(e);
  if (code == LV_EVENT_CLICKED) {
    if (obj == pom_btn_send_multi) {
      char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "<<<%c%d,%d,%d#>>>", 'P', lv_spinbox_get_value(pom_spinbox_addr), lv_spinbox_get_value(pom_spinbox_cv), lv_spinbox_get_value(pom_spinbox_value));
    }
    if (obj == pom_btn_send_accessory) {
      char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "<<<%c%d,%d,%d#>>>", 'A', lv_spinbox_get_value(pom_spinbox_addr), lv_spinbox_get_value(pom_spinbox_cv), lv_spinbox_get_value(pom_spinbox_value));
    }
  }
}

static void address_btn_ok_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED) {
    halt();
    locos[0] = {
      "Addresse " + (String)lv_spinbox_get_value(address_spinbox_addr),
      lv_spinbox_get_value(address_spinbox_addr),
      128,
      0
    };
    active_loco = activate_loco(&locos[0]);
    char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "<<<%c%d#>>>", 'G', lv_spinbox_get_value(address_spinbox_addr));
  }
}

static void lv_spinbox_change_value_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* obj = lv_event_get_target_obj(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    // LV_LOG_USER("CLICKED Spinbox decrement");
    // PoM tab
    if (obj == pom_btn_addr_dec) {
      lv_spinbox_decrement(pom_spinbox_addr);
    }
    if (obj == pom_btn_addr_inc) {
      lv_spinbox_increment(pom_spinbox_addr);
    }
    if (obj == pom_btn_cv_dec) {
      lv_spinbox_decrement(pom_spinbox_cv);
    }
    if (obj == pom_btn_cv_inc) {
      lv_spinbox_increment(pom_spinbox_cv);
    }
    if (obj == pom_btn_value_dec) {
      lv_spinbox_decrement(pom_spinbox_value);
    }
    if (obj == pom_btn_value_inc) {
      lv_spinbox_increment(pom_spinbox_value);
    }

    // Address mode tab
    if (obj == address_btn_addr_dec) {
      lv_spinbox_decrement(address_spinbox_addr);
    }
    if (obj == address_btn_addr_inc) {
      lv_spinbox_increment(address_spinbox_addr);
    }
  }
}

static void touch_area_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* obj = lv_event_get_target_obj(e);
  int index = -1;
  if (code == LV_EVENT_CLICKED) {
    *temp_cmd = 0x0;
    if (obj == touch_area_route_1) {
      // LV_LOG_USER("Fahrstrasse 1 einstellen");
      lv_obj_move_foreground(img1);
      index = 0;
    } else if (obj == touch_area_route_2) {
      // LV_LOG_USER("Fahrstrasse 2 einstellen");
      lv_obj_move_foreground(img2);
      index = 1;
    }
    if (obj == touch_area_route_3) {
      // LV_LOG_USER("Fahrstrasse 3 einstellen");
      lv_obj_move_foreground(img3);
      index = 2;
    }
    if (index > -1 && index < 3) {
      char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "<<<%s>>>", (routes[index].command).c_str());
    }
  }
}

static void tacho_released_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_RELEASED) {
    speed = lv_arc_get_value(tacho);
    lv_label_set_text_fmt(label_tacho, "%d %%", speed);
    // LV_LOG_USER("Tacho released @ %i %i %d ", reversed, forward, (int)active_loco->address);
    if (forward ^ reversed) {
      char dir = active_loco->invers ? (reversed ? 'V' : 'R') : (forward ? 'V' : 'R');
      char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "<<<L%d,%c%d#>>>", (int)active_loco->address, dir, speed);
    }
  } else if (code == LV_EVENT_VALUE_CHANGED) {
    lv_label_set_text_fmt(label_tacho, "%d %%", lv_arc_get_value(tacho));
  }
}

static void dir_reversed_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED) {
    reversed = true;
    forward = false;
    if (lv_obj_get_state(dir_forward) & LV_STATE_CHECKED) {
      lv_obj_remove_state(dir_forward, LV_STATE_CHECKED);
    }
    if (!(lv_obj_get_state(dir_reversed) & LV_STATE_CHECKED)) {
      lv_obj_add_state(dir_reversed, LV_STATE_CHECKED);
    }
    lv_obj_send_event(tacho, LV_EVENT_RELEASED, NULL);
  }
}

static void dir_forward_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED) {
    reversed = false;
    forward = true;
    if (lv_obj_get_state(dir_reversed) & LV_STATE_CHECKED) {
      lv_obj_remove_state(dir_reversed, LV_STATE_CHECKED);
    }
    if (!(lv_obj_get_state(dir_forward) & LV_STATE_CHECKED)) {
      lv_obj_add_state(dir_forward, LV_STATE_CHECKED);
    }
    lv_obj_send_event(tacho, LV_EVENT_RELEASED, NULL);
  }
}

void stop_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED) {
    char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "<<<L%d,H#>>>", (int)active_loco->address);
    // LV_LOG_USER("Stop clicked @ %d ", (int)active_loco->address);
    forward = reversed = false;
    lv_obj_remove_state(dir_forward, LV_STATE_CHECKED);
    lv_obj_remove_state(dir_reversed, LV_STATE_CHECKED);
  }
}

void toggle_fn(int fn) {
  char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "<<<L%d,F%d#>>>", (int)active_loco->address, fn);
}

void shunt_fn_f0_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(shunt_fn_f0) & LV_STATE_CHECKED) {
      lv_buttonmatrix_set_button_ctrl(btnm1, 0, LV_BUTTONMATRIX_CTRL_CHECKED);
    } else {
      lv_buttonmatrix_clear_button_ctrl(btnm1, 0, LV_BUTTONMATRIX_CTRL_CHECKED);
    }
    toggle_fn(0);
  }
}

void shunt_fn_f3_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(shunt_fn_f3) & LV_STATE_CHECKED) {
      lv_buttonmatrix_set_button_ctrl(btnm1, 3, LV_BUTTONMATRIX_CTRL_CHECKED);
    } else {
      lv_buttonmatrix_clear_button_ctrl(btnm1, 3, LV_BUTTONMATRIX_CTRL_CHECKED);
    }
    toggle_fn(3);
  }
}

void fn_btnmatrix_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* obj = lv_event_get_target_obj(e);
  if (code == LV_EVENT_VALUE_CHANGED) {
    uint32_t id = lv_buttonmatrix_get_selected_button(obj);
    bool t = lv_obj_get_state(obj) & LV_STATE_CHECKED;
    if (id == 0) {
      lv_obj_set_state(shunt_fn_f0, LV_STATE_CHECKED, !(lv_obj_get_state(shunt_fn_f0) & LV_STATE_CHECKED));
    } else if (id == 3) {
      lv_obj_set_state(shunt_fn_f3, LV_STATE_CHECKED, !(lv_obj_get_state(shunt_fn_f3) & LV_STATE_CHECKED));
    }
    toggle_fn(id);
  }
}

static void btn_restart_event_cb(lv_event_t* e) {
  char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "%s", "<<<EMERG>>>");
}

static void btn_refresh_event_cb(lv_event_t* e) {
  char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "%s", "<<<RESET>>>");
}

static void btn_power_event_cb(lv_event_t* e) {
  char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "%s", "<<<QUIT>>>");
}

static void loco_list_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* obj = lv_event_get_target_obj(e);
  if (code == LV_EVENT_CLICKED) {
    LV_UNUSED(obj);
    // LV_LOG_USER("ausgewählt: %s", lv_list_get_button_text(loco_list, obj));

    currentItem = obj;
    lv_obj_t* parent = lv_obj_get_parent(obj);
    LOCO_TYPE* temp_loco = NULL;
    int32_t i;
    for (i = 0; i < (int32_t)lv_obj_get_child_count(parent); i++) {
      lv_obj_t* child = lv_obj_get_child(parent, i);
      if (child == currentItem) {
        temp_loco = &locos[i];
        lv_obj_add_style(child, &style_bg_lightblue, 0);
      } else {
        lv_obj_remove_style(child, &style_bg_lightblue, 0);
      }
    }
    active_loco = activate_loco(temp_loco);
    char_cnt = snprintf(temp_cmd, sizeof(temp_cmd), "<<<%c%d#>>>", 'G', active_loco->address);
  }
}

void lv_create_tab_loco(lv_obj_t* tab) {
  /*Create a list*/
  loco_list = lv_list_create(tab);
  lv_obj_set_size(loco_list, 300, 190);
  lv_obj_center(loco_list);

  /*Add buttons to the list*/
  lv_obj_t* item;
  for (int i = 0; i < loco_cnt; i++) {
    item = lv_list_add_button(loco_list, NULL, locos[i].name.c_str());
    lv_obj_add_event_cb(item, loco_list_event_cb, LV_EVENT_CLICKED, NULL);
  }
  currentItem = lv_obj_get_child(loco_list, 0);
  lv_obj_add_style(currentItem, &style_bg_lightblue, 0);  // Set the new style to the object
  active_loco = &locos[0];
}

void lv_create_tab_pom(lv_obj_t* tab) {
  lv_obj_t* label = lv_label_create(tab);
  lv_label_set_text(label, "Adresse:");
  lv_obj_align(label, LV_ALIGN_CENTER, -108, -65);

  pom_spinbox_addr = lv_spinbox_create(tab);
  lv_spinbox_set_range(pom_spinbox_addr, 1, 10239);
  lv_spinbox_set_digit_format(pom_spinbox_addr, 5, 0);
  // lv_spinbox_step_prev(pom_spinbox_addr);
  lv_obj_set_width(pom_spinbox_addr, 100);
  lv_obj_align(pom_spinbox_addr, LV_ALIGN_CENTER, 42, -65);

  int32_t h = lv_obj_get_height(pom_spinbox_addr);

  pom_btn_addr_inc = lv_button_create(tab);
  lv_obj_set_size(pom_btn_addr_inc, h, h);
  lv_obj_align_to(pom_btn_addr_inc, pom_spinbox_addr, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(pom_btn_addr_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(pom_btn_addr_inc, lv_spinbox_change_value_event_cb, LV_EVENT_ALL, NULL);

  pom_btn_addr_dec = lv_button_create(tab);
  lv_obj_set_size(pom_btn_addr_dec, h, h);
  lv_obj_align_to(pom_btn_addr_dec, pom_spinbox_addr, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(pom_btn_addr_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(pom_btn_addr_dec, lv_spinbox_change_value_event_cb, LV_EVENT_ALL, NULL);

  label = lv_label_create(tab);
  lv_label_set_text(label, "     CV:");
  lv_obj_align(label, LV_ALIGN_CENTER, -108, -8);

  pom_spinbox_cv = lv_spinbox_create(tab);
  lv_spinbox_set_range(pom_spinbox_cv, 2, 1024);
  lv_spinbox_set_digit_format(pom_spinbox_cv, 4, 0);
  // lv_spinbox_step_prev(pom_spinbox_cv);
  lv_obj_set_width(pom_spinbox_cv, 100);
  lv_obj_align(pom_spinbox_cv, LV_ALIGN_CENTER, 42, -20);

  h = lv_obj_get_height(pom_spinbox_cv);

  pom_btn_cv_inc = lv_button_create(tab);
  lv_obj_set_size(pom_btn_cv_inc, h, h);
  lv_obj_align_to(pom_btn_cv_inc, pom_spinbox_cv, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(pom_btn_cv_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(pom_btn_cv_inc, lv_spinbox_change_value_event_cb, LV_EVENT_ALL, NULL);

  pom_btn_cv_dec = lv_button_create(tab);
  lv_obj_set_size(pom_btn_cv_dec, h, h);
  lv_obj_align_to(pom_btn_cv_dec, pom_spinbox_cv, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(pom_btn_cv_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(pom_btn_cv_dec, lv_spinbox_change_value_event_cb, LV_EVENT_ALL, NULL);

  label = lv_label_create(tab);
  lv_label_set_text(label, "   Wert:");
  lv_obj_align(label, LV_ALIGN_CENTER, -108, 25);

  pom_spinbox_value = lv_spinbox_create(tab);
  lv_spinbox_set_range(pom_spinbox_value, 0, 255);
  lv_spinbox_set_digit_format(pom_spinbox_value, 3, 0);
  // lv_spinbox_step_prev(pom_spinbox_value);
  lv_obj_set_width(pom_spinbox_value, 100);
  lv_obj_align(pom_spinbox_value, LV_ALIGN_CENTER, 42, 25);

  h = lv_obj_get_height(pom_spinbox_value);

  pom_btn_value_inc = lv_button_create(tab);
  lv_obj_set_size(pom_btn_value_inc, h, h);
  lv_obj_align_to(pom_btn_value_inc, pom_spinbox_value, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(pom_btn_value_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(pom_btn_value_inc, lv_spinbox_change_value_event_cb, LV_EVENT_ALL, NULL);

  pom_btn_value_dec = lv_button_create(tab);
  lv_obj_set_size(pom_btn_value_dec, h, h);
  lv_obj_align_to(pom_btn_value_dec, pom_spinbox_value, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(pom_btn_value_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(pom_btn_value_dec, lv_spinbox_change_value_event_cb, LV_EVENT_ALL, NULL);

  pom_btn_send_accessory = lv_button_create(tab);
  lv_obj_add_event_cb(pom_btn_send_accessory, pom_btn_send_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_align(pom_btn_send_accessory, LV_ALIGN_CENTER, -80, 70);
  lv_obj_remove_flag(pom_btn_send_accessory, LV_OBJ_FLAG_PRESS_LOCK);

  label = lv_label_create(pom_btn_send_accessory);
  lv_label_set_text(label, "Zubehoer " LV_SYMBOL_UPLOAD);
  lv_obj_center(label);

  pom_btn_send_multi = lv_button_create(tab);
  lv_obj_add_event_cb(pom_btn_send_multi, pom_btn_send_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_align(pom_btn_send_multi, LV_ALIGN_CENTER, 72, 70);
  lv_obj_remove_flag(pom_btn_send_multi, LV_OBJ_FLAG_PRESS_LOCK);

  label = lv_label_create(pom_btn_send_multi);
  lv_label_set_text(label, "Lokdecoder " LV_SYMBOL_UPLOAD);
  lv_obj_center(label);
}

void lv_create_tab_address(lv_obj_t* tab) {
  lv_obj_t* label = lv_label_create(tab);
  lv_obj_add_style(label, &style_header_3, 0);
  lv_label_set_text(label, "Adress-Modus:");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, -80);

  address_spinbox_addr = lv_spinbox_create(tab);
  lv_spinbox_set_range(address_spinbox_addr, 1, 10239);
  lv_spinbox_set_digit_format(address_spinbox_addr, 5, 0);
  lv_obj_set_width(address_spinbox_addr, 100);
  lv_obj_align(address_spinbox_addr, LV_ALIGN_CENTER, 0, -25);

  int32_t h = lv_obj_get_height(address_spinbox_addr);

  address_btn_addr_inc = lv_button_create(tab);
  lv_obj_set_size(address_btn_addr_inc, h, h);
  lv_obj_align_to(address_btn_addr_inc, address_spinbox_addr, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(address_btn_addr_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(address_btn_addr_inc, lv_spinbox_change_value_event_cb, LV_EVENT_ALL, NULL);

  address_btn_addr_dec = lv_button_create(tab);
  lv_obj_set_size(address_btn_addr_dec, h, h);
  lv_obj_align_to(address_btn_addr_dec, address_spinbox_addr, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(address_btn_addr_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(address_btn_addr_dec, lv_spinbox_change_value_event_cb, LV_EVENT_ALL, NULL);

  address_btn_ok = lv_button_create(tab);
  lv_obj_add_event_cb(address_btn_ok, address_btn_ok_event_cb, LV_EVENT_ALL, NULL);
  lv_obj_align_to(address_btn_ok, address_btn_addr_inc, LV_ALIGN_OUT_BOTTOM_MID, -16, 16);
  lv_obj_remove_flag(address_btn_ok, LV_OBJ_FLAG_PRESS_LOCK);

  label = lv_label_create(address_btn_ok);
  lv_label_set_text(label, LV_SYMBOL_OK);
  lv_obj_center(label);
}

void lv_create_block_fn(lv_obj_t* tab) {
  btnm1 = lv_buttonmatrix_create(tab);
  lv_buttonmatrix_set_map(btnm1, btnm_map);
  lv_buttonmatrix_set_button_ctrl_all(btnm1, LV_BUTTONMATRIX_CTRL_CHECKABLE);
  lv_obj_align(btnm1, LV_ALIGN_CENTER, 0, 24);
  lv_obj_add_event_cb(btnm1, fn_btnmatrix_event_cb, LV_EVENT_ALL, NULL);
}

void lv_create_tab_fn(lv_obj_t* tab) {
  lv_obj_t* tab_txt_label = lv_label_create(tab);
  lv_label_set_text(tab_txt_label, "Funktionen");
  lv_obj_align(tab_txt_label, LV_ALIGN_TOP_MID, 0, 6);
  tab_fn_label = lv_label_create(tab);
  lv_obj_align(tab_fn_label, LV_ALIGN_TOP_MID, 0, 24);
  if (active_loco != NULL) {
    lv_label_set_text(tab_fn_label, active_loco->name.c_str());
  }
  lv_create_block_fn(tab);
}

void lv_create_tacho(lv_obj_t* tab) {
  tacho = lv_arc_create(tab);
  lv_obj_set_size(tacho, 90, 90);
  lv_arc_set_rotation(tacho, 135);
  lv_arc_set_bg_angles(tacho, 0, 270);
  lv_arc_set_value(tacho, 0);
  lv_obj_align(tacho, LV_ALIGN_CENTER, 80, 0);

  label_tacho = lv_label_create(tab);
  lv_obj_align_to(label_tacho, tacho, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(tacho, tacho_released_event_cb, LV_EVENT_ALL, label_tacho);
  lv_obj_send_event(tacho, LV_EVENT_RELEASED, NULL);
}

void lv_create_tab_shunt(lv_obj_t* tab) {
  // Create a slider aligned in the center bottom of the TFT display
  tab_shunt_label = lv_label_create(tab);
  lv_obj_add_style(tab_shunt_label, &style_header_3, 0);
  lv_obj_align(tab_shunt_label, LV_ALIGN_TOP_MID, 0, 0);
  if (active_loco != NULL) {
    lv_label_set_text(tab_shunt_label, active_loco->name.c_str());
  }

  if (route_cnt == 3) {
    LV_IMAGE_DECLARE(Fahrstrasse_3);
    img3 = lv_image_create(tab);
    lv_image_set_src(img3, &Fahrstrasse_3);
    lv_obj_align_to(img3, tab_shunt_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

    LV_IMAGE_DECLARE(Fahrstrasse_2);
    img2 = lv_image_create(tab);
    lv_image_set_src(img2, &Fahrstrasse_2);
    lv_obj_align_to(img2, tab_shunt_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

    LV_IMAGE_DECLARE(Fahrstrasse_1);
    img1 = lv_image_create(tab);
    lv_image_set_src(img1, &Fahrstrasse_1);
    lv_obj_align_to(img1, tab_shunt_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

    int height = 32, width = 160;
    touch_area_route_1 = lv_obj_create(img1);
    lv_obj_set_pos(touch_area_route_1, 0, 0);
    lv_obj_set_size(touch_area_route_1, width, height);
    lv_obj_add_flag(touch_area_route_1, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(touch_area_route_1, touch_area_event_cb, LV_EVENT_CLICKED, NULL);

    touch_area_route_2 = lv_obj_create(img1);
    lv_obj_set_pos(touch_area_route_2, 0, 33);
    lv_obj_set_size(touch_area_route_2, width, height);
    lv_obj_add_flag(touch_area_route_2, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(touch_area_route_2, touch_area_event_cb, LV_EVENT_CLICKED, NULL);

    touch_area_route_3 = lv_obj_create(img1);
    lv_obj_set_pos(touch_area_route_3, 0, 65);
    lv_obj_set_size(touch_area_route_3, width, height);
    lv_obj_add_flag(touch_area_route_3, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(touch_area_route_3, touch_area_event_cb, LV_EVENT_CLICKED, NULL);

    lv_style_init(&style_transp);
    lv_style_set_bg_opa(&style_transp, LV_OPA_TRANSP);       // Set background opacity to transparent
    lv_style_set_border_width(&style_transp, 0);             // No border
    lv_obj_add_style(touch_area_route_1, &style_transp, 0);  // Set the new style to the object
    lv_obj_add_style(touch_area_route_2, &style_transp, 0);  // Set the new style to the object
    lv_obj_add_style(touch_area_route_3, &style_transp, 0);  // Set the new style to the object
  }
  lv_create_tacho(tab);

  int h = 32;

  shunt_fn_f0 = lv_button_create(tab);
  lv_obj_set_size(shunt_fn_f0, h, h);
  lv_obj_align(shunt_fn_f0, LV_ALIGN_BOTTOM_LEFT, 0, -4);
  lv_obj_add_flag(shunt_fn_f0, LV_OBJ_FLAG_CHECKABLE);
  label_shunt_fn_f0 = lv_label_create(shunt_fn_f0);
  lv_label_set_text(label_shunt_fn_f0, "L");
  lv_obj_center(label_shunt_fn_f0);
  lv_obj_add_event_cb(shunt_fn_f0, shunt_fn_f0_event_cb, LV_EVENT_CLICKED, NULL);

  shunt_fn_f3 = lv_button_create(tab);
  lv_obj_set_size(shunt_fn_f3, h, h);
  lv_obj_align_to(shunt_fn_f3, shunt_fn_f0, LV_ALIGN_CENTER, 48, 0);
  lv_obj_add_flag(shunt_fn_f3, LV_OBJ_FLAG_CHECKABLE);
  label_shunt_fn_f3 = lv_label_create(shunt_fn_f3);
  lv_label_set_text(label_shunt_fn_f3, "F3");
  lv_obj_center(label_shunt_fn_f3);
  lv_obj_add_event_cb(shunt_fn_f3, shunt_fn_f3_event_cb, LV_EVENT_CLICKED, NULL);

  dir_reversed = lv_button_create(tab);
  lv_obj_set_size(dir_reversed, h, h);
  lv_obj_align_to(dir_reversed, tacho, LV_ALIGN_OUT_BOTTOM_MID, -48, 10);
  lv_obj_add_flag(dir_reversed, LV_OBJ_FLAG_CHECKABLE);
  label_reversed = lv_label_create(dir_reversed);
  lv_label_set_text(label_reversed, LV_SYMBOL_LEFT);
  lv_obj_center(label_reversed);
  lv_obj_add_event_cb(dir_reversed, dir_reversed_event_cb, LV_EVENT_CLICKED, NULL);

  stop = lv_button_create(tab);
  lv_obj_set_size(stop, h, h);
  lv_obj_align_to(stop, tacho, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
  label_stop = lv_label_create(stop);
  lv_label_set_text(label_stop, LV_SYMBOL_STOP);
  lv_obj_center(label_stop);
  lv_obj_add_event_cb(stop, stop_event_cb, LV_EVENT_CLICKED, NULL);

  dir_forward = lv_button_create(tab);
  lv_obj_set_size(dir_forward, h, h);
  lv_obj_align_to(dir_forward, tacho, LV_ALIGN_OUT_BOTTOM_MID, 48, 10);
  lv_obj_add_flag(dir_forward, LV_OBJ_FLAG_CHECKABLE);
  label_forward = lv_label_create(dir_forward);
  lv_label_set_text(label_forward, LV_SYMBOL_RIGHT);
  lv_obj_center(label_forward);
  lv_obj_add_event_cb(dir_forward, dir_forward_event_cb, LV_EVENT_CLICKED, NULL);
}

static PAIRING_STATUS last_pairing_status = UNKNOWN;
void set_label_espnow(PAIRING_STATUS pairing_status) {
  if (last_pairing_status != pairing_status) {
    last_pairing_status = pairing_status;
    switch (pairing_status) {
      case PAIRED:
        lv_label_set_text(espnow_label, LV_SYMBOL_WIFI " Verbunden mit");
        lv_label_set_text(layout_label, peer_name);
        break;
      case PAIR_REQUEST:
      case PAIR_REQUESTED:
        lv_label_set_text(espnow_label, LV_SYMBOL_WARNING " Suche Layout...");
        break;
      default:
        lv_label_set_text(espnow_label, LV_SYMBOL_WARNING " Nicht verbunden");
        break;
    }
  }
}

void lv_create_tab_info(lv_obj_t* tab) {
  lv_obj_t* label = lv_label_create(tab);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 3);
  lv_obj_add_style(label, &style_header_3, 0);  // Set the new style to the object
  lv_label_set_text(label, "Inglenook Siding Remote");

  char_cnt = snprintf(temp_label, sizeof(temp_label), "Version: %s.%s Build %s", VERSION[0], VERSION[1], VERSION[2]);
  label = lv_label_create(tab);
  lv_obj_add_style(label, &style_small, 0);  // Set the new style to the object
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 36);
  lv_label_set_text(label, temp_label);
  char_cnt = snprintf(temp_label, sizeof(temp_label), "LVGL Library Version: %d.%d.%d", lv_version_major(), lv_version_minor(), lv_version_patch());
  label = lv_label_create(tab);
  lv_obj_add_style(label, &style_small, 0);  // Set the new style to the object
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 52);
  lv_label_set_text(label, temp_label);
  espnow_label = lv_label_create(tab);
  lv_obj_align(espnow_label, LV_ALIGN_TOP_MID, 0, 72);
  layout_label = lv_label_create(tab);
  lv_obj_align(layout_label, LV_ALIGN_TOP_MID, 0, 88);
  lv_label_set_text(layout_label, "");
  set_label_espnow(pairing_status);

  btn_power = lv_button_create(tab);
  lv_obj_align(btn_power, LV_ALIGN_CENTER, -40, 70);
  label = lv_label_create(btn_power);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(label, LV_SYMBOL_POWER);
  lv_obj_add_event_cb(btn_power, btn_power_event_cb, LV_EVENT_CLICKED, NULL);

  btn_refresh = lv_button_create(tab);
  lv_obj_align(btn_refresh, LV_ALIGN_CENTER, 40, 70);
  label = lv_label_create(btn_refresh);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(label, LV_SYMBOL_REFRESH);
  lv_obj_add_event_cb(btn_refresh, btn_refresh_event_cb, LV_EVENT_CLICKED, NULL);
}

void create_gui(lv_obj_t* screen) {
  tabview = lv_tabview_create(screen);
  lv_obj_remove_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);
  lv_tabview_set_tab_bar_position(tabview, LV_DIR_TOP);
  lv_tabview_set_tab_bar_size(tabview, 32);

  lv_obj_set_style_bg_color(tabview, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
  lv_obj_set_style_text_color(tabview, lv_palette_lighten(LV_PALETTE_GREY, 5), 0);
  tab_loco = lv_tabview_add_tab(tabview, LV_SYMBOL_LIST);
  // lv_obj_remove_flag(lv_tabview_get_content(tab_loco), LV_OBJ_FLAG_SCROLLABLE);
  tab_shunt = lv_tabview_add_tab(tabview, LV_SYMBOL_PLAY);
  // lv_obj_remove_flag(lv_tabview_get_content(tab_shunt), LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(tab_shunt, LV_SCROLLBAR_MODE_OFF);
  tab_fn = lv_tabview_add_tab(tabview, "Fx");
  tab_pom = lv_tabview_add_tab(tabview, "PoM");
  // lv_obj_remove_flag(lv_tabview_get_content(tab_pom), LV_OBJ_FLAG_SCROLLABLE);
  tab_info = lv_tabview_add_tab(tabview, LV_SYMBOL_SETTINGS);

  lv_style_init(&style_bg_lightblue);
  lv_color_t lightblue = { 0xff, 0xb0, 0xa0 };
  lv_style_set_bg_color(&style_bg_lightblue, lightblue);  // Set background color to yellow

  lv_style_init(&style_bg_yellow);
  lv_color_t yellow = { 0xb0, 0xff, 0xff };
  lv_style_set_bg_color(&style_bg_yellow, yellow);  // Set background color to yellow

  // <h>...</h3>
  lv_style_init(&style_small);
  lv_style_set_text_font(&style_small, FONT_SMALL);

  // <h>...</h3>
  lv_style_init(&style_header_3);
  lv_style_set_text_font(&style_header_3, FONT_H3);

  // <h1>...</h1>
  lv_style_init(&style_header_1);
  lv_style_set_text_font(&style_header_1, FONT_H1);

  // Initialize an LVGL input device object (Touchscreen)
  lv_indev_t* indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  // Set the callback function to read Touchscreen input
  lv_indev_set_read_cb(indev, touchscreen_read);

  // Function to draw the GUI (text, buttons and sliders)
  if (loco_cnt > 0) {
    lv_create_tab_loco(tab_loco);
  } else {
    lv_create_tab_address(tab_loco);
  }
  lv_create_tab_shunt(tab_shunt);
  lv_obj_add_state(tab_shunt, LV_STATE_DISABLED);
  lv_create_tab_fn(tab_fn);
  lv_obj_add_state(tab_fn, LV_STATE_DISABLED);
  lv_create_tab_pom(tab_pom);
  lv_create_tab_info(tab_info);
}

void send_outgoing(char* text) {
  Serial.println(text);
  memset((void*)&outgoing, 0x0, sizeof(outgoing));
  outgoing.msg_type = msg_type;
  memcpy(outgoing.mac, myAddress, 6);
  outgoing.length = strlen(text);
  memcpy(outgoing.txt, text, strlen(text));
  uint8_t* mac;
  if (pairing_status == PAIRED) {
    mac = peerAddress;
  } else {
    mac = broadcastAddress;
  }
  esp_err_t result = esp_now_send(mac, (uint8_t*)&outgoing, sizeof(outgoing));
  int repetitions = 5;
  while (repetitions-- > 0 && result != ESP_OK) {
    result = esp_now_send(mac, (uint8_t*)&outgoing, sizeof(outgoing));
  }
}

PAIRING_STATUS autoPair() {
  switch (pairing_status) {
    case NOT_PAIRED:
      memcpy(peer.peer_addr, broadcastAddress, 6);
      peer.channel = 0;
      peer.encrypt = false;

      if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Fehler add_peer");
      }
      snprintf(temp_cmd, sizeof(temp_cmd), "Rangierer_%d", REMOTE_BOARD);
      send_outgoing(temp_cmd);
      pairing_status = PAIR_REQUEST;
      break;

    case PAIR_REQUEST:
      pairing_status = PAIR_REQUESTED;
      pairing_timeout = millis() + PAIRING_FREQ;
      break;

    case PAIR_REQUESTED:
      if (pairing_timeout <= millis()) {
        pairing_status = NOT_PAIRED;
      }
      break;

    case PAIRED:
      msg_type = DATA;
      break;

    default:
      pairing_status = NOT_PAIRED;
  }
  set_label_espnow(pairing_status);
  return pairing_status;
}

void get_mac(uint8_t* mac) {
  String MAC = WiFi.macAddress();
  MAC = "0x" + MAC;
  MAC.replace(":", ":0x");
  uint8_t n = sscanf(MAC.c_str(), "%x:%x:%x:%x:%x:%x", mac, mac + 1, mac + 2, mac + 3, mac + 4, mac + 5);
  if (n != 6) {
    Serial.println("MAC-Adresse nicht korrekt ermittelt");
    ESP.restart();
  }
  // Serial.println(n);
  // Serial.printf("%2.2x : %2.2x : %2.2x : %2.2x : %2.2x : %2.2x  \n", *(mac), *(mac + 1), *(mac + 2), *(mac + 3), *(mac + 4), *(mac + 5));
}

void setup() {
  Serial.begin(115200);

  int32_t timeout = millis() + SD_TIMEOUT;
  while (!SD.begin(SD_CS) && millis() < timeout) {
    delay(1100);
  }
  if (millis() <= timeout) {
    // Read loco and route definitions from SD card
    loadLocosAndRoutesFromJson(filename);
  }

  // Init ESP-Now
  WiFi.begin();
  WiFi.mode(WIFI_STA);
  get_mac(myAddress);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  }

  // Start LVGL
  lv_init();
  // Register print function for debugging
  // lv_log_register_print_cb(log_print);

  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  // Set the Touchscreen rotation in landscape mode
  // Note: in some displays, the touchscreen might be upside down, so you might need to set the rotation to 0: touchscreen.setRotation(0);
  touchscreen.setRotation(2);

  // Initialize the TFT display using the TFT_eSPI library
  disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);

  *temp_cmd = 0x0;
  create_gui(lv_screen_active());

  lv_tabview_set_active(tabview, lv_tabview_get_tab_count(tabview) - 1, LV_ANIM_OFF);
}

void loop() {
  if (autoPair() == PAIRED) {
    if (strlen(temp_cmd)) {
      send_outgoing(temp_cmd);
      *temp_cmd = 0x0;
    }
  }
  if (active_loco == NULL) {
    if (lv_tabview_get_tab_active(tabview) == 1 or lv_tabview_get_tab_active(tabview) == 2) {
      lv_tabview_set_active(tabview, 0, LV_ANIM_OFF);
    }
    lv_obj_add_state(tab_shunt, LV_STATE_DISABLED);
    lv_obj_add_state(tab_fn, LV_STATE_DISABLED);
  } else {
    if (lv_obj_has_state(tab_shunt, LV_STATE_DISABLED)) {
      lv_obj_remove_state(tab_shunt, LV_STATE_DISABLED);
    }
    if (lv_obj_has_state(tab_fn, LV_STATE_DISABLED)) {
      lv_obj_remove_state(tab_fn, LV_STATE_DISABLED);
    }
  }
  lv_timer_handler();  // let the GUI do its work
  lv_tick_inc(5);      // tell LVGL how much time has passed
  delay(5);            // let this time pass
}
