/*
 * main.c — Z906 Console Zigbee (ESP32-H2) — v3 (esp-zigbee-lib 2.x)
 *
 * Console alternativa per Logitech Z906, senza console fisica originale.
 * L'ESP32-H2 simula il comportamento della console verso l'amplificatore.
 *
 * PIN (PCB1_1):
 *   GPIO0  — WS2812B data (LED1–LED13 in catena)
 *   GPIO1  — Q1 base → AMP pin15 (LOW = amp alimentato)
 *   GPIO9  — Factory reset (pulsante su H1, pullup interna)
 *   GPIO10 — VS1838B OUT (ricevitore IR)
 *   GPIO11 — AMP pin8 (segnale stato dall'amp)
 *   GPIO5  — UART0 TX → AMP (amp pin13)
 *   GPIO4  — UART0 RX ← AMP (amp pin12)
 *   GPIO26 — USB D-  (console log via USB Serial/JTAG)
 *   GPIO27 — USB D+
 *
 * UART: UART_NUM_1 dedicato all'amp (GPIO4/5 via GPIO matrix); log su USB.
 *       NON usare UART0: le stampe ROM/ISR (esp_rom_printf, ESP_DRAM_LOGx)
 *       escono su UART0 anche con console USB-JTAG (datasheet H2-MINI-1 §4.2).
 *
 * ZIGBEE (ZHA):
 *   EP1 — On/Off Output  : accensione/spegnimento
 *   EP2 — Analog Output  : volume (0–43)
 *   EP3 — Analog Output  : input (1–6)
 *
 * LED WS2812B (13 totali):
 *   LED[0]   — stato console (ZB/amp)
 *   LED[1]   — input selezionato (colore per input 1–6)
 *   LED[2-12]— barra volume (43 livelli, 4 luminosità per LED)
 *
 * IR NEC1 (device=2, subdevice=160, addr=0xA002, 38.65kHz):
 *   0xAA — Volume+  |  0xA9 — Volume-
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_cluster.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "led_strip.h"
#include "ota.h"
#include <inttypes.h>

#define TAG "Z906"

/* ── UART amp ──────────────────────────────────────────────────────── */
#define UART_AMP    UART_NUM_1
#define GPIO_AMP_TX GPIO_NUM_5
#define GPIO_AMP_RX GPIO_NUM_4
#define Z906_BAUD   57600
#define BUF         256

/* ── GPIO ──────────────────────────────────────────────────────────── */
#define GPIO_AMP15_OUT GPIO_NUM_1    /* Q1: HIGH = pin15 LOW = amp acceso */
#define GPIO_AMP8_IN   GPIO_NUM_11   /* pin8 dall'amp */
#define GPIO_POLL_MS   2

/* ── IR receiver (VS1838B) ─────────────────────────────────────────── */
#define GPIO_IR           GPIO_NUM_10
#define IR_RMT_RES_HZ     1000000     /* 1 MHz → 1 µs/tick */
#define IR_BUF_SYMBOLS    512         /* buffer utente: frame lunghi di altri telecomandi (es. TCL) entrano interi e vengono scartati */
#define IR_HW_MEM_SYMBOLS 64          /* memoria hardware RMT (ping-pong): non aumentare, condivisa con il canale LED */
#define NEC_ADDR_Z906     0xA002u     /* device=2, subdevice=0xA0 */
#define NEC_CMD_VOL_UP    0xAAu
#define NEC_CMD_VOL_DN    0x6Au  /* OBC=106: byte reale sul filo NEC (HEX=A9 è in altro formato) */
#define NEC_CMD_MUTE      0xEAu  /* OBC=234, verificato nel log: IR: addr=0xA002 cmd=0xEA */
#define IR_REPEAT_MS      180         /* intervallo minimo tra step da hold */
#define IR_REPEAT_WINDOW_MS 200       /* repeat valido solo entro questo tempo dal frame/repeat Z906 precedente (periodo NEC 108 ms) */

/* NEC timing (µs) */
#define NEC_HDR_HI   9000u
#define NEC_HDR_LO   4500u
#define NEC_RPT_LO   2250u
#define NEC_BIT_HI    560u
#define NEC_BIT0_LO   560u
#define NEC_BIT1_LO  1690u
#define NEC_MARGIN    300u

/* ── WS2812B LED strip ─────────────────────────────────────────────── */
#define GPIO_LED       GPIO_NUM_0
#define LED_NUM        13
#define LED_STATUS     0   /* LED1  — stato console */
#define LED_INPUT      1   /* LED2  — input selezionato */
#define LED_VOL_BASE   2   /* LED3-LED13 — barra volume (11 LED) */
#define LED_VOL_COUNT  11
#define LED_TASK_MS    50  /* periodo refresh LED */

/* Luminosità barra volume (4 livelli, su 255) */
static const uint8_t s_vol_bri[4] = {20, 40, 60, 80};

/* Colori input [inp 0..6][R,G,B] — indice 0 = amp spento */
static const uint8_t s_inp_col[7][3] = {
    {  0,  0,  0},  /* 0 — spento */
    { 50, 50, 50},  /* 1 — bianco */
    {  0, 50, 50},  /* 2 — ciano */
    {  0, 70,  0},  /* 3 — verde  (TOSLINK, più usato) */
    {  0,  0, 70},  /* 4 — blu */
    { 50,  0, 50},  /* 5 — magenta */
    { 50, 40,  0},  /* 6 — giallo */
};

/* ── Stato amplificatore ───────────────────────────────────────────── */
static uint8_t s_amp_status[24] = {
    0xAA, 0x0A, 0x14,
    0x0E, 0x13, 0x2B, 0x0F,
    0x02,                        /* input 3 / TOSLINK */
    0x00,                        /* mute off */
    0x03, 0x00, 0x00, 0x00, 0x03, 0x03,
    0x01, 0x00, 0x00, 0x00,
    0x00,
    0x00, 0x00,
    0x00,
    0x7B
};

static void recalc_status_checksum(void)
{
    uint8_t sum = 0;
    for (int i = 1; i < 23; i++) sum += s_amp_status[i];
    s_amp_status[23] = (uint8_t)(0x100u - sum);
}

/* ── Trasmissione verso l'amp ──────────────────────────────────────────
 * nomis protocol.rst, "Message formats": non inviare altre richieste mentre
 * l'amp sta inviando (o deve inviare) una risposta lunga AA..: le risposte da
 * un byte sovrascrivono parte del messaggio, una nuova risposta lunga lo fa
 * ricominciare. Quindi:
 *  - tutte le scritture passano da amp_tx(), sotto un mutex (ricorsivo, così
 *    una sequenza intera può essere tenuta con amp_lock/amp_unlock);
 *  - prima di scrivere si attende che non ci sia un messaggio AA in ricezione;
 *  - dopo una richiesta che produce una risposta lunga (0x34, 0x31, scrittura
 *    stato, 0x36) si attende che la risposta sia finita (segnalata da task_amp). */
static SemaphoreHandle_t s_amp_tx_mutex;
static SemaphoreHandle_t s_amp_frame_done;
static volatile bool     s_amp_rx_in_frame = false;
#define AMP_FRAME_WAIT_MS 500

static void amp_lock(void)   { xSemaphoreTakeRecursive(s_amp_tx_mutex, portMAX_DELAY); }
static void amp_unlock(void) { xSemaphoreGiveRecursive(s_amp_tx_mutex); }

static void amp_tx(const uint8_t *d, size_t n, bool expect_frame)
{
    amp_lock();
    for (int i = 0; i < AMP_FRAME_WAIT_MS / 10 && s_amp_rx_in_frame; i++)
        vTaskDelay(pdMS_TO_TICKS(10));
    if (expect_frame) xSemaphoreTake(s_amp_frame_done, 0);   /* azzera segnalazioni vecchie */
    uart_write_bytes(UART_AMP, d, n);
    if (expect_frame) {
        uart_wait_tx_done(UART_AMP, pdMS_TO_TICKS(100));
        if (xSemaphoreTake(s_amp_frame_done, pdMS_TO_TICKS(AMP_FRAME_WAIT_MS)) != pdTRUE)
            ESP_LOGD(TAG, "AMP: nessuna risposta lunga entro %d ms (cmd 0x%02X)",
                     AMP_FRAME_WAIT_MS, d[0]);
    }
    amp_unlock();
}

static void amp_tx1(uint8_t b, bool expect_frame)
{
    amp_tx(&b, 1, expect_frame);
}

static void sanitize_status_for_write(void);   /* definita dopo i valori salvati */

/* Scrittura stato completo: stesso formato della risposta a 0x34
 * (AA 0A 14 + 20 byte + checksum), come nomis "Write status/configuration".
 * I valori fuori limite vengono corretti prima di scrivere. */
static void send_write_status_to_amp(void)
{
    amp_lock();
    amp_tx1(0x30, false);   /* reset idle time, come la console a ogni input utente */

    sanitize_status_for_write();
    uint8_t msg[24];
    msg[0] = 0xAA; msg[1] = 0x0A; msg[2] = 0x14;
    memcpy(msg + 3, s_amp_status + 3, 20);
    uint8_t sum = 0;
    for (int i = 1; i < 23; i++) sum += msg[i];
    msg[23] = (uint8_t)(0x100u - sum);
    amp_tx(msg, sizeof(msg), true);   /* risposta attesa: AA FF 01 8A 76 */
    amp_unlock();
}

/* Cambia il volume con comandi incrementali (0x08/0x09) senza riscrittura DSP.
 * L'amp non muta l'audio durante questi comandi. */
static void send_volume_to_amp(int new_vol)
{
    int delta = new_vol - (int)s_amp_status[3];
    if (delta == 0) return;
    uint8_t cmd = (delta > 0) ? 0x08 : 0x09;
    int steps = delta > 0 ? delta : -delta;
    amp_lock();
    for (int i = 0; i < steps; i++)
        amp_tx1(cmd, false);
    amp_unlock();
    s_amp_status[3] = (uint8_t)new_vol;
    recalc_status_checksum();
}

/* Unmute come la console originale: 0x38, 0x08 ripetuti fino a new_vol, 0x39
 * (nomis protocol.rst, "Unmute"). Usato anche quando il volume cambia da mutato. */
static void send_unmute_to_amp(int new_vol)
{
    amp_lock();
    amp_tx1(0x38, false);
    send_volume_to_amp(new_vol);
    amp_tx1(0x39, false);
    amp_unlock();
}

/* ── Log hex ─────────────────────────────────────────────────────────
 * NOTA: buffer statico — chiamare solo da task_amp (singolo chiamante). */
static void log_hex(const char *dir, const uint8_t *d, int n)
{
    static char s[BUF * 3 + 4];
    int p = 0;
    for (int i = 0; i < n && p < (int)sizeof(s) - 4; i++)
        p += snprintf(s + p, sizeof(s) - p, "%02X ", d[i]);
    ESP_LOGI(TAG, "%s [%d]: %s", dir, n, s);
}

/* ════════════════════════════════════════════════════════════════════ */
/*  NEC IR PARSER                                                      */
/* ════════════════════════════════════════════════════════════════════ */

typedef enum { NEC_NONE, NEC_CMD, NEC_REPEAT } nec_type_t;

typedef struct {
    nec_type_t type;
    uint16_t   addr;
    uint8_t    cmd;
} nec_frame_t;

static inline bool nec_in_range(uint32_t us, uint32_t ref)
{
    return us >= ref - NEC_MARGIN && us <= ref + NEC_MARGIN;
}

/*
 * Decodifica un frame NEC1 (16-bit address + 8-bit cmd + complemento).
 * symbols: buffer RMT a 1 µs/tick.
 * Restituisce true se il frame è valido.
 */
static bool nec_parse(const rmt_symbol_word_t *s, size_t n, nec_frame_t *out)
{
    if (n < 2) return false;

    /* Ricostruisce il burst header 9ms che il glitch AGC del VS1838B può spezzare in
     * due parti. Accumula la durata LOW attraverso brevi spazi (<500µs) oppure salta
     * simboli spuri con spazio lungo. Avanza al massimo di 2 posizioni. */
    uint32_t hdr_burst = s[0].duration0;
    size_t   hdr_off   = 0;
    while (!nec_in_range(hdr_burst, NEC_HDR_HI) && hdr_off < 2 && n > (34 + hdr_off)) {
        uint32_t gap      = s[hdr_off].duration1;
        uint32_t combined = hdr_burst + s[hdr_off + 1].duration0;
        hdr_off++;
        if (gap < 500u && combined <= (NEC_HDR_HI + NEC_MARGIN))
            hdr_burst = combined;             /* glitch corto: fondi i due pezzi */
        else
            hdr_burst = s[hdr_off].duration0; /* simbolo spurio: riparti */
    }
    if (!nec_in_range(hdr_burst, NEC_HDR_HI)) return false;

    s += hdr_off;
    n -= hdr_off;
    uint32_t hdr_space = s[0].duration1;

    /* Repeat frame: spazio di 2.25 ms, solo 2 simboli */
    if (nec_in_range(hdr_space, NEC_RPT_LO) && n <= 3) {
        out->type = NEC_REPEAT;
        return true;
    }

    /* Full frame: spazio header di 4.5 ms + 32 bit dati */
    if (!nec_in_range(hdr_space, NEC_HDR_LO)) return false;
    if (n < 34) return false;

    uint16_t addr  = 0;
    uint16_t cmd16 = 0;

    /* 16 bit indirizzo (LSB first sul filo → bit 0 = primo trasmesso) */
    for (int i = 0; i < 16; i++) {
        if (!nec_in_range(s[1 + i].duration0, NEC_BIT_HI)) return false;
        addr >>= 1;
        if      (nec_in_range(s[1 + i].duration1, NEC_BIT0_LO)) { /* 0 */ }
        else if (nec_in_range(s[1 + i].duration1, NEC_BIT1_LO)) addr |= 0x8000u;
        else return false;
    }

    /* 16 bit comando (cmd + ~cmd, LSB first) */
    for (int i = 0; i < 16; i++) {
        if (!nec_in_range(s[17 + i].duration0, NEC_BIT_HI)) return false;
        cmd16 >>= 1;
        if      (nec_in_range(s[17 + i].duration1, NEC_BIT0_LO)) { /* 0 */ }
        else if (nec_in_range(s[17 + i].duration1, NEC_BIT1_LO)) cmd16 |= 0x8000u;
        else return false;
    }

    /* Verifica complemento comando */
    uint8_t cmd     = cmd16 & 0xFFu;
    uint8_t cmd_inv = (cmd16 >> 8) & 0xFFu;
    if ((cmd ^ cmd_inv) != 0xFFu) return false;

    out->type = NEC_CMD;
    out->addr = addr;
    out->cmd  = cmd;
    return true;
}

/* ════════════════════════════════════════════════════════════════════ */
/*  STATO GLOBALE                                                      */
/* ════════════════════════════════════════════════════════════════════ */

static volatile bool  s_zb_power      = false;
static volatile int   s_amp_actual_on = 0;
static volatile bool  s_zb_attr_dirty = false;
static volatile bool  s_poweron_busy  = false;

#define EP_POWER  1
#define EP_VOLUME 2
#define EP_INPUT  3
#define EP_REAR   4   /* livello posteriori  (0-43) */
#define EP_CENTER 5   /* livello centrale    (0-43) */
#define EP_SUB    6   /* livello subwoofer   (0-43) */
#define EP_FX3D   7   /* switch effetto 3D sull'ingresso corrente */
static bool zigbee_ready            = false;

#define COORDINATOR_PING_TICKS 5
static uint8_t s_ping_tick      = 0;
static void coordinator_ping_cb(uint8_t param);

static volatile TickType_t s_last_change_tick = 0;
#define ZHA_SYNC_DEBOUNCE_MS 2000

static volatile int8_t  s_zb_vol_pending = -1;
static volatile int8_t  s_zb_inp_pending = -1;
static volatile int8_t  s_zb_pwr_pending = -1;
static volatile int8_t  s_ir_vol_delta   =  0;  /* delta volume accumulato da IR */
static volatile bool    s_ir_mute_toggle = false; /* richiesta mute/unmute da IR */

/* Mute come la console originale (nomis protocol.rst, "Mute"/"Unmute"):
 * mute = volume a 0 con comandi 0x09; unmute = 0x38, 0x08 fino al volume salvato, 0x39.
 * Il volume "mutato" si azzera a ogni accensione. Usati solo da task_zb_cmd. */
static bool    s_muted     = false;
static uint8_t s_muted_vol = 0;
static SemaphoreHandle_t s_zb_cmd_sem;

static float s_vol_present_value   = 0.0f;
static float s_input_present_value = 1.0f;

static const uint8_t s_fx_idx[6] = {13, 11, 14, 9, 10, 12};

/* ── Livelli posteriori / centrale / subwoofer ─────────────────────────
 * nomis protocol.rst: byte di stato 4 = rear, 5 = centre, 6 = subwoofer (0-43);
 * comandi a passo singolo: 0E/0F rear su/giù, 0C/0D centre, 0A/0B subwoofer. */
typedef struct {
    uint8_t     ep;
    uint8_t     idx;      /* indice in s_amp_status */
    uint8_t     cmd_up;
    uint8_t     cmd_dn;
    uint8_t     max;      /* valore massimo ammesso */
    const char *name;
} level_ch_t;

#define LEVEL_COUNT 3
static const level_ch_t s_levels[LEVEL_COUNT] = {
    { EP_REAR,   4, 0x0E, 0x0F,  43, "posteriori" },
    /* Verificato sull'amp: i comandi 0x0C oltre 43 vengono ignorati (resta 0x2B) */
    { EP_CENTER, 5, 0x0C, 0x0D,  43, "centrale"   },
    { EP_SUB,    6, 0x0A, 0x0B,  43, "subwoofer"  },
};
static float           s_level_present[LEVEL_COUNT];
static volatile int8_t s_zb_level_pending[LEVEL_COUNT] = { -1, -1, -1 };

/* ── Effetto 3D (solo ingresso 3) ──────────────────────────────────────
 * Effetto per ingresso nei byte di stato s_fx_idx[] (0 = 3D, 1 = 2.1, 2 = 4.1,
 * 3 = disabilitato); comandi 0x14 = 3D, 0x35 = disabilitato (nomis protocol.rst).
 * Lo switch vale per l'ingresso 3 ed è salvato in NVS: è il valore di riferimento,
 * riapplicato all'amp quando si passa all'ingresso 3 e dopo l'accensione. */
#define FX3D_INPUT 2                      /* indice 2 = ingresso 3 */
static bool            s_fx3d_on          = false;
static volatile int8_t s_zb_fx3d_pending  = -1;

/* Copia l'impostazione 3D nel byte effetto dell'ingresso 3 dello stato locale */
static void fx3d_to_status(void)
{
    s_amp_status[s_fx_idx[FX3D_INPUT]] = s_fx3d_on ? 0x00 : 0x03;
    recalc_status_checksum();
}

/* Valore per HA dal byte di stato: limitato al massimo del canale */
static void levels_from_status(void)
{
    for (int i = 0; i < LEVEL_COUNT; i++) {
        uint8_t v = s_amp_status[s_levels[i].idx];
        s_level_present[i] = (float)(v > s_levels[i].max ? s_levels[i].max : v);
    }
}

/* Contatore delle risposte di stato (AA 0A 14) ricevute: serve per attendere
 * lo stato dell'amp dopo una richiesta 0x34. */
static volatile uint32_t s_status_rx_count = 0;

/* ── Ingresso salvato in NVS ───────────────────────────────────────────
 * L'amp salva le impostazioni solo con 0x36 (spegnimento, nomis protocol.rst).
 * Se la console perde alimentazione l'amp va in standby senza salvare e alla
 * riaccensione torna all'ultimo ingresso salvato: per questo l'ingresso scelto
 * da HA viene salvato anche qui e ripristinato dopo l'accensione. */
#define NVS_NS        "z906"
#define NVS_KEY_INPUT "input"
static uint8_t s_saved_input = 0xFF;   /* 0..5, 0xFF = nessun valore salvato */

static void nvs_load_input(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;  /* namespace assente al primo avvio */
    uint8_t v;
    if (nvs_get_u8(h, NVS_KEY_INPUT, &v) == ESP_OK && v < 6) s_saved_input = v;
    nvs_close(h);
}

static void nvs_save_input(uint8_t inp)
{
    if (inp >= 6 || inp == s_saved_input) return;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, NVS_KEY_INPUT, inp);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    if (err == ESP_OK) {
        s_saved_input = inp;
        ESP_LOGI(TAG, "NVS: ingresso salvato = %d", inp + 1);
    } else {
        ESP_LOGW(TAG, "NVS: salvataggio ingresso fallito (%s)", esp_err_to_name(err));
    }
}

/* Livelli posteriori / centrale / subwoofer: stesso motivo dell'ingresso
 * (verificato nei log: dopo uno stacco dell'alimentazione l'amp torna ai valori salvati) */
static const char *const s_nvs_key_level[LEVEL_COUNT] = { "lv_rear", "lv_center", "lv_sub" };
static uint8_t s_saved_level[LEVEL_COUNT] = { 0xFF, 0xFF, 0xFF };   /* 0xFF = non salvato */

static void nvs_load_levels(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    for (int i = 0; i < LEVEL_COUNT; i++) {
        uint8_t v;
        if (nvs_get_u8(h, s_nvs_key_level[i], &v) == ESP_OK && v <= s_levels[i].max)
            s_saved_level[i] = v;
    }
    nvs_close(h);
}

static void nvs_save_level(int i, uint8_t v)
{
    if (v > s_levels[i].max || v == s_saved_level[i]) return;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, s_nvs_key_level[i], v);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    if (err == ESP_OK) {
        s_saved_level[i] = v;
        ESP_LOGI(TAG, "NVS: %s salvato = %d", s_levels[i].name, v);
    } else {
        ESP_LOGW(TAG, "NVS: salvataggio %s fallito (%s)", s_levels[i].name, esp_err_to_name(err));
    }
}

#define NVS_KEY_FX3D "fx3d"

static void nvs_load_fx3d(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t v;
    if (nvs_get_u8(h, NVS_KEY_FX3D, &v) == ESP_OK) s_fx3d_on = (v != 0);
    nvs_close(h);
}

static void nvs_save_fx3d(bool on)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, NVS_KEY_FX3D, on ? 1 : 0);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    if (err == ESP_OK) ESP_LOGI(TAG, "NVS: 3D ingresso 3 salvato = %s", on ? "ON" : "OFF");
    else               ESP_LOGW(TAG, "NVS: salvataggio 3D fallito (%s)", esp_err_to_name(err));
}

/* ── Limiti dei campi di stato (nomis protocol.rst, "Read status") ─────
 * volume e livelli 0-43, ingresso 0-5, mute 0-1, effetti 0-3.
 * Uno stato con checksum valido è ciò che l'amp ha davvero in memoria: viene
 * accettato anche se contiene valori fuori limite (altrimenti la sincronizzazione
 * si bloccherebbe), ma le anomalie vengono segnalate nel log e corrette alla
 * successiva scrittura dello stato (sanitize_status_for_write). */
static bool status_field_ok(const uint8_t *st, int idx)
{
    switch (idx) {
    case 3: case 4: case 5: case 6: return st[idx] <= 43;
    case 7:                         return st[idx] <= 5;
    case 8:                         return st[idx] <= 1;
    case 9: case 10: case 11:
    case 12: case 13: case 14:      return st[idx] <= 3;
    default:                        return true;
    }
}

static void status_check_ranges(const uint8_t *st)
{
    static uint32_t s_last_mask = 0;
    uint32_t mask = 0;
    for (int idx = 3; idx <= 14; idx++)
        if (!status_field_ok(st, idx)) mask |= (1u << idx);
    if (mask == s_last_mask) return;   /* segnala solo quando cambia */
    s_last_mask = mask;
    if (mask == 0) {
        ESP_LOGI(TAG, "AMP: stato di nuovo tutto nei limiti");
        return;
    }
    for (int idx = 3; idx <= 14; idx++)
        if (mask & (1u << idx))
            ESP_LOGW(TAG, "AMP: byte di stato %d fuori limite (0x%02X)", idx, st[idx]);
}

/* Prima di una scrittura completa: sostituisce i valori fuori limite con quelli
 * salvati in NVS o, se assenti, con i valori del "Configuration reset" nomis
 * (volume 0x0A, livelli 0x15, effetto ingresso 1 = 3, altri = 0). */
static void sanitize_status_for_write(void)
{
    static const uint8_t reset_val[23] = {
        [3] = 0x0A, [4] = 0x15, [5] = 0x15, [6] = 0x15, [7] = 0x00, [8] = 0x00,
        [9] = 0x00, [10] = 0x00, [11] = 0x00, [12] = 0x00, [13] = 0x03, [14] = 0x00,
    };
    bool fixed = false;
    for (int idx = 3; idx <= 14; idx++) {
        if (status_field_ok(s_amp_status, idx)) continue;
        uint8_t v = reset_val[idx];
        if (idx == 7 && s_saved_input < 6) v = s_saved_input;
        for (int i = 0; i < LEVEL_COUNT; i++)
            if (idx == s_levels[i].idx && s_saved_level[i] <= s_levels[i].max) v = s_saved_level[i];
        if (idx == s_fx_idx[FX3D_INPUT]) v = s_fx3d_on ? 0x00 : 0x03;
        ESP_LOGW(TAG, "Scrittura stato: byte %d corretto 0x%02X → 0x%02X", idx, s_amp_status[idx], v);
        s_amp_status[idx] = v;
        fixed = true;
    }
    if (fixed) recalc_status_checksum();
}

/* Cambia un livello con comandi a passo singolo, come send_volume_to_amp.
 * Se il valore attuale è fuori scala (> max, visto nei log: rear=0xAA) i passi
 * non hanno un punto di partenza valido: si riscrive lo stato completo. */
static void send_level_to_amp(int i, int new_val)
{
    uint8_t idx = s_levels[i].idx;
    int     cur = s_amp_status[idx];
    if (cur > s_levels[i].max) {
        s_amp_status[idx] = (uint8_t)new_val;
        recalc_status_checksum();
        send_write_status_to_amp();
        ESP_LOGW(TAG, "Livello %s fuori scala (%d): riscritto stato completo",
                 s_levels[i].name, cur);
        return;
    }
    int delta = new_val - cur;
    if (delta == 0) return;
    uint8_t cmd   = (delta > 0) ? s_levels[i].cmd_up : s_levels[i].cmd_dn;
    int     steps = delta > 0 ? delta : -delta;
    amp_lock();
    for (int k = 0; k < steps; k++)
        amp_tx1(cmd, false);
    amp_unlock();
    s_amp_status[idx] = (uint8_t)new_val;
    recalc_status_checksum();
}

/* ── IR / LED handles ──────────────────────────────────────────────── */
static rmt_channel_handle_t      s_ir_channel = NULL;
static QueueHandle_t             s_ir_queue;
static rmt_symbol_word_t         s_ir_symbols[IR_BUF_SYMBOLS];
static const rmt_receive_config_t s_ir_recv_cfg = {
    .signal_range_min_ns = 1250,
    .signal_range_max_ns = 12000000,
};

static led_strip_handle_t s_led_strip;

/* ════════════════════════════════════════════════════════════════════ */
/*  TASK: lettura UART dall'amp                                        */
/* ════════════════════════════════════════════════════════════════════ */

static void task_amp(void *pv)
{
    uint8_t buf[BUF];
    uint8_t aa_rx[32];
    int     aa_rx_pos         = 0;
    bool    aa_rx_active      = false;
    TickType_t aa_rx_start_ms = 0;

    for (;;) {
        int n = uart_read_bytes(UART_AMP, buf, sizeof(buf), pdMS_TO_TICKS(10));

        if (aa_rx_active &&
            (xTaskGetTickCount() - aa_rx_start_ms) >= pdMS_TO_TICKS(300)) {
            ESP_LOGW(TAG, "AMP: messaggio incompleto scartato (pos=%d)", aa_rx_pos);
            aa_rx_active      = false;
            aa_rx_pos         = 0;
            s_amp_rx_in_frame = false;
            xSemaphoreGive(s_amp_frame_done);
        }

        if (n <= 0) continue;
        log_hex("AMP→", buf, n);

        for (int i = 0; i < n; i++) {
            uint8_t b = buf[i];

            if (aa_rx_active) {
                if (aa_rx_pos < (int)sizeof(aa_rx)) aa_rx[aa_rx_pos++] = b;
                aa_rx_start_ms = xTaskGetTickCount();
                if (aa_rx_pos >= 3) {
                    int expected = 3 + aa_rx[2] + 1;
                    if (expected > (int)sizeof(aa_rx)) {
                        /* Lunghezza impossibile per i messaggi noti: byte non allineati */
                        ESP_LOGW(TAG, "AMP: lunghezza non valida (%d), messaggio scartato", aa_rx[2]);
                        aa_rx_active      = false;
                        aa_rx_pos         = 0;
                        s_amp_rx_in_frame = false;
                        xSemaphoreGive(s_amp_frame_done);
                        continue;
                    }
                    if (aa_rx_pos >= expected) {
                        /* Checksum nomis: somma U8 da tipo a checksum inclusi = 0 */
                        uint8_t sum = 0;
                        for (int k = 1; k < expected; k++) sum += aa_rx[k];
                        bool cks_ok = (sum == 0);
                        if (!cks_ok) {
                            ESP_LOGW(TAG, "AMP: checksum errato, messaggio scartato");
                        } else if (aa_rx[1] == 0x0A && aa_rx[2] == 0x14 && aa_rx_pos == 24) {
                            status_check_ranges(aa_rx);
                            aa_rx[0] = 0xAA;
                            memcpy(s_amp_status, aa_rx, 24);
                            recalc_status_checksum();
                            s_vol_present_value   = (float)s_amp_status[3];
                            s_input_present_value = (float)(s_amp_status[7] + 1);
                            levels_from_status();
                            s_status_rx_count++;

                            if (s_amp_status[20] == 0 && s_zb_power) {
                                s_amp_actual_on = 1;
                            } else if (!s_zb_power) {
                                s_amp_actual_on = 0;
                            }

                            s_zb_attr_dirty = true;
                            ESP_LOGI(TAG, "Sync amp: on=%d vol=%d input=%d mute=%d",
                                     s_amp_actual_on, s_amp_status[3],
                                     s_amp_status[7] + 1, s_amp_status[8]);
                        }
                        aa_rx_active      = false;
                        aa_rx_pos         = 0;
                        s_amp_rx_in_frame = false;
                        xSemaphoreGive(s_amp_frame_done);
                    }
                }
                continue;
            }

            if (b == 0xAA) {
                aa_rx_active      = true;
                s_amp_rx_in_frame = true;
                aa_rx_pos         = 0;
                aa_rx_start_ms    = xTaskGetTickCount();
                aa_rx[aa_rx_pos++] = b;
                continue;
            }

            if (b == 0x11 && s_amp_actual_on == 0 && s_zb_power) {
                s_amp_actual_on = 1;
                s_zb_attr_dirty = true;
                ESP_LOGI(TAG, "AMP confermato ON (echo 0x11)");
            }
        }
    }
}

/* ── Task sync: 0x34 ogni 30s, 0x31 ogni 60s ──────────────────────── */
static void task_amp_sync(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    {
        amp_tx1(0x34, true);
        ESP_LOGI(TAG, "Sync startup: rilevamento stato amp");
        vTaskDelay(pdMS_TO_TICKS(1000));
        s_zb_attr_dirty = true;
        ESP_LOGI(TAG, "Stato amp rilevato: %s", s_amp_actual_on ? "ON" : "OFF");
    }

    TickType_t last_full_sync  = xTaskGetTickCount();
    TickType_t last_idle_query = xTaskGetTickCount() - pdMS_TO_TICKS(45000);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (s_poweron_busy) continue;

        TickType_t now = xTaskGetTickCount();

        if (s_last_change_tick != 0 &&
            (now - s_last_change_tick) >= pdMS_TO_TICKS(ZHA_SYNC_DEBOUNCE_MS)) {
            s_last_change_tick = 0;
            amp_tx1(0x34, true);
            ESP_LOGI(TAG, "Sync ZHA post-change");
        }

        if ((now - last_full_sync) >= pdMS_TO_TICKS(30000)) {
            last_full_sync = now;
            amp_tx1(0x34, true);
            ESP_LOGI(TAG, "Sync: richiesto status all'amp");
        }

        if ((now - last_idle_query) >= pdMS_TO_TICKS(60000)) {
            last_idle_query = now;
            amp_tx1(0x31, true);
            ESP_LOGD(TAG, "Idle query (0x31) → amp");
        }
    }
}

/* ── Task GPIO: pin15 e pin8 ────────────────────────────────────────── */
static void task_gpio(void *pv)
{
    int prev15 = -1, prev8 = -1;
    for (;;) {
        int amp15 = s_zb_power ? 1 : 0;
        if (amp15 != prev15) {
            gpio_set_level(GPIO_AMP15_OUT, amp15);
            if (amp15 == 0) {
                s_amp_actual_on = 0;
                s_zb_attr_dirty = true;
            }
            ESP_LOGI(TAG, "pin15: %s", amp15 ? "ON" : "OFF");
            prev15 = amp15;
        }

        int a8 = gpio_get_level(GPIO_AMP8_IN);
        if (a8 != prev8) {
            if (a8 == 1 && s_zb_power && s_amp_actual_on == 1) {
                ESP_LOGW(TAG, "pin8: HIGH inatteso — amp perso alimentazione");
                s_zb_power      = false;
                s_amp_actual_on = 0;
                s_zb_attr_dirty = true;
                if (zigbee_ready && esp_zb_lock_acquire(pdMS_TO_TICKS(2000))) {
                    s_ping_tick = COORDINATOR_PING_TICKS - 1;
                    esp_zb_scheduler_alarm_cancel(coordinator_ping_cb, 0);
                    esp_zb_scheduler_alarm(coordinator_ping_cb, 0, 0);
                    esp_zb_lock_release();
                }
            }
            prev8 = a8;
        }
        vTaskDelay(pdMS_TO_TICKS(GPIO_POLL_MS));
    }
}

/* ════════════════════════════════════════════════════════════════════ */
/*  TASK: ricevitore IR                                                */
/* ════════════════════════════════════════════════════════════════════ */

static bool IRAM_ATTR ir_rx_done_cb(rmt_channel_handle_t ch,
                                     const rmt_rx_done_event_data_t *edata,
                                     void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR((QueueHandle_t)user_ctx, edata, &woken);
    return woken == pdTRUE;
}

static void task_ir(void *pv)
{
    rmt_rx_done_event_data_t rx_data;
    TickType_t last_vol_tick = 0;
    uint8_t    last_vol_cmd  = 0;  /* salva direzione per i repeat frame */
    TickType_t last_rx_tick  = 0;  /* ultimo frame/repeat valido del Z906 */

    rmt_receive(s_ir_channel, s_ir_symbols, sizeof(s_ir_symbols), &s_ir_recv_cfg);

    for (;;) {
        /* Il ricevitore resta in ascolto finché non arriva un frame: niente riarmo
         * periodico (IDF 5.5: rmt_receive su canale già in ascolto → ESP_ERR_INVALID_STATE) */
        if (xQueueReceive(s_ir_queue, &rx_data, portMAX_DELAY) == pdFALSE)
            continue;

        nec_frame_t frame = {0};
        bool ok = nec_parse(rx_data.received_symbols, rx_data.num_symbols, &frame);
        esp_err_t rx_err = rmt_receive(s_ir_channel, s_ir_symbols, sizeof(s_ir_symbols), &s_ir_recv_cfg);
        if (rx_err != ESP_OK)
            ESP_LOGW(TAG, "IR: riarmo ricevitore fallito (%s)", esp_err_to_name(rx_err));

        if (!ok) {
            /* Frame non NEC (es. altri telecomandi): scartato in silenzio.
             * Livello DEBUG: non stampato con il livello di log di default (INFO). */
            ESP_LOGD(TAG, "IR: parse fallito (n=%d)", (int)rx_data.num_symbols);
            continue;
        }

        TickType_t now = xTaskGetTickCount();

        if (frame.type == NEC_CMD) {
            /* Frame NEC di altri telecomandi: scartato in silenzio (log solo a livello DEBUG)
             * e interrompe un'eventuale sequenza di repeat del Z906. */
            if (frame.addr != NEC_ADDR_Z906) {
                ESP_LOGD(TAG, "IR: altro telecomando addr=0x%04X cmd=0x%02X", frame.addr, frame.cmd);
                last_vol_cmd = 0;
                continue;
            }
            ESP_LOGI(TAG, "IR: addr=0x%04X cmd=0x%02X", frame.addr, frame.cmd);

            /* Salva il comando per i repeat che seguono (i repeat NEC non portano cmd).
             * Un comando non-volume azzera, così i suoi repeat non ripetono il volume. */
            last_vol_cmd  = (frame.cmd == NEC_CMD_VOL_UP || frame.cmd == NEC_CMD_VOL_DN)
                            ? frame.cmd : 0;
            last_rx_tick  = now;
            last_vol_tick = now - pdMS_TO_TICKS(IR_REPEAT_MS);
        } else if (frame.type == NEC_REPEAT) {
            /* I repeat NEC non contengono l'indirizzo: validi solo se seguono a breve
             * (frame o repeat precedente del Z906, periodo NEC 108 ms) */
            if (last_vol_cmd == 0 ||
                (now - last_rx_tick) > pdMS_TO_TICKS(IR_REPEAT_WINDOW_MS)) {
                last_vol_cmd = 0;
                continue;
            }
            last_rx_tick = now;
            frame.cmd = last_vol_cmd;
            if ((now - last_vol_tick) < pdMS_TO_TICKS(IR_REPEAT_MS)) continue;
        } else {
            continue;
        }

        if (!s_amp_actual_on) continue;

        /* Mute: solo su frame completo (i repeat sono già esclusi: last_vol_cmd = 0) */
        if (frame.type == NEC_CMD && frame.cmd == NEC_CMD_MUTE) {
            s_ir_mute_toggle = true;
            xSemaphoreGive(s_zb_cmd_sem);
            ESP_LOGI(TAG, "IR: mute");
            continue;
        }

        if (frame.cmd != NEC_CMD_VOL_UP && frame.cmd != NEC_CMD_VOL_DN) continue;

        last_vol_tick = now;
        int delta = (frame.cmd == NEC_CMD_VOL_UP) ? +1 : -1;
        s_ir_vol_delta += delta;
        xSemaphoreGive(s_zb_cmd_sem);
        ESP_LOGI(TAG, "IR: vol%s", delta > 0 ? "+" : "-");
    }
}

/* ════════════════════════════════════════════════════════════════════ */
/*  TASK: LED WS2812B                                                  */
/* ════════════════════════════════════════════════════════════════════ */

/*
 * Barra volume: 11 LED × 4 livelli = 44 slot → vol 0-43.
 * - LED[i] pieno quando vol >= (i+1)*4
 * - LED[i] parziale quando (i*4 < vol < (i+1)*4): step = vol - i*4, livello 1-4
 * - LED[i] spento quando vol <= i*4
 * Colore amber: R=bri, G=bri/3, B=0
 */
/* Colori calcolati nel ciclo corrente e ultimi inviati alla catena LED.
 * La catena viene riscritta solo se qualcosa cambia, più un refresh ogni
 * LED_FORCE_REFRESH_MS per correggere un eventuale frame corrotto: ogni invio
 * RMT (senza DMA su H2) dipende dall'ISR di ricarica, se ritarda i bit escono
 * sbagliati e un LED spento lampeggia (primo byte WS2812 = verde). */
#define LED_FORCE_REFRESH_MS 1000
static uint8_t s_led_cur[LED_NUM][3];
static uint8_t s_led_sent[LED_NUM][3];

static void led_set(int idx, uint8_t r, uint8_t g, uint8_t b)
{
    s_led_cur[idx][0] = r;
    s_led_cur[idx][1] = g;
    s_led_cur[idx][2] = b;
}

static void task_led(void *pv)
{
    uint32_t   tick = 0;
    TickType_t last_refresh = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LED_TASK_MS));
        tick++;

        /* ── LED[0]: stato console ───────────────────────────────── */
        uint8_t sr = 0, sg = 0, sb = 0;
        if (s_poweron_busy) {
            /* Verde lampeggio veloce (2 tick = 100ms) */
            if ((tick & 1u) == 0u) sg = 60;
        } else if (!zigbee_ready) {
            /* Rosso lampeggio lento (10 tick = 500ms per semi-periodo) */
            if ((tick % 20u) < 10u) sr = 60;
        } else if (!s_amp_actual_on) {
            /* ZB connesso, amp spento: blu fisso basso */
            sb = 20;
        } else {
            /* Amp acceso: verde fisso */
            sg = 50;
        }
        led_set(LED_STATUS, sr, sg, sb);

        /* ── LED[1]: input selezionato ───────────────────────────── */
        {
            int inp = s_amp_actual_on ? (int)(s_input_present_value + 0.5f) : 0;
            if (inp < 0 || inp > 6) inp = 0;
            led_set(LED_INPUT, s_inp_col[inp][0], s_inp_col[inp][1], s_inp_col[inp][2]);
        }

        /* ── LED[2-12]: barra volume ─────────────────────────────── */
        {
            int vol = s_amp_actual_on ? (int)(s_vol_present_value + 0.5f) : 0;
            if (vol < 0) vol = 0;
            if (vol > 43) vol = 43;

            for (int i = 0; i < LED_VOL_COUNT; i++) {
                uint8_t bri = 0;
                int threshold_full    = (i + 1) * 4;
                int threshold_partial =  i      * 4;

                if (vol >= threshold_full) {
                    bri = s_vol_bri[3];
                } else if (vol > threshold_partial) {
                    int step = vol - threshold_partial; /* 1..4 */
                    bri = s_vol_bri[step - 1];
                }
                led_set(LED_VOL_BASE + i, bri, bri / 3u, 0);
            }
        }

        /* Invio solo se cambiato, oppure refresh periodico di sicurezza */
        TickType_t now = xTaskGetTickCount();
        bool changed = memcmp(s_led_cur, s_led_sent, sizeof(s_led_cur)) != 0;
        if (changed || (now - last_refresh) >= pdMS_TO_TICKS(LED_FORCE_REFRESH_MS)) {
            for (int i = 0; i < LED_NUM; i++)
                led_strip_set_pixel(s_led_strip, i,
                    s_led_cur[i][0], s_led_cur[i][1], s_led_cur[i][2]);
            led_strip_refresh(s_led_strip);
            memcpy(s_led_sent, s_led_cur, sizeof(s_led_cur));
            last_refresh = now;
        }
    }
}

/* ════════════════════════════════════════════════════════════════════ */
/*  TASK: comandi Zigbee/IR → amp                                      */
/* ════════════════════════════════════════════════════════════════════ */

static void task_zb_cmd(void *pv)
{
    for (;;) {
        xSemaphoreTake(s_zb_cmd_sem, portMAX_DELAY);

        /* Power */
        int8_t pwr = s_zb_pwr_pending;
        if (pwr >= 0) {
            s_zb_pwr_pending = -1;
            s_muted = false;   /* il volume mutato si azzera a ogni accensione/spegnimento */
            if (pwr == 1) {
                s_poweron_busy  = true;
                s_amp_actual_on = 0;

                amp_tx1(0x30, false);
                vTaskDelay(pdMS_TO_TICKS(500));
                vTaskDelay(pdMS_TO_TICKS(3500));

                static const uint8_t fx_to_cmd[4] = {0x14, 0x16, 0x15, 0x35};
                uint8_t cur_inp  = s_amp_status[7];
                if (cur_inp == FX3D_INPUT) fx3d_to_status();   /* 3D salvato per l'ingresso 3 */
                uint8_t fx_val   = (cur_inp < 6) ? s_amp_status[s_fx_idx[cur_inp]] : 3;
                uint8_t effect_b = fx_to_cmd[fx_val < 4 ? fx_val : 3];

                const uint8_t poweron_seq[] = {
                    0x11, 0x11, effect_b, 0x39, 0x38, 0x30, 0x39
                };
                amp_tx(poweron_seq, sizeof(poweron_seq), false);

                for (int i = 0; i < 300 && s_amp_actual_on == 0; i++)
                    vTaskDelay(pdMS_TO_TICKS(10));

                if (s_amp_actual_on == 0) {
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    amp_tx(poweron_seq, sizeof(poweron_seq), false);
                    for (int i = 0; i < 300 && s_amp_actual_on == 0; i++)
                        vTaskDelay(pdMS_TO_TICKS(10));
                }

                s_poweron_busy = false;

                if (s_amp_actual_on == 0) {
                    ESP_LOGW(TAG, "power ON: amp non risponde → revert ZHA a OFF");
                    s_zb_power      = false;
                    s_zb_attr_dirty = true;
                } else {
                    amp_tx1(0x39, false);   /* mute off */
                    vTaskDelay(pdMS_TO_TICKS(200));
                    uint32_t rx_before = s_status_rx_count;
                    amp_tx1(0x34, true);
                    ESP_LOGI(TAG, "ZB→power ON: confermato");

                    /* Ripristino ingresso salvato: attende lo stato dell'amp (max 1 s) */
                    for (int i = 0; i < 100 && s_status_rx_count == rx_before; i++)
                        vTaskDelay(pdMS_TO_TICKS(10));
                    if (s_status_rx_count == rx_before) {
                        ESP_LOGW(TAG, "power ON: stato amp non ricevuto, ingresso non verificato");
                    } else if (s_saved_input < 6 && s_amp_status[7] != s_saved_input) {
                        uint8_t si = s_saved_input;
                        s_amp_status[7] = si;
                        fx3d_to_status();   /* l'effetto dell'ingresso 3 segue lo switch 3D */
                        send_write_status_to_amp();
                        s_input_present_value = (float)(si + 1);
                        s_last_change_tick    = xTaskGetTickCount();
                        s_zb_attr_dirty       = true;
                        ESP_LOGI(TAG, "power ON: ripristinato ingresso %d", si + 1);
                    } else if (s_amp_status[7] == FX3D_INPUT &&
                               s_amp_status[s_fx_idx[FX3D_INPUT]] != (s_fx3d_on ? 0x00 : 0x03)) {
                        /* Ingresso già corretto ma effetto diverso dallo switch 3D salvato */
                        amp_tx1(s_fx3d_on ? 0x14 : 0x35, false);
                        fx3d_to_status();
                        s_last_change_tick = xTaskGetTickCount();
                        ESP_LOGI(TAG, "power ON: ripristinato 3D %s", s_fx3d_on ? "ON" : "OFF");
                    }

                    /* Ripristino livelli salvati (posteriori / centrale / subwoofer) */
                    if (s_status_rx_count != rx_before) {
                        bool changed = false;
                        for (int i = 0; i < LEVEL_COUNT; i++) {
                            uint8_t sv = s_saved_level[i];
                            if (sv > s_levels[i].max || s_amp_status[s_levels[i].idx] == sv) continue;
                            ESP_LOGI(TAG, "power ON: ripristino %s %d → %d", s_levels[i].name,
                                     s_amp_status[s_levels[i].idx], sv);
                            send_level_to_amp(i, sv);
                            s_level_present[i] = (float)sv;
                            changed = true;
                        }
                        if (changed) {
                            s_last_change_tick = xTaskGetTickCount();
                            s_zb_attr_dirty    = true;
                        }
                    }
                }
            } else {
                s_amp_actual_on = 0;
                s_zb_attr_dirty = true;
                const uint8_t poweroff[] = {0x30, 0x37, 0x36};
                amp_tx(poweroff, sizeof(poweroff), true);   /* 0x36: possibile risposta AA FF 01 8A 76 */
                vTaskDelay(pdMS_TO_TICKS(200));
                amp_tx1(0x34, true);
                ESP_LOGI(TAG, "ZB→power OFF");
            }
        }

        /* Mute/unmute da IR */
        if (s_ir_mute_toggle) {
            s_ir_mute_toggle = false;
            if (!s_muted) {
                if (s_amp_status[3] > 0) {
                    s_muted_vol = s_amp_status[3];
                    send_volume_to_amp(0);
                    s_muted = true;
                    ESP_LOGI(TAG, "IR→mute (volume salvato=%d)", s_muted_vol);
                } else {
                    ESP_LOGI(TAG, "IR→mute ignorato: volume già 0");
                }
            } else {
                send_unmute_to_amp(s_muted_vol);
                s_muted = false;
                ESP_LOGI(TAG, "IR→unmute (vol=%d)", s_muted_vol);
            }
            s_vol_present_value = (float)s_amp_status[3];
            s_last_change_tick  = xTaskGetTickCount();
            s_zb_attr_dirty     = true;
        }

        /* Volume da IR (delta accumulato) — processa prima di ZB per permettere override.
         * Se mutato: la console toglie il mute al primo cambio volume (nomis), ripartendo
         * dal volume salvato. */
        int8_t ir_delta = s_ir_vol_delta;
        if (ir_delta != 0) {
            s_ir_vol_delta = 0;
            int base = s_muted ? (int)s_muted_vol : (int)s_amp_status[3];
            int v = base + ir_delta;
            if (v < 0) v = 0;
            if (v > 43) v = 43;
            if (s_muted) {
                s_muted = false;
                send_unmute_to_amp(v);
                s_vol_present_value = (float)v;
                s_last_change_tick  = xTaskGetTickCount();
                s_zb_attr_dirty     = true;
                ESP_LOGI(TAG, "IR→unmute, vol=%d", v);
            } else if ((uint8_t)v != s_amp_status[3]) {
                send_volume_to_amp(v);
                s_vol_present_value = (float)v;
                s_last_change_tick  = xTaskGetTickCount();
                s_zb_attr_dirty     = true;
                ESP_LOGI(TAG, "IR→vol=%d", v);
            }
        }

        /* Volume da ZB (valore assoluto) */
        int8_t vol = s_zb_vol_pending;
        if (vol >= 0) {
            s_zb_vol_pending = -1;
            if (s_muted) {
                /* Volume da HA mentre mutato: togli il mute e vai al valore richiesto */
                s_muted = false;
                send_unmute_to_amp(vol);
                s_vol_present_value = (float)vol;
                s_last_change_tick  = xTaskGetTickCount();
                ESP_LOGI(TAG, "ZB→unmute, vol=%d", vol);
            } else if ((uint8_t)vol != s_amp_status[3]) {
                send_volume_to_amp(vol);
                s_vol_present_value = (float)vol;
                s_last_change_tick  = xTaskGetTickCount();
                ESP_LOGI(TAG, "ZB→vol=%d", vol);
            }
        }

        /* Livelli posteriori / centrale / subwoofer da ZB (valore assoluto) */
        for (int i = 0; i < LEVEL_COUNT; i++) {
            int8_t lv = s_zb_level_pending[i];
            if (lv < 0) continue;
            s_zb_level_pending[i] = -1;
            nvs_save_level(i, (uint8_t)lv);   /* non fa nulla se uguale al valore salvato */
            if ((uint8_t)lv != s_amp_status[s_levels[i].idx]) {
                send_level_to_amp(i, lv);
                s_level_present[i] = (float)lv;
                s_last_change_tick = xTaskGetTickCount();
                ESP_LOGI(TAG, "ZB→%s=%d", s_levels[i].name, lv);
            }
        }

        /* Effetto 3D da ZB: si applica all'ingresso corrente */
        int8_t fx = s_zb_fx3d_pending;
        if (fx >= 0) {
            s_zb_fx3d_pending = -1;
            bool on = (fx != 0);
            if (on != s_fx3d_on) {
                s_fx3d_on = on;
                nvs_save_fx3d(on);
            }
            fx3d_to_status();
            if (s_amp_actual_on && s_amp_status[7] == FX3D_INPUT) {
                amp_tx1(on ? 0x14 : 0x35, false);
                s_last_change_tick = xTaskGetTickCount();
                ESP_LOGI(TAG, "ZB→3D %s (applicato all'ingresso 3)", on ? "ON" : "OFF");
            } else {
                ESP_LOGI(TAG, "ZB→3D %s (salvato, applicato con l'ingresso 3)", on ? "ON" : "OFF");
            }
            s_zb_attr_dirty = true;
        }

        /* Input da ZB */
        int8_t inp = s_zb_inp_pending;
        if (inp >= 0) {
            s_zb_inp_pending = -1;
            nvs_save_input((uint8_t)inp);   /* non fa nulla se uguale al valore già salvato */
            if ((uint8_t)inp != s_amp_status[7]) {
                s_amp_status[7] = (uint8_t)inp;
                /* Effetto: resta quello memorizzato per l'ingresso; per l'ingresso 3
                 * segue lo switch 3D salvato */
                if (inp == FX3D_INPUT) fx3d_to_status();
                recalc_status_checksum();
                send_write_status_to_amp();
                s_last_change_tick = xTaskGetTickCount();
                ESP_LOGI(TAG, "ZB→input=%d", inp + 1);
            }
        }
    }
}

/* ════════════════════════════════════════════════════════════════════ */
/*  ZIGBEE                                                             */
/* ════════════════════════════════════════════════════════════════════ */

#define INSTALLCODE_POLICY_ENABLE   false
#define MAX_CHILDREN                10
#define ESP_ZB_PRIMARY_CHANNEL_MASK (1UL << 25)
#define OTA_UPGRADE_QUERY_INTERVAL  240

#define ESP_MANUFACTURER_NAME  "\x09""Handmade!"
#define ESP_MODEL_IDENTIFIER   "\x0C""Z906 Console"

#define ESP_ZB_ZR_CONFIG() {                                          \
    .esp_zb_role         = ESP_ZB_DEVICE_TYPE_ROUTER,                 \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE,                 \
    .nwk_cfg.zczr_cfg    = { .max_children = MAX_CHILDREN },          \
}
#define ESP_ZB_DEFAULT_RADIO_CONFIG() { .radio_mode = ZB_RADIO_MODE_NATIVE }
#define ESP_ZB_DEFAULT_HOST_CONFIG()  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE }

static bool s_coordinator_confirmed = false;
static bool s_ping_in_flight        = false;
static bool s_reporting_ready       = false;

#define COORDINATOR_PING_TICK_MS    60000
#define COORDINATOR_PING_TIMEOUT_MS 40000

static void coordinator_ping_timeout_cb(uint8_t param);
static void zb_attr_sync_cb(uint8_t param);

static esp_zb_attribute_list_t *create_basic_cluster(void)
{
    esp_zb_basic_cluster_cfg_t cfg = {
        .zcl_version  = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,
    };
    esp_zb_attribute_list_t *c = esp_zb_basic_cluster_create(&cfg);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  ESP_MODEL_IDENTIFIER);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID,          OTA_SW_BUILD_ID);
    return c;
}

static void add_power_endpoint(esp_zb_ep_list_t *ep_list)
{
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cl, create_basic_cluster(),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_identify_cluster_cfg_t id_cfg = { .identify_time = 0 };
    esp_zb_cluster_list_add_identify_cluster(cl,
        esp_zb_identify_cluster_create(&id_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    uint8_t on_off_val = s_zb_power ? 1 : 0;
    esp_zb_on_off_cluster_cfg_t oo = { .on_off = on_off_val };
    esp_zb_attribute_list_t *on_off_attrs = esp_zb_on_off_cluster_create(&oo);
    uint8_t startup_on_off = 0x00;
    esp_zb_on_off_cluster_add_attr(on_off_attrs,
        ESP_ZB_ZCL_ATTR_ON_OFF_START_UP_ON_OFF, &startup_on_off);
    esp_zb_cluster_list_add_on_off_cluster(cl, on_off_attrs,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    /* CLIENT role: come per gli Analog Output, perché report_attr_cmd_req con
       direction=TO_CLI non vada in assert (esp-idf#15962) */
    esp_zb_cluster_list_add_on_off_cluster(cl, esp_zb_on_off_cluster_create(&oo),
        ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

#if ZB_OTA_ENABLED
    esp_zb_cluster_list_add_ota_cluster(cl, ota_cluster_create(),
        ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
#endif

    esp_zb_endpoint_config_t epcfg = {
        .endpoint = EP_POWER, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID, .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
}

static void add_volume_endpoint(esp_zb_ep_list_t *ep_list)
{
    esp_zb_analog_output_cluster_cfg_t ao_cfg = {
        .out_of_service = false, .present_value = s_vol_present_value, .status_flags = 0,
    };
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cl, create_basic_cluster(),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_attribute_list_t *ao_vol = esp_zb_analog_output_cluster_create(&ao_cfg);
    static float s_vol_min = 0.0f, s_vol_max = 43.0f;
    esp_zb_analog_output_cluster_add_attr(ao_vol,
        ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, &s_vol_min);
    esp_zb_analog_output_cluster_add_attr(ao_vol,
        ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, &s_vol_max);
    esp_zb_cluster_list_add_analog_output_cluster(cl, ao_vol,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    /* CLIENT role necessario perché report_attr_cmd_req con direction=TO_CLI
       non vada in assert (esp-idf#15962) */
    esp_zb_cluster_list_add_analog_output_cluster(cl,
        esp_zb_analog_output_cluster_create(&ao_cfg),
        ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_endpoint_config_t epcfg = {
        .endpoint = EP_VOLUME, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = 0x000D, .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
}

static void add_input_endpoint(esp_zb_ep_list_t *ep_list)
{
    esp_zb_analog_output_cluster_cfg_t ao_cfg = {
        .out_of_service = false, .present_value = s_input_present_value, .status_flags = 0,
    };
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cl, create_basic_cluster(),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_attribute_list_t *ao_inp = esp_zb_analog_output_cluster_create(&ao_cfg);
    static float s_inp_min = 1.0f, s_inp_max = 6.0f;
    esp_zb_analog_output_cluster_add_attr(ao_inp,
        ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, &s_inp_min);
    esp_zb_analog_output_cluster_add_attr(ao_inp,
        ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, &s_inp_max);
    esp_zb_cluster_list_add_analog_output_cluster(cl, ao_inp,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    /* CLIENT role necessario perché report_attr_cmd_req con direction=TO_CLI
       non vada in assert (esp-idf#15962) */
    esp_zb_cluster_list_add_analog_output_cluster(cl,
        esp_zb_analog_output_cluster_create(&ao_cfg),
        ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_endpoint_config_t epcfg = {
        .endpoint = EP_INPUT, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = 0x000D, .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
}

/* Endpoint livello (posteriori / centrale / subwoofer): Analog Output 0-max,
 * stessa struttura dell'endpoint volume. */
static void add_level_endpoint(esp_zb_ep_list_t *ep_list, int i)
{
    esp_zb_analog_output_cluster_cfg_t ao_cfg = {
        .out_of_service = false, .present_value = s_level_present[i], .status_flags = 0,
    };
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cl, create_basic_cluster(),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_attribute_list_t *ao = esp_zb_analog_output_cluster_create(&ao_cfg);
    static float s_lvl_min = 0.0f;
    static float s_lvl_max[LEVEL_COUNT];
    s_lvl_max[i] = (float)s_levels[i].max;
    esp_zb_analog_output_cluster_add_attr(ao,
        ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, &s_lvl_min);
    esp_zb_analog_output_cluster_add_attr(ao,
        ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, &s_lvl_max[i]);
    esp_zb_cluster_list_add_analog_output_cluster(cl, ao,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    /* CLIENT role necessario perché report_attr_cmd_req con direction=TO_CLI
       non vada in assert (esp-idf#15962) */
    esp_zb_cluster_list_add_analog_output_cluster(cl,
        esp_zb_analog_output_cluster_create(&ao_cfg),
        ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_endpoint_config_t epcfg = {
        .endpoint = s_levels[i].ep, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = 0x000D, .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
}

/* Endpoint switch effetto 3D: On/Off server (+ client per il report esplicito) */
static void add_fx3d_endpoint(esp_zb_ep_list_t *ep_list)
{
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cl, create_basic_cluster(),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_on_off_cluster_cfg_t oo = { .on_off = s_fx3d_on };
    esp_zb_cluster_list_add_on_off_cluster(cl, esp_zb_on_off_cluster_create(&oo),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    /* CLIENT role: report_attr_cmd_req con direction=TO_CLI (esp-idf#15962) */
    esp_zb_cluster_list_add_on_off_cluster(cl, esp_zb_on_off_cluster_create(&oo),
        ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_endpoint_config_t epcfg = {
        .endpoint = EP_FX3D, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID, .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
}

static esp_err_t handle_on_off(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    if (msg->info.cluster != ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) return ESP_OK;
    if (msg->attribute.id != ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) return ESP_OK;
    if (msg->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_BOOL) return ESP_OK;

    bool state       = *(bool *)msg->attribute.data.value;

    if (msg->info.dst_endpoint == EP_FX3D) {
        s_zb_fx3d_pending = state ? 1 : 0;
        xSemaphoreGive(s_zb_cmd_sem);
        ESP_LOGI(TAG, "ZB→3D richiesto %s", state ? "ON" : "OFF");
        return ESP_OK;
    }
    if (msg->info.dst_endpoint != EP_POWER) return ESP_OK;

    s_zb_power       = state;
    s_zb_pwr_pending = state ? 1 : 0;
    xSemaphoreGive(s_zb_cmd_sem);
    ESP_LOGI(TAG, "ZB→power %s", state ? "ON" : "OFF");
    return ESP_OK;
}

static esp_err_t handle_analog_output(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    if (msg->info.cluster != ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT) return ESP_OK;
    if (msg->attribute.id != ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID) return ESP_OK;

    float   val = *(float *)msg->attribute.data.value;
    uint8_t ep  = msg->info.dst_endpoint;

    if (ep == EP_VOLUME) {
        int v = (int)roundf(val);
        if (v < 0) v = 0;
        if (v > 43) v = 43;
        s_vol_present_value = (float)v;
        s_zb_vol_pending    = (int8_t)v;
        xSemaphoreGive(s_zb_cmd_sem);
    } else if (ep == EP_INPUT) {
        int inp = (int)roundf(val) - 1;
        if (inp < 0 || inp > 5) return ESP_OK;
        s_input_present_value = (float)(inp + 1);
        s_zb_inp_pending      = (int8_t)inp;
        xSemaphoreGive(s_zb_cmd_sem);
    } else {
        for (int i = 0; i < LEVEL_COUNT; i++) {
            if (ep != s_levels[i].ep) continue;
            int v = (int)roundf(val);
            if (v < 0) v = 0;
            if (v > s_levels[i].max) v = s_levels[i].max;
            s_level_present[i]    = (float)v;
            s_zb_level_pending[i] = (int8_t)v;
            xSemaphoreGive(s_zb_cmd_sem);
            break;
        }
    }
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t cb_id,
                                    const void *msg)
{
    switch (cb_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: {
        const esp_zb_zcl_set_attr_value_message_t *m = msg;
        if (!m || m->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) return ESP_OK;
        if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
            return handle_on_off(m);
        if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT)
            return handle_analog_output(m);
        return ESP_OK;
    }
#if ZB_OTA_ENABLED
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        return ota_upgrade_handler(
            (const esp_zb_zcl_ota_upgrade_value_message_t *)msg);
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
#endif
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        return ESP_OK;
    default:
        ESP_LOGD(TAG, "ZB action non gestita: 0x%x", cb_id);
        return ESP_OK;
    }
}

static void bdb_start_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(
        esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK,
        , TAG, "Errore commissioning BDB");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *sig)
{
    uint32_t *p   = sig->p_app_signal;
    esp_err_t err = sig->esp_err_status;
    esp_zb_app_signal_type_t type = *p;

    switch (type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err == ESP_OK) {
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        zigbee_ready = false; s_coordinator_confirmed = false; s_reporting_ready = false;
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            zigbee_ready = true;
            ESP_LOGI(TAG, "ZB rete: ch=%d PAN=0x%04hx addr=0x%04hx",
                     esp_zb_get_current_channel(), esp_zb_get_pan_id(),
                     esp_zb_get_short_address());
            s_ping_tick = COORDINATOR_PING_TICKS - 1;
            /* 2.x: una nuova schedulazione dello stesso callback crea una seconda catena
             * (visto su ZB2): cancella prima quella eventualmente attiva */
            esp_zb_scheduler_alarm_cancel(coordinator_ping_cb, 0);
            esp_zb_scheduler_alarm(coordinator_ping_cb, 0, 1000);
        } else {
            ESP_LOGW(TAG, "ZB steering fallito, riprovo tra 30s");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        if (zigbee_ready) {
            zigbee_ready = false; s_coordinator_confirmed = false; s_reporting_ready = false;
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_NLME_STATUS_INDICATION:
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        break;
    default:
        ESP_LOGI(TAG, "ZB signal: %s (0x%x) %s",
                 esp_zb_zdo_signal_to_string(type), type, esp_err_to_name(err));
        break;
    }
}

static void zb_push_state(void)
{
    uint8_t on_val = s_amp_actual_on ? 0x01 : 0x00;   /* ZCL Boolean: 0x00/0x01 (0xFF = invalido) */
    esp_zb_zcl_set_attribute_val(EP_POWER,
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on_val, false);
    esp_zb_zcl_set_attribute_val(EP_VOLUME,
        ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &s_vol_present_value, false);
    esp_zb_zcl_set_attribute_val(EP_INPUT,
        ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &s_input_present_value, false);
    for (int i = 0; i < LEVEL_COUNT; i++)
        esp_zb_zcl_set_attribute_val(s_levels[i].ep,
            ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &s_level_present[i], false);
    uint8_t fx_val = s_fx3d_on ? 0x01 : 0x00;
    esp_zb_zcl_set_attribute_val(EP_FX3D,
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &fx_val, false);

    if (s_reporting_ready) {
        esp_zb_zcl_report_attr_cmd_t rep = {0};
        rep.address_mode                        = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        rep.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
        rep.zcl_basic_cmd.dst_endpoint          = 1;
        rep.clusterID    = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT;
        rep.attributeID  = ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID;
        rep.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        rep.zcl_basic_cmd.src_endpoint = EP_VOLUME;
        esp_zb_zcl_report_attr_cmd_req(&rep);
        rep.zcl_basic_cmd.src_endpoint = EP_INPUT;
        esp_zb_zcl_report_attr_cmd_req(&rep);
        for (int i = 0; i < LEVEL_COUNT; i++) {
            rep.zcl_basic_cmd.src_endpoint = s_levels[i].ep;
            esp_zb_zcl_report_attr_cmd_req(&rep);
        }

        /* Stato acceso/spento: report esplicito, così HA si allinea anche dopo un
         * riavvio della console (il valore locale non cambia → nessun report automatico) */
        rep.clusterID    = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        rep.attributeID  = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
        rep.zcl_basic_cmd.src_endpoint = EP_POWER;
        esp_zb_zcl_report_attr_cmd_req(&rep);
        rep.zcl_basic_cmd.src_endpoint = EP_FX3D;
        esp_zb_zcl_report_attr_cmd_req(&rep);
    }
}

static void reporting_ready_cb(uint8_t param)
{
    (void)param;
    s_reporting_ready = true;
    /* Il primo zb_push_state (alla conferma del coordinator) arriva prima di questo
     * momento e quindi non invia report: forza un nuovo invio entro 2 s. */
    s_zb_attr_dirty = true;
    ESP_LOGI(TAG, "ZB reporting+binding pronti");
}

static void setup_reporting(const esp_zb_ieee_addr_t coord_ieee)
{
    uint16_t own_addr = esp_zb_get_short_address();
    float reportable_change = 1.0f;

    esp_zb_zcl_config_report_record_t rec = {0};
    rec.direction         = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
    rec.attributeID       = ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID;
    rec.attrType          = ESP_ZB_ZCL_ATTR_TYPE_SINGLE;
    rec.min_interval      = 0;
    rec.max_interval      = 300;
    rec.reportable_change = &reportable_change;

    esp_zb_zcl_config_report_cmd_t cfg = {0};
    cfg.address_mode                    = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cfg.clusterID                       = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT;
    cfg.record_number                   = 1;
    cfg.record_field                    = &rec;
    cfg.zcl_basic_cmd.dst_addr_u.addr_short = own_addr;

    cfg.zcl_basic_cmd.src_endpoint = EP_VOLUME;
    cfg.zcl_basic_cmd.dst_endpoint = EP_VOLUME;
    esp_zb_zcl_config_report_cmd_req(&cfg);

    cfg.zcl_basic_cmd.src_endpoint = EP_INPUT;
    cfg.zcl_basic_cmd.dst_endpoint = EP_INPUT;
    esp_zb_zcl_config_report_cmd_req(&cfg);

    for (int i = 0; i < LEVEL_COUNT; i++) {
        cfg.zcl_basic_cmd.src_endpoint = s_levels[i].ep;
        cfg.zcl_basic_cmd.dst_endpoint = s_levels[i].ep;
        esp_zb_zcl_config_report_cmd_req(&cfg);
    }

    esp_zb_zdo_bind_req_param_t bind = {0};
    bind.cluster_id    = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT;
    bind.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    bind.dst_endp      = 1;
    bind.req_dst_addr  = own_addr;
    esp_zb_ieee_address_by_short(own_addr, bind.src_address);
    memcpy(bind.dst_address_u.addr_long, coord_ieee, sizeof(esp_zb_ieee_addr_t));

    bind.src_endp = EP_VOLUME;
    esp_zb_zdo_device_bind_req(&bind, NULL, NULL);

    bind.src_endp = EP_INPUT;
    esp_zb_zdo_device_bind_req(&bind, NULL, NULL);

    for (int i = 0; i < LEVEL_COUNT; i++) {
        bind.src_endp = s_levels[i].ep;
        esp_zb_zdo_device_bind_req(&bind, NULL, NULL);
    }

    esp_zb_scheduler_alarm(reporting_ready_cb, 0, 500);
    ESP_LOGI(TAG, "ZB reporting+binding inviati — attivo tra 500ms");
}

static void coordinator_ping_result(esp_zb_zdp_status_t zdo_status,
                                     esp_zb_zdo_ieee_addr_rsp_t *resp,
                                     void *user_ctx)
{
    s_ping_in_flight = false;
    if (!zigbee_ready) return;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        s_ping_tick = 0;
        if (!s_coordinator_confirmed) {
            s_coordinator_confirmed = true;
            ESP_LOGI(TAG, "ZB coordinator raggiungibile — operativo");
            setup_reporting(resp->ieee_addr);
#if ZB_OTA_ENABLED
            esp_zb_ota_upgrade_client_query_interval_set(EP_POWER,
                OTA_UPGRADE_QUERY_INTERVAL);
            esp_zb_ota_upgrade_client_query_image_req(0x0000, 1);
#endif
        }
        zb_push_state();
        esp_zb_scheduler_alarm(coordinator_ping_cb, 0, COORDINATOR_PING_TICK_MS);
    } else {
        ESP_LOGW(TAG, "ZB coordinator irraggiungibile (0x%02x)", zdo_status);
        zigbee_ready = false; s_coordinator_confirmed = false; s_reporting_ready = false; s_ping_tick = 0;
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
    }
}

static void coordinator_ping_timeout_cb(uint8_t param)
{
    (void)param;
    if (!s_ping_in_flight) return;
    coordinator_ping_result(ESP_ZB_ZDP_STATUS_TIMEOUT, NULL, NULL);
}

static void coordinator_ping_cb(uint8_t param)
{
    (void)param;
    if (!zigbee_ready) return;
    s_ping_tick++;
    if (s_ping_tick < COORDINATOR_PING_TICKS) {
        esp_zb_scheduler_alarm(coordinator_ping_cb, 0, COORDINATOR_PING_TICK_MS);
        return;
    }
    s_ping_tick = 0;
    esp_zb_zdo_ieee_addr_req_param_t req = {
        .dst_nwk_addr = 0x0000, .addr_of_interest = 0x0000,
        .request_type = 0, .start_index = 0,
    };
    s_ping_in_flight = true;
    esp_zb_zdo_ieee_addr_req(&req, coordinator_ping_result, NULL);
    esp_zb_scheduler_alarm(coordinator_ping_timeout_cb, 0, COORDINATOR_PING_TIMEOUT_MS);
}

static void zb_attr_sync_cb(uint8_t param)
{
    (void)param;
    if (s_zb_attr_dirty && zigbee_ready) {
        s_zb_attr_dirty = false;
        zb_push_state();
    }
    esp_zb_scheduler_alarm(zb_attr_sync_cb, 0, 2000);
}

static void esp_zb_task(void *pv)
{
    esp_zb_cfg_t cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    add_power_endpoint(ep_list);
    add_volume_endpoint(ep_list);
    add_input_endpoint(ep_list);
    for (int i = 0; i < LEVEL_COUNT; i++)
        add_level_endpoint(ep_list, i);
    add_fx3d_endpoint(ep_list);

    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    /* Il ping al coordinator parte solo dopo l'ingresso in rete (STEERING ok):
     * avviarlo anche qui creava due catene di allarmi in 2.x (visto su ZB2) */
    esp_zb_scheduler_alarm(zb_attr_sync_cb, 0, 2000);

    esp_zb_stack_main_loop();
}

/* ── Factory reset (GPIO9, hold 5s) ────────────────────────────────── */
#define FACTORY_RESET_GPIO   GPIO_NUM_9
#define FACTORY_RESET_HOLD_S 5

/* Migrazione una tantum dei dati Zigbee 1.x (zb_storage, ZBOSS) → 2.x (nvs, namespace
 * "zigbee"), come in ESP32-H2-Custom-Zigbee 5.5-ZB2 (provata: il modulo rientra in rete da
 * solo) e nell'esempio Espressif examples/nvram_migration. Dopo nvs_flash_init, prima
 * dell'avvio dello stack. Il marcatore "zb2_mig/done" evita di ripeterla; un solo
 * tentativo, riuscito o no, senza bloccare l'avvio. */
static void zb2_migrate_legacy_nvram(void)
{
    nvs_handle_t h;
    if (nvs_open("zb2_mig", NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "migrazione 1.x: nvs_open fallito, salto");
        return;
    }
    uint8_t done = 0;
    if (nvs_get_u8(h, "done", &done) == ESP_OK) {
        nvs_close(h);
        return;                                       /* già fatta */
    }
    esp_err_t err = esp_zigbee_nvram_migrate(NULL);   /* default: zb_storage → nvs, IEEE = MAC */
    ESP_LOGW(TAG, "migrazione dati Zigbee 1.x → 2.x: %s", esp_err_to_name(err));
    nvs_set_u8(h, "done", 1);
    nvs_commit(h);
    nvs_close(h);
}

static void factory_reset_task(void *pv)
{
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << FACTORY_RESET_GPIO),
        .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);
    vTaskDelay(pdMS_TO_TICKS(10000));
    for (;;) {
        if (gpio_get_level(FACTORY_RESET_GPIO) != 0) {
            vTaskDelay(pdMS_TO_TICKS(200)); continue;
        }
        ESP_LOGW(TAG, "GPIO9 premuto — tieni %ds per factory reset", FACTORY_RESET_HOLD_S);
        bool held = true;
        for (int i = 0; i < FACTORY_RESET_HOLD_S * 10; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (gpio_get_level(FACTORY_RESET_GPIO) != 0) { held = false; break; }
        }
        if (!held) { ESP_LOGI(TAG, "GPIO9 rilasciato — annullato"); continue; }
        ESP_LOGW(TAG, "*** FACTORY RESET ZIGBEE ***");
        /* esp-zigbee-lib 2.x salva le credenziali nella partizione "nvs", namespace
         * "zigbee" (verificato sul modulo ZB2): va cancellato quello. zb_storage/zb_fct
         * (dati 1.x) vengono cancellati come prima. */
        nvs_handle_t zh;
        if (nvs_open("zigbee", NVS_READWRITE, &zh) == ESP_OK) {
            nvs_erase_all(zh);
            nvs_commit(zh);
            nvs_close(zh);
        }
        const char *zb_parts[] = { "zb_storage", "zb_fct" };
        for (int i = 0; i < 2; i++) {
            const esp_partition_t *p = esp_partition_find_first(
                ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, zb_parts[i]);
            if (p) esp_partition_erase_range(p, 0, p->size);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
}

/* ════════════════════════════════════════════════════════════════════ */
/*  ENTRY POINT                                                        */
/* ════════════════════════════════════════════════════════════════════ */

static void led_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num   = GPIO_LED,
        .max_leds         = LED_NUM,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model        = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma    = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_led_strip));
    led_strip_clear(s_led_strip);
}

static void ir_init(void)
{
    rmt_rx_channel_config_t rx_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = IR_RMT_RES_HZ,
        .mem_block_symbols = IR_HW_MEM_SYMBOLS,
        .gpio_num          = GPIO_IR,
        .flags.invert_in   = false,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &s_ir_channel));

    s_ir_queue = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t ir_cbs = { .on_recv_done = ir_rx_done_cb };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(s_ir_channel, &ir_cbs, s_ir_queue));
    ESP_ERROR_CHECK(rmt_enable(s_ir_channel));
}

void app_main(void)
{
    esp_zb_platform_config_t zb_cfg = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Ingresso salvato: diventa lo stato iniziale (effetto all'accensione, valore HA) */
    nvs_load_input();
    if (s_saved_input < 6) {
        s_amp_status[7] = s_saved_input;
        recalc_status_checksum();
        s_input_present_value = (float)(s_saved_input + 1);
        ESP_LOGI(TAG, "NVS: ingresso salvato = %d", s_saved_input + 1);
    }
    /* Livelli salvati: diventano lo stato iniziale (valori HA fino al primo 0x34) */
    nvs_load_levels();
    for (int i = 0; i < LEVEL_COUNT; i++) {
        if (s_saved_level[i] > s_levels[i].max) continue;
        s_amp_status[s_levels[i].idx] = s_saved_level[i];
        ESP_LOGI(TAG, "NVS: %s salvato = %d", s_levels[i].name, s_saved_level[i]);
    }
    recalc_status_checksum();
    levels_from_status();   /* valori iniziali endpoint livelli, aggiornati al primo 0x34 */
    nvs_load_fx3d();
    fx3d_to_status();
    ESP_LOGI(TAG, "NVS: 3D ingresso 3 = %s", s_fx3d_on ? "ON" : "OFF");

    zb2_migrate_legacy_nvram();   /* prima dell'avvio dello stack Zigbee */
    ESP_ERROR_CHECK(esp_zb_platform_config(&zb_cfg));
    ota_mark_valid();

    /* TX amp idle HIGH prima di init UART */
    gpio_set_direction(GPIO_AMP_TX, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_AMP_TX, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Z906 Console v3 avviato");

    /* UART1 → amp (57600 8O1) */
    const uart_config_t uart_cfg = {
        .baud_rate  = Z906_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_ODD,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_AMP, BUF * 2, BUF * 2, 0, NULL, 0);
    uart_param_config(UART_AMP, &uart_cfg);
    uart_set_pin(UART_AMP, GPIO_AMP_TX, GPIO_AMP_RX, -1, -1);

    /* GPIO amp pin8 (input) */
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << GPIO_AMP8_IN),
        .mode = GPIO_MODE_INPUT, .pull_up_en = 0, .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_cfg);

    /* GPIO amp pin15 (output via Q1) */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << GPIO_AMP15_OUT),
        .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0, .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);
    gpio_set_level(GPIO_AMP15_OUT, 0);

    /* LED strip e IR */
    led_init();
    ir_init();

    s_zb_cmd_sem     = xSemaphoreCreateBinary();
    s_amp_tx_mutex   = xSemaphoreCreateRecursiveMutex();
    s_amp_frame_done = xSemaphoreCreateBinary();

    xTaskCreate(task_amp,          "amp",      4096, NULL, 5, NULL);
    xTaskCreate(task_amp_sync,     "sync",     2048, NULL, 3, NULL);
    xTaskCreate(task_gpio,         "gpio",     2048, NULL, 4, NULL);
    xTaskCreate(task_zb_cmd,       "zb_cmd",   3072, NULL, 2, NULL);
    xTaskCreate(task_ir,           "ir",       2048, NULL, 4, NULL);
    xTaskCreate(task_led,          "led",      2048, NULL, 1, NULL);
    xTaskCreate(factory_reset_task,"rst",      2048, NULL, 1, NULL);
    xTaskCreate(esp_zb_task,       "zb_main",  8192, NULL, 5, NULL);
}
