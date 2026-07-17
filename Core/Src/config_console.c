#include "config_console.h"
#include "vehicle_config.h"
#include "at24c256.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONSOLE_LINE_SIZE 96U
#define CONSOLE_OUTPUT_SIZE 768U

static UART_HandleTypeDef *console_uart;
static volatile bool line_ready;
static char rx_line[CONSOLE_LINE_SIZE];
static volatile uint16_t rx_length;
static char command_line[CONSOLE_LINE_SIZE];
static char display_title[24] = "CONFIG READY";
static char display_line[24] = "type help";
static bool display_error;

static void Send(const char *text)
{
    HAL_UART_Transmit(console_uart, (uint8_t *)text, (uint16_t)strlen(text), 500U);
}

static void Show(const char *title, const char *line, bool error)
{
    snprintf(display_title, sizeof(display_title), "%s", title);
    snprintf(display_line, sizeof(display_line), "%s", line);
    display_error = error;
}

static void ReplyError(const char *message)
{
    char output[140];
    snprintf(output, sizeof(output), "ERROR: %s\r\n> ", message);
    Send(output);
    Show("CONFIG ERROR", message, true);
}

static void ReplyStorageError(ConfigStatus_t status)
{
    char output[180];
    snprintf(output, sizeof(output),
             "ERROR: %s stage=%s hal=%u i2c=0x%08lX state=0x%02X\r\n> ",
             VehicleConfig_StatusText(status), AT24C256_GetLastStage(),
             (unsigned)AT24C256_GetLastHALStatus(),
             (unsigned long)AT24C256_GetLastI2CError(),
             (unsigned)AT24C256_GetI2CState());
    Send(output);
    Show("EEPROM ERROR", AT24C256_GetLastStage(), true);
}

static void ProcessCommand(char *line)
{
    char *command = strtok(line, " \t");
    if (command == NULL) { Send("> "); return; }
    for (char *p = command; *p; p++) *p = (char)tolower((unsigned char)*p);

    if (strcmp(command, "help") == 0) {
        Send("help|get|set <key> <value>|save|load|defaults\r\n> ");
        Show("CONFIG HELP", "see UART", false);
    } else if (strcmp(command, "get") == 0) {
        static char output[CONSOLE_OUTPUT_SIZE];
        VehicleConfig_Format(output, sizeof(output));
        Send(output); Send("> ");
        Show("CONFIG VALUES", "sent to UART", false);
    } else if (strcmp(command, "set") == 0) {
        char *key = strtok(NULL, " \t");
        char *value_text = strtok(NULL, " \t");
        char *end;
        char error[64];
        if (key == NULL || value_text == NULL) {
            ReplyError("usage: set <key> <value>"); return;
        }
        long value = strtol(value_text, &end, 10);
        if (*value_text == '\0' || *end != '\0') {
            ReplyError("value must be integer"); return;
        }
        for (char *p = key; *p; p++) *p = (char)tolower((unsigned char)*p);
        if (!VehicleConfig_SetValue(key, (int32_t)value, error, sizeof(error))) {
            ReplyError(error); return;
        }
        char output[120];
        snprintf(output, sizeof(output), "OK: %s=%ld (RAM only; use save)\r\n> ", key, value);
        Send(output);
        snprintf(output, sizeof(output), "%s=%ld", key, value);
        Show("CONFIG CHANGED", output, false);
    } else if (strcmp(command, "save") == 0) {
        ConfigStatus_t status = VehicleConfig_Save();
        if (status != CONFIG_STATUS_OK) ReplyStorageError(status);
        else { Send("OK: saved and verified\r\n> "); Show("CONFIG SAVED", "EEPROM verified", false); }
    } else if (strcmp(command, "load") == 0) {
        ConfigStatus_t status = VehicleConfig_Load();
        if (status != CONFIG_STATUS_OK) ReplyStorageError(status);
        else { Send("OK: loaded\r\n> "); Show("CONFIG LOADED", "EEPROM -> RAM", false); }
    } else if (strcmp(command, "defaults") == 0) {
        VehicleConfig_SetDefaults();
        Send("OK: defaults loaded to RAM; use save\r\n> ");
        Show("DEFAULTS LOADED", "use save", false);
    } else {
        ReplyError("unknown command");
    }
}

void ConfigConsole_Init(UART_HandleTypeDef *uart)
{
    console_uart = uart;
    rx_length = 0;
    line_ready = false;
    Send("Commands: help, get, set <key> <value>, save, load, defaults\r\n> ");
}

static void ReceiveByte(uint8_t byte)
{
    if (!line_ready) {
        if (byte == '\r' || byte == '\n') {
            if (rx_length > 0U) {
                rx_line[rx_length] = '\0';
                line_ready = true;
            }
        } else if ((byte == '\b' || byte == 0x7FU) && rx_length > 0U) {
            rx_length--;
        } else if (byte >= 32U && byte <= 126U && rx_length < CONSOLE_LINE_SIZE - 1U) {
            rx_line[rx_length++] = (char)byte;
        }
    }
}

void ConfigConsole_Update(void)
{
    uint8_t byte;
    while (__HAL_UART_GET_FLAG(console_uart, UART_FLAG_RXNE) != RESET) {
        if (HAL_UART_Receive(console_uart, &byte, 1U, 0U) == HAL_OK) ReceiveByte(byte);
        else break;
    }
    if (!line_ready) return;
    __disable_irq();
    snprintf(command_line, sizeof(command_line), "%s", rx_line);
    rx_length = 0;
    line_ready = false;
    __enable_irq();
    ProcessCommand(command_line);
}

const char *ConfigConsole_GetDisplayTitle(void) { return display_title; }
const char *ConfigConsole_GetDisplayLine(void) { return display_line; }
bool ConfigConsole_HasError(void) { return display_error; }
