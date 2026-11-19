#ifndef HOST_MQTT_CLIENT_H
#define HOST_MQTT_CLIENT_H
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
typedef void *esp_mqtt_client_handle_t;
typedef const char *esp_event_base_t;
typedef enum { MQTT_EVENT_CONNECTED, MQTT_EVENT_DISCONNECTED, MQTT_EVENT_DATA, MQTT_EVENT_ERROR } esp_mqtt_event_id_t;
typedef struct {
    int current_data_offset, total_data_len, data_len, topic_len;
    bool retain;
    char *data, *topic;
} esp_mqtt_event_t;
typedef esp_mqtt_event_t *esp_mqtt_event_handle_t;
typedef struct {
    struct { struct { const char *uri; } address; } broker;
    struct { uint64_t limit; } outbox;
} esp_mqtt_client_config_t;
#define ESP_EVENT_ANY_ID -1
#define ESP_ERROR_CHECK(value) assert((value) == 0)
int esp_mqtt_client_enqueue(esp_mqtt_client_handle_t, const char *, const char *, int, int, int, bool);
int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t, const char *, int);
esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *);
int esp_mqtt_client_register_event(esp_mqtt_client_handle_t, int, void (*)(void *, esp_event_base_t, int32_t, void *), void *);
int esp_mqtt_client_start(esp_mqtt_client_handle_t);
#endif
