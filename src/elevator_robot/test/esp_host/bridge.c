/* Build the production MQTT parser/serializer and core with only the transport stubbed. */
#include "mqtt/mqtt_handler.c"
static elevator_sync_t simulation;
static void (*host_output)(const char *);
static uint64_t host_now;
static sync_emit_fn host_callback;
int esp_mqtt_client_enqueue(esp_mqtt_client_handle_t c, const char *topic, const char *payload,
                           int len, int qos, int retain, bool store)
{
    (void)c; (void)topic; (void)len; (void)qos; (void)retain; (void)store;
    assert(qos != 0 || store); /* Real ESP-MQTT rejects QoS0 enqueue without store. */
    host_output(payload);
    return 1;
}
int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t c, const char *topic, int qos)
{ (void)c; (void)topic; (void)qos; return 1; }
esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config)
{ (void)config; return (void *)1; }
int esp_mqtt_client_register_event(esp_mqtt_client_handle_t c, int id,
    void (*callback)(void *, esp_event_base_t, int32_t, void *), void *args)
{ (void)c; (void)id; (void)callback; (void)args; return 0; }
int esp_mqtt_client_start(esp_mqtt_client_handle_t c) { (void)c; return 0; }
void hardware_set_status_callback(sync_emit_fn callback) { host_callback = callback; }
bool hardware_enqueue_request(const sync_request_t *r)
{
    elevator_sync_request(&simulation, r, host_now);
    return true;
}
void host_init(void (*output)(const char *))
{
    host_output = output;
    host_now = 1000;
    mqtt_handler_start();
    elevator_sync_init(&simulation, 1, "hostboot", host_callback);
    mqtt_event_handler(NULL, NULL, MQTT_EVENT_CONNECTED, NULL);
    elevator_sync_report(&simulation, host_now);
}
void host_tick(unsigned milliseconds)
{
    host_now += milliseconds;
    elevator_sync_tick(&simulation, host_now);
    elevator_sync_report(&simulation, host_now);
}
void host_input(const char *payload, int length, int offset, int total, int retained)
{
    esp_mqtt_event_t event = {
        .current_data_offset = offset, .total_data_len = total, .data_len = length,
        .data = (char *)payload, .topic = REQUEST_TOPIC, .topic_len = sizeof(REQUEST_TOPIC)-1,
        .retain = retained,
    };
    mqtt_event_handler(NULL, NULL, MQTT_EVENT_DATA, &event);
}

void host_connection(int connected)
{
    mqtt_event_handler(NULL, NULL, connected ? MQTT_EVENT_CONNECTED : MQTT_EVENT_DISCONNECTED, NULL);
}
