#include "mongoose.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>


//Reads WebSocket messages, writes them to a buffer and sends them into a queue

//FreeRTOS queue with WebSocket msgs!
extern QueueHandle_t ws_msg_queue;

void event_handler(struct mg_connection *c, int ev, void *ev_data) {
    //two char buffers of length 100
    static char buffer_a[100];
    static char buffer_b[100];

    //char pointer that points to buffer_a
    static char* current_buffer = buffer_a;

    //if the incoming message is a websocket message (if ev==MG_EV_WS_MSG)
    if (ev == MG_EV_WS_MSG) {
        //wm->data is the recieved data
        struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;

        //extract the message contents into a buffer
        size_t len = wm->data.len < sizeof(buffer_a) - 1 ? wm->data.len : sizeof(buffer_a) - 1;
        memcpy(current_buffer, wm->data.ptr, len);
        current_buffer[len] = '\0';

        //paste the address of the char pointer into a freertos message queue
        xQueueSend(ws_msg_queue, &current_buffer, 0);

        //swap the address of the pointer
        if (current_buffer == buffer_a) {
            current_buffer = buffer_b;
        }
        else {
            current_buffer = buffer_a;
        }
        

    }
}