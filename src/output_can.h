#ifndef OUTPUT_CAN_H
#define OUTPUT_CAN_H

#include "mbed.h"
#include "config.h"
#include "data_objects.h"
#include "can_msg_queue.h"



void can_send_data();
void can_receive();
void can_process_outbox();
void can_process_inbox();

#endif
