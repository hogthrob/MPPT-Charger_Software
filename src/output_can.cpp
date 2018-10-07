/* LibreSolar Battery Management System firmware
 * Copyright (c) 2016-2018 Martin Jäger (www.libre.solar)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"
#include "config.h"
#include "output.h"
#include "output_can.h"
#include "can_msg_queue.h"
//#include "can_iso_tp.h"

#include "data_objects.h"
#include "dcdc.h"
#include "charger.h"

extern Serial serial;
extern CAN can;


#define PUB_MULTIFRAME_EN (0x1U << 7)
#define PUB_TIMESTAMP_EN (0x1U << 6)

// Protocol functions
#define TS_READ     0x00
#define TS_WRITE    0x01
#define TS_PUB_REQ  0x02
#define TS_SUB_REQ  0x03
#define TS_OBJ_NAME 0x04
#define TS_LIST     0x05


static uint8_t can_node_id = CAN_NODE_ID;

void can_list_object_ids(int category = TS_C_ALL);
void can_send_object_name(int obj_id, uint8_t can_dest_id);


CANMsgQueue can_tx_queue;
CANMsgQueue can_rx_queue;

//----------------------------------------------------------------------------
// preliminary simple CAN functions to send data to the bus for logging
// Data format based on CBOR specification (except for first byte, which uses
// only 6 bit to specify type and transport protocol)
//
// Protocol details:
// https://github.com/LibreSolar/ThingSet/blob/master/can.md

void can_pub_msg(DataObject_t data_obj)
{
    union float2bytes { float f; char b[4]; } f2b;     // for conversion of float to single bytes

    int msg_priority = 6;

    CANMessage msg;
    msg.format = CANExtended;
    msg.type = CANData;

    msg.id = msg_priority << 26
        | (1U << 24) | (1U << 25)   // identify as publication message
        | data_obj.id << 8
        | can_node_id;

    // first byte: TinyTP-header or data type for single frame message
    // currently: no multi-frame and no timestamp
    msg.data[0] = 0x00;

    int32_t value;
    uint32_t value_abs;
    uint8_t exponent_abs;

    // CBOR byte order: most significant byte first
    switch (data_obj.type) {
        case T_FLOAT32:
            msg.len = 5;
            msg.data[0] |= TS_T_FLOAT32;
            f2b.f = *((float*)data_obj.data);
            msg.data[1] = f2b.b[3];
            msg.data[2] = f2b.b[2];
            msg.data[3] = f2b.b[1];
            msg.data[4] = f2b.b[0];
            break;
        case T_INT32:
            if (data_obj.exponent == 0) {
                    msg.len = 5;
                    value = *((int*)data_obj.data);
                    value_abs = abs(value);
                    if (value >= 0) {
                        msg.data[0] |= TS_T_POS_INT32;
                        value_abs = abs(value);
                    }
                    else {
                        msg.data[0] |= TS_T_NEG_INT32;
                        value_abs = abs(value - 1);         // 0 is always pos integer
                    }
                    msg.data[1] = value_abs >> 24;
                    msg.data[2] = value_abs >> 16;
                    msg.data[3] = value_abs >> 8;
                    msg.data[4] = value_abs;
            }
            else {
                msg.len = 8;
                msg.data[0] |= TS_T_DECIMAL_FRAC;
                msg.data[1] = 0x82;     // array of length 2

                // exponent in single byte value, valid: -24 ... 23
                exponent_abs = abs(data_obj.exponent);
                if (data_obj.exponent >= 0 && data_obj.exponent <= 23) {
                    msg.data[2] = exponent_abs;
                }
                else if (data_obj.exponent < 0 && data_obj.exponent > -24) {
                    msg.data[2] = (exponent_abs - 1) | 0x20;      // negative uint8
                }

                // value as positive or negative uint32
                value = *((int*)data_obj.data);
                value_abs = abs(value);
                if (value >= 0) {
                    msg.data[3] = 0x1a;     // positive int32
                    value_abs = abs(value);
                }
                else {
                    msg.data[3] = 0x3a;     // negative int32
                    value_abs = abs(value - 1);         // 0 is always pos integer
                }
                msg.data[4] = value_abs >> 24;
                msg.data[5] = value_abs >> 16;
                msg.data[6] = value_abs >> 8;
                msg.data[7] = value_abs;
            }
            break;
        case T_BOOL:
            msg.len = 1;
            if (*((bool*)data_obj.data) == true) {
                msg.data[0] = TS_T_TRUE;     // simple type: true
            }
            else {
                msg.data[0] = TS_T_FALSE;     // simple type: false
            }
            break;
        case T_STRING:
            break;
    }
    can_tx_queue.enqueue(msg);
}

void can_send_data()
{
    for (unsigned int i = 0; i < dataObjectsCount; ++i) {
        if (dataObjects[i].category == TS_C_OUTPUT) {
            can_pub_msg(dataObjects[i]);
        }
    }
}

void can_process_outbox()
{
    int max_attempts = 15;
    while (!can_tx_queue.empty() && max_attempts > 0) {
        CANMessage msg;
        can_tx_queue.first(msg);
        if(can.write(msg)) {
            can_tx_queue.dequeue();
        }
        else {
            //serial.printf("Sending CAN message failed, err: %u, MCR: %u, MSR: %u, TSR: %u, id: %u\n", can.tderror(), (uint32_t)CAN1->MCR, (uint32_t)CAN1->MSR, (uint32_t)CAN1->TSR, msg.id);
        }
        max_attempts--;
    }
}

void can_list_object_ids(int category) {

}

/**
 * This function assumes it gets up to 5 bytes of data, starting with the ThingSet type id 
 * followed by the actual data if any
 * @param data_obj reference to a valid data_obj to which the value is saved to.
 * @param bytes  the at 1 or 5 bytes long data structure of to be decoded ThingSet data bytes.
 * @return true if the data object was actually written, otherwise a type mismatch or unsupported type was used
 */
static bool can_set_object_value(const DataObject_t& data_obj, uint8_t* bytes)
{
    union float2bytes { float f; char b[4]; } f2b;     // for conversion of float to single bytes
    // int32_t value;
    uint32_t value_abs;
    // uint8_t exponent_abs;
    bool retval = false;
    uint8_t tsDataType = bytes[0];

    // CBOR byte order: most significant byte first
    // after a successful decode set retval to true.
    switch (data_obj.type) {
        case T_FLOAT32:
            if (tsDataType == TS_T_FLOAT32)
            {
                f2b.b[3] = bytes[1];
                f2b.b[2] = bytes[2];
                f2b.b[1] = bytes[3];
                f2b.b[0] = bytes[4];
                *((float*)data_obj.data) = f2b.f;
                retval = true;
            }
            break;

        case T_INT32:
            if (bytes[0] == TS_T_POS_INT32 || bytes[0] == TS_T_NEG_INT32) {
                value_abs = 0;
                value_abs |= bytes[1] << 24;
                value_abs |= bytes[2] << 16;
                value_abs |= bytes[3] << 8;
                value_abs |= bytes[4];

                *((int*)data_obj.data) = bytes[0] == TS_T_POS_INT32 ? value_abs : -value_abs;
                retval = true;
            }
            // we don't support TS_T_DECIMAL_FRAC as it exceeds the available 5 bytes value bytes limit of a single message 
            break;
        case T_BOOL:
            if (tsDataType == TS_T_TRUE || tsDataType == TS_T_FALSE)
            {
                *((bool*)data_obj.data) = tsDataType == TS_T_TRUE;     // simple type: true or false
                retval = true;
            }
            break;
        case T_STRING:
            break;
    }
    return retval;
}

static const DataObject_t* can_find_object(int data_obj_id) {
    const DataObject_t* retval = NULL;
    for (unsigned int idx = 0; idx < dataObjectsCount; idx++) {
        if (dataObjects[idx].id == data_obj_id) {
            retval = &dataObjects[idx];
            break;  // correct array entry found
        }
    }
    return retval;
}

void can_send_object_name(int data_obj_id, uint8_t can_dest_id) {
    uint8_t msg_priority = 7;   // low priority service message
    uint8_t function_id = 0x84;
    CANMessage msg;
    msg.format = CANExtended;
    msg.type = CANData;
    msg.id = msg_priority << 26 | function_id << 16 |(can_dest_id << 8)| can_node_id;      // TODO: add destination node ID

    const DataObject_t* optr = can_find_object(data_obj_id);

    // optr != NULL -> we found an object with that id
    if (optr != NULL) {
        if (optr->access & ACCESS_READ) {
            msg.data[2] = T_STRING;
            int len = strlen(optr->name);
            for (int i = 0; i < len && i < (8-3); i++) {
                msg.data[i+3] = *(optr->name + i);
            }
            msg.len = ((len < 5) ? 3 + len : 8);
            serial.printf("TS Send Object Name: %s (id = %d)\n", optr->name, data_obj_id);
        }
    }
    else {
        // send error message
        // data[0] : ISO-TP header
        msg.data[1] = 1;    // TODO: Define error code numbers
        msg.len = 2;
    }

    can_tx_queue.enqueue(msg);
}

void can_process_inbox() {
    int max_attempts = 15;
    while (!can_rx_queue.empty() && max_attempts >0) {
        CANMessage msg;
        can_rx_queue.dequeue(msg);

        if (!(msg.id & (0x1U<<25))) {
            serial.printf("CAN ID bit 25 = 1 --> ignored\n");
            continue;  // might be SAE J1939 or NMEA 2000 message --> ignore
        }

        if (msg.id & (0x1U<<24)) {
            serial.printf("Data object publication frame\n");
            // data object publication frame
        } else {
            serial.printf("Service frame\n");
            // service frame
            uint8_t can_target_id = (msg.id >> 8) & (int)0xFF;

            if (can_target_id == can_node_id || can_target_id == 0xff) {
                int function_id = (msg.id >> 16) & (int)0xFF;
                uint8_t can_dest_id = msg.id & (int)0xFF;
                switch (function_id) {
                    case TS_WRITE:
                    {
                        int data_obj_id = msg.data[1] + (msg.data[2] << 8);
                        const DataObject_t* optr = can_find_object(data_obj_id);

                        if (optr != NULL) {
                            if (optr->access & ACCESS_WRITE) {
                                serial.printf("ThingSet Write to object with id = %d)\n", optr->name, data_obj_id);
                                if (can_set_object_value(*optr, &msg.data[3]) == false)
                                {
                                    // TODO: operation failed on object notification?    
                                }
                            } else {
                                serial.printf("No write allowed to data object %s (id = %d)\n", optr->name, data_obj_id);
                                // TODO: access denied for operation on object notification?

                            }
                        } else {
                            // TODO: requested object does not exist notification?
                        }
                        break;
                    }
                    case TS_READ:
                    {
                        int data_obj_id = msg.data[1] + (msg.data[2] << 8);
                        const DataObject_t* optr = can_find_object(data_obj_id);

                        if (optr != NULL) {
                            if (optr->access & ACCESS_READ) {
                                serial.printf("ThingSet read to object with id = %d)\n", optr->name, data_obj_id);
                                can_pub_msg(*optr); // TODO: should we use something else than a pub msg as reply (e.g. use the dest id) ? 
                            } else {
                                serial.printf("No read allowed to data object %s (id = %d)\n", optr->name, data_obj_id);
                                // TODO: access denied for operation on object notification?
                            }
                        } else {
                            // TODO: requested object does not exist notification?
                        }
                        break;
                    }

                    case TS_OBJ_NAME:
                    {
                        int data_obj_id = msg.data[1] + (msg.data[2] << 8);
                        can_send_object_name(data_obj_id, can_dest_id);
                        serial.printf("Get Data Object Name: %d\n", data_obj_id);
                        break;
                    }
                    case TS_LIST:
                        can_list_object_ids(msg.data[1]);
                        serial.printf("List Data Object IDs: %d\n", msg.data[1]);
                        break;
                }
            }
        }

        max_attempts--;
    }
}

void can_receive() {
    CANMessage msg;
    while (can.read(msg)) {
        if (!can_rx_queue.full()) {
            can_rx_queue.enqueue(msg);
            serial.printf("Message received. id: %d, data: %d\n", msg.id, msg.data[0]);
        } else {
            serial.printf("CAN rx queue full\n");
        }
    }
}
