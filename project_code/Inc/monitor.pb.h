/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.3 at Thu Mar 14 14:37:50 2019. */

#ifndef PB_MONITOR_PB_H_INCLUDED
#define PB_MONITOR_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _Node {
    int32_t nodeId;
    int32_t address;
    int32_t channel;
/* @@protoc_insertion_point(struct:Node) */
} Node;

typedef struct _DCtrlReq {
    Node node;
    int32_t cmd;
/* @@protoc_insertion_point(struct:DCtrlReq) */
} DCtrlReq;

typedef struct _NCtrlRsp {
    Node node;
    int32_t errorcode;
/* @@protoc_insertion_point(struct:NCtrlRsp) */
} NCtrlRsp;

typedef struct _SensorValue {
    Node node;
    double value1;
/* @@protoc_insertion_point(struct:SensorValue) */
} SensorValue;

typedef struct _DeviceSend {
    int32_t sid;
    pb_size_t which_payload;
    union {
        DCtrlReq control;
    } payload;
/* @@protoc_insertion_point(struct:DeviceSend) */
} DeviceSend;

typedef struct _NodeSend {
    int32_t sid;
    pb_size_t which_payload;
    union {
        SensorValue sensorValue;
        NCtrlRsp control;
    } payload;
/* @@protoc_insertion_point(struct:NodeSend) */
} NodeSend;

/* Default values for struct fields */

/* Initializer values for message structs */
#define Node_init_default                        {0, 0, 0}
#define DCtrlReq_init_default                    {Node_init_default, 0}
#define SensorValue_init_default                 {Node_init_default, 0}
#define NCtrlRsp_init_default                    {Node_init_default, 0}
#define NodeSend_init_default                    {0, 0, {SensorValue_init_default}}
#define DeviceSend_init_default                  {0, 0, {DCtrlReq_init_default}}
#define Node_init_zero                           {0, 0, 0}
#define DCtrlReq_init_zero                       {Node_init_zero, 0}
#define SensorValue_init_zero                    {Node_init_zero, 0}
#define NCtrlRsp_init_zero                       {Node_init_zero, 0}
#define NodeSend_init_zero                       {0, 0, {SensorValue_init_zero}}
#define DeviceSend_init_zero                     {0, 0, {DCtrlReq_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define Node_nodeId_tag                          1
#define Node_address_tag                         2
#define Node_channel_tag                         3
#define DCtrlReq_node_tag                        1
#define DCtrlReq_cmd_tag                         2
#define NCtrlRsp_node_tag                        1
#define NCtrlRsp_errorcode_tag                   2
#define SensorValue_node_tag                     1
#define SensorValue_value1_tag                   2
#define DeviceSend_control_tag                   2
#define DeviceSend_sid_tag                       1
#define NodeSend_sensorValue_tag                 2
#define NodeSend_control_tag                     3
#define NodeSend_sid_tag                         1

/* Struct field encoding specification for nanopb */
extern const pb_field_t Node_fields[4];
extern const pb_field_t DCtrlReq_fields[3];
extern const pb_field_t SensorValue_fields[3];
extern const pb_field_t NCtrlRsp_fields[3];
extern const pb_field_t NodeSend_fields[4];
extern const pb_field_t DeviceSend_fields[3];

/* Maximum encoded size of messages (where known) */
#define Node_size                                33
#define DCtrlReq_size                            46
#define SensorValue_size                         44
#define NCtrlRsp_size                            46
#define NodeSend_size                            59
#define DeviceSend_size                          59

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define MONITOR_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
