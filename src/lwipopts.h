#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Generally you would define your own explicit list of lwIP options
// (see https://www.nongnu.org/lwip/2_1_x/group__lwip__opts.html)
//
// This example uses a common include to avoid repetition
#include "lwipopts_examples_common.h"

// FreeRTOS threading settings (required for NO_SYS=0)
#if !NO_SYS
#define TCPIP_THREAD_STACKSIZE 4096
#define DEFAULT_THREAD_STACKSIZE 1024
#define ASYNC_CONTEXT_DEFAULT_FREERTOS_TASK_STACK_SIZE 4096
#define DEFAULT_RAW_RECVMBOX_SIZE 8
#define TCPIP_MBOX_SIZE 8
#define LWIP_TIMEVAL_PRIVATE 0
#define TCPIP_THREAD_PRIO   7
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1
#define LWIP_SO_RCVTIMEO 1
#endif

// Increase memory for large web pages
#undef MEM_SIZE
#define MEM_SIZE                    16000

// Increase TCP send buffer for large HTML pages
#undef TCP_SND_BUF
#define TCP_SND_BUF                 (16 * TCP_MSS)

#undef TCP_WND
#define TCP_WND                     (8 * TCP_MSS)

// Increase TCP segments to match larger send buffer
#undef MEMP_NUM_TCP_SEG
#define MEMP_NUM_TCP_SEG            64

// TCP Keepalive - helps detect dead connections and keeps NAT/firewall alive
#define LWIP_TCP_KEEPALIVE          1

// Increase TCP retransmissions for stability
#define TCP_MAXRTX                  12
#define TCP_SYNMAXRTX               6

// Extra timeout slots for applications (MQTT, etc.)
#undef MEMP_NUM_SYS_TIMEOUT
#define MEMP_NUM_SYS_TIMEOUT        (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 1)

// HTTPD options (required for http_rest.c)
#define LWIP_HTTPD                  1
#define LWIP_HTTPD_CGI              1
#define LWIP_HTTPD_CGI_SSI          0
#define LWIP_HTTPD_SSI              0
#define LWIP_HTTPD_DYNAMIC_HEADERS  1
#define LWIP_HTTPD_CUSTOM_FILES     0
#define LWIP_HTTPD_SUPPORT_POST     0
#define LWIP_HTTPD_POST_MAX_PAYLOAD_LEN 2048
#define HTTPD_SERVER_PORT           80
#define LWIP_HTTPD_MAX_CGI_PARAMETERS 16
#define LWIP_HTTPD_MAX_REQUEST_URI_LEN 2048

#endif
