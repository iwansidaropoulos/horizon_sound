#ifndef SRC_HAIFA_COMMS_TASKS_H_
#define SRC_HAIFA_COMMS_TASKS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define HAIFA_TX_BUFFER_SIZE 64
#define HAIFA_DMA_BUFFER_SIZE 240
#define HAIFA_MESSAGE_LIST_SIZE 3
#define HAIFA_MESSAGE_SIZE 80
#define HAIFA_UART huart1

typedef struct haifa_payload_t
{
	uint8_t message_size;
	uint8_t message[HAIFA_TX_BUFFER_SIZE];
} haifa_payload_t;

void setupHaifaRtosObjects();

void haifaRxHandler(void *argument);
void haifaTxHandler(void *argument);

void haifaTxDone();

#ifdef __cplusplus
}
#endif

#endif /* SRC_USER_TASKS_H_ */
