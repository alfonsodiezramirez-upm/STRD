#ifndef MODE_H
#define MODE_H
#include <FreeRTOS.h>
#include <stddef.h>
#include <semphr.h>

void MODE_init(void);
void MODE_set(int);
int MODE_get(void);

#endif /* MODE_H */