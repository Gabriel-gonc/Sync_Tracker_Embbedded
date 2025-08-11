#ifndef WIFI_ACESS_POINT_H
#define WIFI_ACESS_POINT_H
#include "project_types.h"

/*********************************************************
 * Funções
 *********************************************************/
void wifi_init_softap(void);

void wifi_get_callback(system_callback_t callback);

#endif // WIFI_ACESS_POINT_H