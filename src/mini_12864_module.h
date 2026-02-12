#ifndef MINI_12864_MODULE_H_
#define MINI_12864_MODULE_H_


#define EEPROM_MINI_12864_MODULE_DATA_REV           2

#include "http_rest.h"
#include "encoder.h"  // For ButtonEncoderEvent_t and encoder functions


typedef struct {
    uint32_t data_rev;
    // Legacy fields - now use display_config for these settings
} mini_12864_module_config_t;


#ifdef __cplusplus
extern "C" {
#endif

bool mini_12864_module_init(void);
void display_init(void);
bool http_rest_mini_12864_module_config(struct fs_file *file, int num_params, char *params[], char *values[]);
bool mini_12864_module_config_save(void);

#ifdef __cplusplus
}
#endif



#endif  // MINI_12864_MODULE_H_
