#ifndef __PC_TO_PLY_H__
#define __PC_TO_PLY_H__
#include <stdint.h>
#include "../../devices/muxin/mx_types.h"
#ifdef __cplusplus
extern "C" {
#endif

int32_t write_point_cloud_to_ply(char *file_name, int32_t num_points, point3f_t *pointers);

#ifdef __cplusplus
}
#endif
#endif
