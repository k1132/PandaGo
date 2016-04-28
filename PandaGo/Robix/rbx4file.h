#ifndef __MODULE_RBX4FILE_H___
#define __MODULE_RBX4FILE_H___

#include "rbx4head.h"
#include "rbx4err.h"
#include "rbx4list.h"
#include "rbx4alc.h"
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

struct cfg_info
{
	char name [DATA_NAME_SIZE];
	char value[DATA_NAME_SIZE];
};

struct cfg_file
{
	int				 fd;
	int				 num;
	struct cfg_info *info;
	struct list_head next;
};

int readCfgFile(char *filename);
char *getCfg(int fd, char *name);
int closeCfgFile(int fd);

#ifdef __cplusplus
}
#endif

#endif
