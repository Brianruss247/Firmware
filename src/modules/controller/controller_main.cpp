#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
 
extern "C" __EXPORT int controller_main(int argc, char *argv[]);
 
int controller_main(int argc, char *argv[])
{
	printf("Hello Sky!\n");
	return OK;
}
