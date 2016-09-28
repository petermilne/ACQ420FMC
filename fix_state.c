/* 
 * reset state to 0
 * sed -ie 's/^[0-9]/0/' /dev/shm/state   breaks inotifywait
 * so we do it the unobtrusive way.
 * NB: STATE MUST be a single char at [0]
 */
#include <stdio.h>
#include <stdlib.h>

#define SF	"/dev/shm/state"


int main(int argc, char* argv[])
{
	FILE *fp = fopen(SF, "r+");
	char data[128];
	if (!fp){
		perror(SF);
		return 1;
	}
	fgets(data, 128, fp);
	data[0] = '0';
	rewind(fp);
	fputs(data, fp);
	fclose(fp);
	return 0;
}
