#ifndef ZJUCODE_H
#define ZJUCODE_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

bool openCodeFile(char *filename);
void makeCodeList();
bool readCodeFile(char *cmd ,unsigned int *sub, unsigned int *buf);
int intiActionPos();
int getActionPos(int *x, int *y, int *z);
int nextActionPos();
int getActionTask(char *cmd_type, unsigned int *cmd_sub, unsigned int *param);
int nextActionTask();

typedef struct str_task
{
	char cmd_type;
	unsigned int cmd_sub;
	unsigned int param[4];
	str_task *pNext;
}struct_task;
typedef struct str_pos
{
	int x;
	int y;
	int z;
	unsigned int en;
	unsigned int pos_type;
	struct_task *pTask;
	str_pos *pNext;
}struct_pos;


#endif // ZJUCODE_H
