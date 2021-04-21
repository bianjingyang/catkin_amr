#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "zjucode.h"
#define	BUFSIZE	1024
static int bufindr = 0;
static char *strchr_pointer;
static char cmdbuffer[BUFSIZE];
#define	code_printf	//printf
static FILE *fp;
bool openCodeFile(char *filename)
{
    fp = fopen(filename, "r");
    if( fp == NULL )
    {
        code_printf("open file error ! \n");
        return false;
    }
    makeCodeList();
	intiActionPos();
    return true;
}

static bool code_seen(char code)
{
    strchr_pointer = strchr(&cmdbuffer[bufindr], code);
    return (strchr_pointer != NULL);
}

static unsigned int code_value(void)
{
	return (strtod(&cmdbuffer[strchr_pointer -cmdbuffer + 1], NULL));
}

bool readCodeFile(char *cmd ,unsigned int *sub, unsigned int *buf)
{
    *cmd = 0;
    *sub = 0;
    while( fgets(cmdbuffer, sizeof(cmdbuffer)-1, fp) != NULL ) 
    {
        //code_printf("line :%s", cmdbuffer);
    
	bufindr = 0;
	if( code_seen('#') == true )
	{
	    code_printf("cmd ##\n");
            *cmd = '#';
	    continue;
	}
	else if( code_seen('S') == true )
	{
	    code_printf("cmd param \n");
            *cmd = 'S';
            *sub = code_value();
	    return true;
	}
	else if( code_seen('G') == true )
	{
 	    int x,y,z;
	    code_printf("cmd go\n");
            *sub = code_value();
	    if( code_seen('X'))
            {
		x = code_value();
                code_printf("X:%d ",x);
            }
	    if( code_seen('Y'))
            {
		y = code_value();
                code_printf("Y:%d ",y);
            }
	    if( code_seen('Z'))
            {
		z = code_value();
                code_printf("Z:%d ",z);
            }
            code_printf("\n");
            *cmd = 'G';
            int *xyz = (int *)buf;
            xyz[0] = x;
            xyz[1] = y;
            xyz[2] = z;
	    return true;
	}
	else if( code_seen('T') == true )
	{
            if(code_value() == 0 )
	    {
	        code_printf("cmd f\n");
	        if( code_seen('P'))
                {
	            code_printf("P:%d \n",(int)code_value());
                    *cmd = 'T';
                    *sub = 0;
                    int *task = (int *)buf;
                    task[0] = code_value();
                }
	    }
	    else if(code_value() == 1 )
	    {
	        code_printf("cmd r\n");
	        if( code_seen('P'))
                {
	            code_printf("P:%d \n",(int)code_value());
                    *cmd = 'T';
                    *sub = 1;
                    int *task = (int *)buf;
                    task[0] = code_value();
                }
	    }
	    return true;
	}
	bufindr = (bufindr + 1)%BUFSIZE;
        return true;
    }
    return false;
}
struct_pos *dealPosHead = NULL;
struct_pos *dealPosIndex = NULL;
struct_task *dealTaskIndex = NULL;
void makeCodeList()
{
    bool ret ;
    char cmd = 0;
    unsigned int sub = 0;
    unsigned int buf[4];
    dealPosHead = (struct_pos *)NULL;
    while( readCodeFile(&cmd, &sub, buf) == true )
    {
        if( cmd == 'G' )
		{
	   		struct_pos *pPos = (struct_pos *)malloc (sizeof(struct_pos ));
            pPos->x = buf[0];
            pPos->y = buf[1];
            pPos->z = buf[2];
            pPos->en = 1;
            pPos->pos_type = sub;
            pPos->pTask = NULL;
            pPos->pNext = NULL;

			if( dealPosHead == NULL )
			{
				dealPosHead = pPos;
				dealPosIndex = dealPosHead;
			}
			else
			{
				dealPosIndex->pNext = pPos;    //当前链表元素后面增加一个元素
				dealPosIndex = pPos;                   //索引指向新增加的元素
			}
		}
		else{
		    code_printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa:%c \n",cmd);
	    	struct_task *pTask = (struct_task *)malloc (sizeof(struct_task ));
            pTask->cmd_type = cmd;
            pTask->cmd_sub = sub;
            pTask->param[0] = buf[0];
            pTask->param[1] = buf[1];
            pTask->param[2] = buf[2];
            pTask->param[3] = buf[3];
            pTask->pNext = NULL;

			if( dealPosIndex->pTask == NULL )
			{
				dealPosIndex->pTask = pTask;
				dealTaskIndex = dealPosIndex->pTask;
			}
			else
			{
				dealTaskIndex->pNext = pTask;
				dealTaskIndex = pTask;
			}
		}
    }
    dealPosIndex = dealPosHead;
    code_printf("\n\n\n");
    while( dealPosIndex != NULL )
    {
		code_printf("P%d    %d    %d\n",dealPosIndex->pos_type,dealPosIndex->x,dealPosIndex->y);
		dealTaskIndex = dealPosIndex->pTask;
		while( dealTaskIndex != NULL )
    	{
	    	code_printf("%c%d    %d\n",dealTaskIndex->cmd_type,dealTaskIndex->cmd_sub,dealTaskIndex->param[0]);
	    	dealTaskIndex = dealTaskIndex->pNext;
		}
		dealPosIndex = dealPosIndex->pNext;
   	}
}

struct_pos *dealActHead = NULL;
struct_pos *dealActIndex = NULL;
int intiActionPos()
{
	dealActHead = dealPosHead;
	dealActIndex = dealActHead;
	if( dealActHead != NULL )
	{
		while( 1 )
		{
			if( dealActHead->pos_type == 0 )
			{
				dealActIndex = dealActHead;
				dealTaskIndex = dealActIndex->pTask;
				return 1;
     		}
			else
			{
				if(dealActHead->pNext != NULL)
				{
					dealActHead = dealActHead->pNext;
				}
				else
				{
					dealActIndex = dealActHead;
					return 0;
				}
			}
		}
	}
}

int getActionPos(int *x, int *y, int *z)
{
	if(dealActIndex != NULL)
	{
		*x = dealActIndex->x;
		*y = dealActIndex->y;
		*z = dealActIndex->z;
		return 1;
	}
	return 0;
}

int nextActionPos()
{
	if(dealActHead == NULL)
	{
		return 0;
	}
	else
	{
		if(dealActIndex->pNext != NULL)
		{
			dealActIndex = dealActIndex->pNext;
			dealTaskIndex = dealActIndex->pTask;      ///////////////////////////////////add
			return 1;
		}
		else
		{
			dealActIndex = dealActHead;
			dealTaskIndex = dealActIndex->pTask;      ///////////////////////////////////add
			return 2;
		}
	}
}

int getActionTask(char *cmd_type, unsigned int *cmd_sub, unsigned int *param)
{
	if(dealTaskIndex != NULL)
	{
		*cmd_type = dealTaskIndex->cmd_type;
		*cmd_sub = dealTaskIndex->cmd_sub;
		param[0] = dealTaskIndex->param[0];
		param[1] = dealTaskIndex->param[1];
		param[2] = dealTaskIndex->param[2];
		param[3] = dealTaskIndex->param[3];
		return 1;
	}
	return 0;
}

int nextActionTask()
{
	if(dealTaskIndex == NULL)
	{
		return 0;
	}
	else
	{
		if(dealTaskIndex->pNext != NULL)
		{
			dealTaskIndex = dealTaskIndex->pNext;
			return 1;
		}
		else
		{
			dealTaskIndex = dealTaskIndex;
			dealTaskIndex = NULL;
			return 2;
		}
	}
}

