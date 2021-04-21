#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "zjucode.h"
#include "zjucmd.h"
#include "zjudeal.h"


#define	BUFSIZE	1024
#define cmd_printf 
static int bufindr = 0;
static char *strchr_pointer;
static char *cmdbuffer;
static char cmdret[1024];
static bool code_seen(char code)
{
    strchr_pointer = strchr(&cmdbuffer[bufindr], code);
    return (strchr_pointer != NULL);
}
static unsigned int code_value(void)
{
	return (strtod(&cmdbuffer[strchr_pointer -cmdbuffer + 1], NULL));
}

#define	make_commands_ext sprintf
#define	__CMD__ cmdret


char *process_commands_ext(char * cmd)
{
	cmd_printf("CMD:%s\n",cmd);
	cmdbuffer = cmd;
	if( code_seen('#') == true )
	{
	    cmd_printf("net cmd ##\n");
	    make_commands_ext(__CMD__,"##");
	}
	else if( code_seen('L') == true )
	{
	    cmd_printf("net login \n");
	    if( code_seen('P') )
	    {
	        cmd_printf("net login pw: %d\n",code_value());
	        make_commands_ext(__CMD__,"L1");
        }
	}
	else if( code_seen('D') == true )
	{
	   	cmd_printf("net set run/stop\n");
		if( code_value() == 1 )
		{
		    cmd_printf("net set run/stop: RUN\n");
		    setStatus(1);
	        make_commands_ext(__CMD__,"D1");
		}
		else if( code_value() == 0 )
		{
			int x,y,z,r;
			std::string baseInfo;
		    cmd_printf("net set run/stop: CHECK\n");
			getPosition(&x,&y,&z,&r);
			getBaseInfo(baseInfo);
	     	make_commands_ext(__CMD__,"D%d X%d Y%d Z%d %s",getStatus(),x,y,z,baseInfo.c_str());//if(status == 1){x+=100;y+=1;}
		}
		else
		{
		    cmd_printf("net set run/stop: STOP\n");
		    setStatus(2);
			make_commands_ext(__CMD__,"D2");
		}
	}
	else if( code_seen('T') == true )
	{
	    if( code_value() == 1 )
		{
	        make_commands_ext(__CMD__,"T1");
		}
		else if( code_value() == 2 )
		{
			make_commands_ext(__CMD__,"T2");
		}
	    else 
	    {
	        if( code_seen('P') )
			{
				make_commands_ext(__CMD__,"T0");
			}
        }
	}
	else if( code_seen('M') == true )
	{
	    if( code_value() == 0 )
		{
	        make_commands_ext(__CMD__,"M0");
		}
	}
	else
	{

	}
	cmd_printf("RET:%s\n",__CMD__);
	return __CMD__;
}

