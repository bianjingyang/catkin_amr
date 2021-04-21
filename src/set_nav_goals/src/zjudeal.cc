#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "zjudeal.h"

Struct_deal deal;

void initCmdDeal()
{
	deal.statusChange = 0;
	deal.status = 0;
	deal.baseInfo = "";
}

void setStatus(int status)
{
	deal.statusChange = 1;
	deal.status = status;
}

int getStatus()
{
	return deal.status;
}


void setPosition(int x,int y,int z,int r)
{
	deal.x = x;
	deal.y = y;
	deal.z = z;
	deal.r = r;
}

int getPosition(int *x,int *y,int *z,int *r)
{
	*x = deal.x;
	*y = deal.y;
	*z = deal.z ;
	*r = deal.r;
	return 1;
}

void setBaseInfo(std::string info)
{
	deal.baseInfo = info;
}

void getBaseInfo(std::string &info)
{
	info = deal.baseInfo;
}
