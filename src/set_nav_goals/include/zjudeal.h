#ifndef ZJUDEAL_H
#define ZJUDEAL_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
typedef struct
{
	int statusChange;
	int status;
	int x;
	int y;
	int z;
	int r;
	std::string baseInfo;
}Struct_deal;

extern Struct_deal deal;
void initCmdDeal();
void setStatus(int status);
int getStatus();
void setPosition(int x,int y,int z,int r);
int getPosition(int *x,int *y,int *z,int *r);
void setBaseInfo(std::string info);
void getBaseInfo(std::string &info);

#endif // ZJUDEAL_H
