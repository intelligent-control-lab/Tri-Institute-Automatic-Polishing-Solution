#ifndef SRI_COMM_MANAGER_H
#define SRI_COMM_MANAGER_H


#include "sriCommDefine.h"
#include "sriCommTCPClient.h"
#include "sriCommATParser.h"
#include "sriCommM8218Parser.h"

class CSRICommManager
{
public:
	CSRICommManager();
	~CSRICommManager();

	bool Init();
	bool Run();
	bool Stop();

	bool SendCommand(std::string command, std::string parames);
	
	bool OnNetworkFailure(std::string infor);//通讯失败
	bool OnCommACK(std::string command);//ACK应答数据处理
	bool OnCommM8218(float fx, float fy, float fz, float mx, float my, float mz);//GSD数据处理
private:
	CSRICommTCPClient mTCPClient;
	CSRICommATParser mATParser;//AT指令解析器
	CSRICommM8218Parser mM8218Parser;//GSD数据解析器

	bool mIsGetACK;
	std::string mCommandACK;
	std::string mParamesACK;

};

#endif

