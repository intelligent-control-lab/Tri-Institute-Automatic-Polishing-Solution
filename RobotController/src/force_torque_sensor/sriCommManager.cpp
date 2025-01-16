#include "sriCommManager.h"



CSRICommManager::CSRICommManager()
{
}


CSRICommManager::~CSRICommManager()
{
}
//��ʼ��
bool CSRICommManager::Init()
{
	mTCPClient.OpenTCP("192.168.0.108", 4008);
	mTCPClient.AddCommParser(&mATParser);
	mTCPClient.AddCommParser(&mM8218Parser);


	//������ͨѶʧ�ܴ������� 
	SRICommNetworkFailureCallbackFunction networkFailureCallback = std::bind(&CSRICommManager::OnNetworkFailure, this, std::placeholders::_1);
	mTCPClient.SetNetworkFailureCallbackFunction(networkFailureCallback);


	//��ACKָ�������   	
	SRICommATCallbackFunction atCallback = std::bind(&CSRICommManager::OnCommACK, this, std::placeholders::_1);
	mATParser.SetATCallbackFunction(atCallback);


	//��M8128���ݴ�������
	SRICommM8218CallbackFunction m8218Callback = std::bind(&CSRICommManager::OnCommM8218, this, 
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
		std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
	mM8218Parser.SetM8218CallbackFunction(m8218Callback);

	return true;
}
//����
bool CSRICommManager::Run()
{

	//����TCP����
	if (mTCPClient.Connect() == false)
	{
		return false;
	}

	////��m8128����ָ����������ϴ���ʽ







	//��m8128����ָ����ò�����
	if (SendCommand("SMPF", "200") == false)
	{
		return false;
	}


	//��m8128����ָ���������У��ģʽ
	if (SendCommand("DCKMD", "SUM") == false)
	{
		return false;
	}
	//Send command to M8128 Start to upload sensor data continuously��
	//��m8128����ָ���ʼ�����ϴ�����������
	if (SendCommand("GSD", "") == false)
	{


	}
	return true;
}

bool CSRICommManager::Stop()
{
	mTCPClient.CloseTCP();
	return true;
}

//ͨѶ�쳣����
bool CSRICommManager::OnNetworkFailure(std::string infor)
{
	printf("OnNetworkFailure = %s\n", infor.data());
	return true;
}



//��������
bool CSRICommManager::SendCommand(std::string command, std::string parames)
{
	mIsGetACK = false;
	mCommandACK = "";
	mParamesACK = "";


	//���ATָ��
	std::string atCommand = "AT+" + command + "=" + parames + "\r\n";
	if (mTCPClient.OnSendData((BYTE*)atCommand.data(), (int)atCommand.length()) == false)
	{
		return false;
	}

#ifdef  IS_WINDOWS_OS
	std::clock_t start = clock();
	while (true)
	{
		if (mIsGetACK == true)
		{
			break;
		}
		std::clock_t end = clock();
		long span = end - start;
		if (span >= 10000)
		{
			return false;
		}
	}
#else
	timeval start, end;
	gettimeofday(&start, NULL);
	while (true)
	{
		if (mIsGetACK == true)
		{
			break;
		}
		gettimeofday(&end, NULL);
		long span = 1000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000;
		if (span >= 10000)
		{
			return false;
		}
	}
#endif

	if (mCommandACK != command)
	{

		return false;
	}

	printf("ACK+%s=%s", mCommandACK.data(), mParamesACK.data());

	return true;
}


//ACKӦ����
bool CSRICommManager::OnCommACK(std::string command)
{
	int index = (int)command.find('=');
	if (index == -1)
	{
		mCommandACK = command;
		mParamesACK = "";
	}
	else
	{
		mCommandACK = command.substr(0, index);
		mParamesACK = command.substr(index+1);
	}
	mCommandACK = mCommandACK.substr(4);	
	
	mIsGetACK = true;//Ӧ����ȷ

	return true;
}

//M8128����
bool CSRICommManager::OnCommM8218(float fx, float fy, float fz, float mx, float my, float mz)
{
	printf("M8218 = %f, %f, %f,   %f, %f, %f\n", fx, fy, fz, mx, my, mz);

	return true;
}