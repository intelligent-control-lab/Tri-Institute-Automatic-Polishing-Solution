#include "sriCommATParser.h"



CSRICommATParser::CSRICommATParser()
{
	mCircularBuffer.Init(102400);
	mAtCallbackFunction = NULL;
}


CSRICommATParser::~CSRICommATParser()
{
}

//ATָ��Ӧ��������
bool CSRICommATParser::SetATCallbackFunction(SRICommATCallbackFunction atCallbackFunction)
{
	mAtCallbackFunction = atCallbackFunction;
	return false;
}

//�������ݴ���
bool CSRICommATParser::OnReceivedData(BYTE* data, int dataLen)
{
	if (data == NULL)
	{
		return false;
	}
	if (dataLen <= 0)
	{
		return false;
	}
	mCircularBuffer.Write(data, dataLen);

	int delLen = 0;
	std::string ack = "";
	if (ParseDataFromBuffer(delLen, ack) == false)
	{
		mCircularBuffer.Clear(delLen);
		return false;
	}
	mCircularBuffer.Clear(delLen);
	
	if (mAtCallbackFunction != NULL)
	{
		mAtCallbackFunction(ack);
	}
	return true;
}

bool CSRICommATParser::OnNetworkFailure(std::string infor)
{
	return true;
}



//����ACKָ��
bool CSRICommATParser::ParseDataFromBuffer(int& delLen, std::string& ack)
{
	int dataLen = 0;
	BYTE* data = mCircularBuffer.ReadTry(dataLen);//��ȡ��r data
	if (data == NULL)
	{
		return false;
	}


	//���ݳ���̫С
	if (dataLen < 4)
	{
		delLen = 0;
		ack = "";
		return false;
	}

	//��ȡ֡ͷλ��
	int headIndex = ParseGetHeadIndex(data, dataLen);
	if (headIndex == -1)
	{
		delLen = dataLen - 3; //�Ҳ���֡ͷ��ɾ����ǰ�������ݣ�����3/4��֡ͷ��
		ack = "";
		return false;
	}


	//��ȡ֡βλ��
	int endIndex = ParseGetEndIndex(data, dataLen, headIndex + 4);
	if (endIndex == -1)
	{
		delLen = headIndex;
		ack = "";
		return false;
	}

	int len = endIndex - headIndex + 2;
	char* command = new char[len];
	memcpy(command, data + headIndex, len);
	std::string commandStr = command;//Ӧ��ָ������
	delete command;
	ack = commandStr;//Ӧ��ָ������
	delLen = endIndex + 2;
	return true;
}

//��ȡACKָ��Ӧ��֡ͷλ��
int CSRICommATParser::ParseGetHeadIndex(BYTE* data, int dataLen)
{
	int headIndex = -1;
	for (int i = 0; i < dataLen - 3; i++)
	{

		if ((data[i] == 65) && (data[i + 1] == 67) && (data[i + 2] == 75) && (data[i + 3] == 43))
		{
			headIndex = i;
			break;
		}
	}
	return headIndex;
}


//��ȡATָ��Ӧ��֡β����
int CSRICommATParser::ParseGetEndIndex(BYTE* data, int dataLen, int index)
{
	int endIndex = -1;
	for (int i = index; i < dataLen - 1; i++)
	{
		if ((data[i] == 13) && (data[i + 1] == 10))
		{
			endIndex = i;
			break;
		}
	}
	return endIndex;
}