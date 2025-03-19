// UDPReceiver.h
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Common/UdpSocketReceiver.h" // ��Ҫ����Networking���ͷ
#include "HAL/RunnableThread.h"
#include "HAL/CriticalSection.h"
#include "Sockets.h"
#include "SocketSubsystem.h"

#include "TrainData.h" // �������涨���FTrainData

#include "UDPReceiver.generated.h"

UCLASS()
class VEHICLE4WDB_API AUDPReceiver : public AActor
{
    GENERATED_BODY()

public:
    // ���캯��
    AUDPReceiver();

    // ���ڴ��ⲿ��ȡ���µ��г�����
    UFUNCTION(BlueprintCallable, Category = "UDPReceiver")
    bool GetLatestTrainData(FTrainData& OutData) const;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    // ������Tick������������(�����������������)
    virtual void Tick(float DeltaSeconds) override;

private:
    // ============== Socket��� ==============
    bool InitializeUDPReceiver();
    void CloseSocket();

    // �ص���������UDP�յ�����ʱ����
    void Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt);

    // �Ƿ���֤����
    bool IsValidData(double x, double y, double z, double roll, double yaw, double pitch);

private:
    // Socketָ��
    FSocket* ListenSocket;

    // �����첽���յĶ���
    TSharedPtr<FUdpSocketReceiver> UDPReceiver;

    // �����������߳�д����
    mutable FCriticalSection DataMutex;

    // ������µ��г�/��������
    FTrainData LatestTrainData;

    // ����һ��UDP�����ֽ���
    int32 ExpectedDataSize;

    /**
     * @brief ָʾ�Ƿ��Ѿ��յ�������һ����Ч��UDP����
     * ��ʼΪ false���ɹ�����һ�����ݺ��� true
     */
    bool bHasValidData;
};
