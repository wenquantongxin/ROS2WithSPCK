// UDPReceiver.h
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Common/UdpSocketReceiver.h" // 需要引入Networking相关头
#include "HAL/RunnableThread.h"
#include "HAL/CriticalSection.h"
#include "Sockets.h"
#include "SocketSubsystem.h"

#include "TrainData.h" // 引入上面定义的FTrainData

#include "UDPReceiver.generated.h"

UCLASS()
class VEHICLE4WDB_API AUDPReceiver : public AActor
{
    GENERATED_BODY()

public:
    // 构造函数
    AUDPReceiver();

    // 用于从外部获取最新的列车数据
    UFUNCTION(BlueprintCallable, Category = "UDPReceiver")
    bool GetLatestTrainData(FTrainData& OutData) const;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    // 用于在Tick里做心跳检测等(本例无需操作可留空)
    virtual void Tick(float DeltaSeconds) override;

private:
    // ============== Socket相关 ==============
    bool InitializeUDPReceiver();
    void CloseSocket();

    // 回调函数，当UDP收到数据时触发
    void Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt);

    // 是否验证数据
    bool IsValidData(double x, double y, double z, double roll, double yaw, double pitch);

private:
    // Socket指针
    FSocket* ListenSocket;

    // 负责异步接收的对象
    TSharedPtr<FUdpSocketReceiver> UDPReceiver;

    // 用来保护多线程写数据
    mutable FCriticalSection DataMutex;

    // 存放最新的列车/车体数据
    FTrainData LatestTrainData;

    // 期望一次UDP包的字节数
    int32 ExpectedDataSize;

    /**
     * @brief 指示是否已经收到过至少一次有效的UDP数据
     * 初始为 false，成功解析一次数据后置 true
     */
    bool bHasValidData;
};
