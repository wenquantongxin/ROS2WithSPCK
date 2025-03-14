#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Common/UdpSocketReceiver.h"
#include "Common/UdpSocketBuilder.h"
#include "IPAddress.h"
#include "UDPReceiver.generated.h"

UCLASS(BlueprintType)
class PROJECT_4WDBVEHICLE_API AUDPReceiver : public AActor
{
    GENERATED_BODY()
public:
    AUDPReceiver();
    virtual void Tick(float DeltaTime) override;
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    /** 获取最新的位置和旋转数据 */
    UFUNCTION(BlueprintCallable, Category = "Network")
    bool GetLatestTransformData(FVector& OutLocation, FRotator& OutRotation);

private:
    /** 初始化UDP接收器 */
    bool InitializeUDPReceiver();

    /** 关闭Socket连接 */
    void CloseSocket();

    /** Socket接收回调 */
    void Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt);

    /** 验证数据有效性 */
    bool IsValidData(float x, float y, float z, float pitch, float yaw, float roll);

private:
    /** 网络组件 */
    FSocket* ListenSocket;
    TSharedPtr<FUdpSocketReceiver> UDPReceiver;

    /** 数据互斥锁 */
    FCriticalSection DataMutex;

    /** 最新接收的数据 */
    FVector LatestLocation;
    FRotator LatestRotation;

    /** 是否曾经收到过数据（仅用于指示是否收到过有效数据） */
    bool bHasReceivedData;

    /** 预期的数据包大小：6 个 float (XYZ + Pitch/Yaw/Roll) */
    int32 ExpectedDataSize;
};
