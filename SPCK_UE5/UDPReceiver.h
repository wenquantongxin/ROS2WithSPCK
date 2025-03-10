#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Sockets.h"
#include "Networking.h"
#include "Common/UdpSocketReceiver.h"
#include "Common/UdpSocketBuilder.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "SocketSubsystem.h"
#include "HAL/CriticalSection.h"
#include "Containers/Array.h"
#include "Serialization/ArrayReader.h"

#include "UDPReceiver.generated.h"

/**
 * 在场景中放置此Actor，用于接收外部6DoF数据（x,y,z,pitch,yaw,roll）
 */
UCLASS()
class MYDEMO_0309A_API AUDPReceiver : public AActor
{
    GENERATED_BODY()

public:
    // 构造函数
    AUDPReceiver();

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    // Socket，用于监听UDP端口
    FSocket* ListenSocket;

    // 异步接收器
    TSharedPtr<FUdpSocketReceiver> UDPReceiver;

    // 接收到数据时的回调函数
    void Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt);

    // 线程安全保护 6DoF 数据
    FCriticalSection DataMutex;

    // 最新的位置信息
    FVector LatestLocation;
    // 最新的旋转信息
    FRotator LatestRotation;

public:
    // 供外部（比如MoveComponent）查询最新的 6DoF 数据
    UFUNCTION(BlueprintCallable, Category = "UDP")
    bool GetLatestTransformData(FVector& OutLocation, FRotator& OutRotation);
};
