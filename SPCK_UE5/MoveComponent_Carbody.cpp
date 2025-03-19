#include "MoveComponent_Carbody.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"
#include "UDPReceiver.h"  // 调用GetLatestTrainData
#include "TrainData.h"    // FTrainData

UMoveComponent_Carbody::UMoveComponent_Carbody()
{
    PrimaryComponentTick.bCanEverTick = true;

    // 默认直接使用世界坐标
    bUseWorldTransform = true;

    // 缺省插值速度
    InterpSpeed = 5.0f;

    bInitialized = false;
    CurrentLocation = FVector::ZeroVector;
    CurrentRotation = FRotator::ZeroRotator;
}

void UMoveComponent_Carbody::BeginPlay()
{
    Super::BeginPlay();

    // 如果想在初始就让 CurrentLocation = 本Actor 的现有坐标，可在这里读取
    AActor* OwnerActor = GetOwner();
    if (OwnerActor)
    {
        CurrentLocation = OwnerActor->GetActorLocation();
        CurrentRotation = OwnerActor->GetActorRotation();
    }

    // 等待第一次接收到数据后，再将 bInitialized 置为true。
    bInitialized = false;
}

void UMoveComponent_Carbody::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1) 确保有UDPReceiver可用
    if (!UDPReceiverRef)
    {
        return;
    }

    // 2) 获取最新TrainData
    FTrainData TrainData;
    if (!UDPReceiverRef->GetLatestTrainData(TrainData))
    {
        return; // 还没有有效数据
    }

    // 3) 提取目标位置与旋转(世界空间)
    FVector  TargetLocation = TrainData.CarBodyLocation;
    FRotator TargetRotation = TrainData.CarBodyRotation; // (Pitch,Yaw,Roll)

    // 4) 做平滑插值，减少跳动
    if (!bInitialized)
    {
        // 第一次：直接跳到目标位置，避免初始瞬移
        CurrentLocation = TargetLocation;
        CurrentRotation = TargetRotation;
        bInitialized = true;
    }
    else
    {
        // 位置插值
        CurrentLocation = FMath::VInterpTo(
            CurrentLocation,   // 上一帧插值位置
            TargetLocation,    // 本帧想要到达的位置
            DeltaTime,         // 帧DeltaTime
            InterpSpeed        // 插值速度
        );

        // 旋转插值
        CurrentRotation = FMath::RInterpTo(
            CurrentRotation,
            TargetRotation,
            DeltaTime,
            InterpSpeed
        );
    }

    // 5) 将插值结果应用
    AActor* OwnerActor = GetOwner();
    if (!OwnerActor) return;

    if (bUseWorldTransform)
    {
        // 直接Set世界坐标
        OwnerActor->SetActorLocationAndRotation(CurrentLocation, CurrentRotation);
    }
    else
    {
        // 使用相对坐标
        // 注意：本示例直接Set到组件自身的相对坐标，如有父子级Actor或组件，需要做额外换算
        SetRelativeLocationAndRotation(CurrentLocation, CurrentRotation);
    }
}
