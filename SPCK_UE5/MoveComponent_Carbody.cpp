// MoveComponent_Carbody.cpp

#include "MoveComponent_Carbody.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"
#include "UDPReceiver.h"  // 调用GetLatestTrainData
#include "TrainData.h"    // FTrainData

static const float cb_hc = 1.2f;
static const float DistanceScale = 100.f;

UMoveComponent_Carbody::UMoveComponent_Carbody()
{
    PrimaryComponentTick.bCanEverTick = true;

    // 默认直接使用世界坐标
    bUseWorldTransform = true;

    // 缺省插值速度
    InterpSpeed = 5.0f;

    bInitialized = false;

    // 先将插值位置初始化为车体默认位置
    SetupDefaultTransform();
    CurrentLocation = DefaultLocation;
    CurrentRotation = DefaultRotation;
}

void UMoveComponent_Carbody::SetupDefaultTransform()
{
    // 设置车体默认位置为(0, 0, cb_hc)，注意单位转换
    DefaultLocation = FVector(0.0f, 0.0f, cb_hc * DistanceScale);

    // 默认旋转为零
    DefaultRotation = FRotator::ZeroRotator;
}

void UMoveComponent_Carbody::BeginPlay()
{
    Super::BeginPlay();

    // 重新设置默认变换
    SetupDefaultTransform();

    // 将当前位置重置为默认位置，以确保初始显示正确
    CurrentLocation = DefaultLocation;
    CurrentRotation = DefaultRotation;

    // 等待第一次接收到数据后，再将 bInitialized 置为true
    bInitialized = false;
}

void UMoveComponent_Carbody::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1) 确保有UDPReceiver可用
    if (!UDPReceiverRef)
    {
        // 如果没有UDPReceiver，直接使用默认值
        if (!bInitialized)
        {
            // 未初始化前直接跳到默认
            CurrentLocation = DefaultLocation;
            CurrentRotation = DefaultRotation;
            bInitialized = true;
        }
        else
        {
            // 已初始化后进行插值
            CurrentLocation = FMath::VInterpTo(CurrentLocation, DefaultLocation, DeltaTime, InterpSpeed);
            CurrentRotation = FMath::RInterpTo(CurrentRotation, DefaultRotation, DeltaTime, InterpSpeed);
        }

        // 应用位置和旋转
        ApplyTransform();
        return;
    }

    // 2) 获取最新TrainData
    FTrainData TrainData;
    bool bHasData = UDPReceiverRef->GetLatestTrainData(TrainData);

    // 定义目标位置与旋转，默认先使用Default值
    FVector TargetLocation = DefaultLocation;
    FRotator TargetRotation = DefaultRotation;

    // 3) 如果有数据，更新目标位置与旋转
    if (bHasData)
    {
        TargetLocation = TrainData.CarBodyLocation;
        TargetRotation = TrainData.CarBodyRotation;
    }

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
    ApplyTransform();
}

void UMoveComponent_Carbody::ApplyTransform()
{
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
        SetRelativeLocationAndRotation(CurrentLocation, CurrentRotation);
    }
}