// MoveComponent_Bogie.cpp

#include "MoveComponent_Bogie.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"
#include "UDPReceiver.h"  // 调用GetLatestTrainData
#include "TrainData.h"    // FTrainData

static const float DistanceScale = 100.f;

UMoveComponent_Bogie::UMoveComponent_Bogie()
{
    PrimaryComponentTick.bCanEverTick = true;

    // 默认为前部转向架
    BogieIndex = 0;

    // 默认直接使用世界坐标
    bUseWorldTransform = true;

    // 默认插值速度
    InterpSpeed = 5.0f;

    bInitialized = false;

    // 先将插值位置初始化为零（BeginPlay中会再次重置）
    CurrentLocation = FVector::ZeroVector;
    CurrentRotation = FRotator::ZeroRotator;

    // 同时设置默认位置和旋转
    SetupDefaultTransform();

    //UE_LOG(LogTemp, Log, TEXT("Bogie%d 构造：默认位置 X=%.2f Y=%.2f Z=%.2f"),
     //   BogieIndex, DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z);
}

void UMoveComponent_Bogie::BeginPlay()
{
    Super::BeginPlay();

    // 重新设置默认变换，确保用正确的 BogieIndex
    SetupDefaultTransform();

    // 将当前位置重置为默认位置，以确保初始显示正确
    CurrentLocation = DefaultLocation;
    CurrentRotation = DefaultRotation;

    //UE_LOG(LogTemp, Log, TEXT("Bogie%d BeginPlay：默认位置 X=%.2f Y=%.2f Z=%.2f"),
        //BogieIndex, DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z);

    // 等待第一次成功获取数据后，再将 bInitialized 置为true
    bInitialized = false;
}

void UMoveComponent_Bogie::SetBogieIndex(int32 NewIndex)
{
    if (BogieIndex != NewIndex)
    {
        //UE_LOG(LogTemp, Log, TEXT("Bogie索引从 %d 更改为 %d"), BogieIndex, NewIndex);
        BogieIndex = NewIndex;

        // 重新设置默认变换
        SetupDefaultTransform();

        // 如果已经初始化，可能需要重置位置
        if (bInitialized)
        {
            CurrentLocation = DefaultLocation;
            CurrentRotation = DefaultRotation;
        }

        //UE_LOG(LogTemp, Log, TEXT("Bogie%d 索引更新后：默认位置 X=%.2f Y=%.2f Z=%.2f"),
            //BogieIndex, DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z);
    }
}

void UMoveComponent_Bogie::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1) 确保有UDPReceiver可用
    if (!UDPReceiverRef)
    {
        // 如果没有UDPReceiver，直接使用默认值（可保持静止或根据需求处理）
        if (!bInitialized)
        {
            // 未初始化前直接跳到默认
            CurrentLocation = DefaultLocation;
            CurrentRotation = DefaultRotation;
            bInitialized = true;

            //UE_LOG(LogTemp, Verbose, TEXT("Bogie%d 初始化：无UDP接收器，使用默认位置 X=%.2f Y=%.2f Z=%.2f"),
                //BogieIndex, DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z);
        }
        else
        {
            // 已初始化后进行插值
            CurrentLocation = FMath::VInterpTo(CurrentLocation, DefaultLocation, DeltaTime, InterpSpeed);
            CurrentRotation = FMath::RInterpTo(CurrentRotation, DefaultRotation, DeltaTime, InterpSpeed);
        }
    }
    else
    {
        // 2) 获取最新TrainData
        FTrainData TrainData;
        bool bHasData = UDPReceiverRef->GetLatestTrainData(TrainData);

        // 定义目标位置与旋转，默认先使用Default值
        FVector TargetLocation = DefaultLocation;
        FRotator TargetRotation = DefaultRotation;

        // 3) 如果有数据，更新目标位置与旋转
        if (bHasData)
        {
            if (BogieIndex == 0)
            {
                // 前部转向架数据
                TargetLocation = TrainData.Bogie01Location;
                TargetRotation = TrainData.Bogie01Rotation;
            }
            else
            {
                // 后部转向架数据
                TargetLocation = TrainData.Bogie02Location;
                TargetRotation = TrainData.Bogie02Rotation;
            }

            // 首次收到UDP数据时记录日志
            if (!bInitialized)
            {
                //UE_LOG(LogTemp, Log, TEXT("Bogie%d 首次UDP数据：位置 X=%.2f Y=%.2f Z=%.2f"),
                    //BogieIndex, TargetLocation.X, TargetLocation.Y, TargetLocation.Z);
            }
        }
        else if (!bInitialized)
        {
            // 有UDP接收器但还没有数据时记录日志
            //UE_LOG(LogTemp, Verbose, TEXT("Bogie%d：UDP接收器就绪但无数据，使用默认位置 X=%.2f Y=%.2f Z=%.2f"),
                //BogieIndex, DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z);
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
                CurrentLocation,    // 上一帧插值位置
                TargetLocation,     // 本帧目标位置
                DeltaTime,          // 帧DeltaTime
                InterpSpeed         // 插值速度
            );

            // 旋转插值
            CurrentRotation = FMath::RInterpTo(
                CurrentRotation,
                TargetRotation,
                DeltaTime,
                InterpSpeed
            );
        }
    }

    // 5) 将插值结果应用
    AActor* OwnerActor = GetOwner();
    if (!OwnerActor)
    {
        return;
    }

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

void UMoveComponent_Bogie::SetupDefaultTransform()
{
    // 根据BogieIndex设置不同默认位置
    if (BogieIndex == 0)
    {
        // 前部转向架默认位置
        DefaultLocation = FVector(DistanceScale * (8.75f), DistanceScale * 0.0f, DistanceScale * (-0.43f));
    }
    else
    {
        // 后部转向架默认位置
        DefaultLocation = FVector(DistanceScale * (-8.75f), DistanceScale * 0.0f, DistanceScale * (-0.43f));
    }

    // 其余自由度为0
    DefaultRotation = FRotator::ZeroRotator;
}