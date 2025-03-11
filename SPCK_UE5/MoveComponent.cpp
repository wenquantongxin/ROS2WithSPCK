#include "MoveComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "UDPReceiver.h"  // 访问AUDPReceiver

UMoveComponent::UMoveComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UMoveComponent::BeginPlay()
{
    Super::BeginPlay();

    // 组件初始相对位置
    StartRelativeLocation = GetRelativeLocation();
    CurrentLocation = StartRelativeLocation;
}

void UMoveComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1. 从UDPReceiver获取最新目标位置、旋转
    if (UDPReceiverActor)
    {
        FVector UDP_Loc;
        FRotator UDP_Rot;
        if (UDPReceiverActor->GetLatestTransformData(UDP_Loc, UDP_Rot))
        {
            // 在此示例中，只用位置来驱动EndRelativeLocation
            // 也可以根据需要直接SetWorldLocation
            EndRelativeLocation = UDP_Loc;

            // 如果想应用旋转，可以插值或直接SetRelativeRotation
            // 这里演示一个简单的插值
            FRotator CurrentRot = GetRelativeRotation();
            FRotator NewRot = FMath::RInterpTo(CurrentRot, UDP_Rot, DeltaTime, 3.0f);
            SetRelativeRotation(NewRot);
        }
    }

    // 2. 按原有逻辑朝 EndRelativeLocation 移动
    float dist = FVector::Dist(CurrentLocation, EndRelativeLocation);
    if (dist < stoppingDistance)
    {
        // 靠近目标就停止
        return;
    }

    FVector Direction = (EndRelativeLocation - CurrentLocation).GetSafeNormal();
    FVector NewLocation = CurrentLocation + (Direction * (speed * DeltaTime));
    SetRelativeLocation(NewLocation);

    // 更新当前记录
    CurrentLocation = NewLocation;
}
