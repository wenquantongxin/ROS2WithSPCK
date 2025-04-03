// File: TrainHUDActor.cpp

#include "TrainHUDActor.h"
#include "TrainDataWidget.h"
#include "UDPReceiver.h"        // 您已有的AUDPReceiver
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"

ATrainHUDActor::ATrainHUDActor()
{
    PrimaryActorTick.bCanEverTick = true; // 我们要在Tick里更新UI
    bAutoFindUDPReceiver = false; // 是否自动寻找 BP_UDP 
    ManualUDPReceiverRef = nullptr;

    TrainDataWidgetInstance = nullptr;
    UDPReceiverPtr = nullptr;
}

void ATrainHUDActor::BeginPlay()
{
    Super::BeginPlay();

    // 1) 找到或设置 UDPReceiver
    if (bAutoFindUDPReceiver)
    {
        // 查找关卡中所有AUDPReceiver
        TArray<AActor*> FoundReceivers;
        UGameplayStatics::GetAllActorsOfClass(GetWorld(), AUDPReceiver::StaticClass(), FoundReceivers);
        if (FoundReceivers.Num() > 0)
        {
            // 只拿第一个，也可做更复杂的筛选
            UDPReceiverPtr = Cast<AUDPReceiver>(FoundReceivers[0]);
        }
    }
    else
    {
        UDPReceiverPtr = ManualUDPReceiverRef;
    }

    // 2) 创建HUD Widget
    if (TrainDataWidgetClass)
    {
        TrainDataWidgetInstance = CreateWidget<UTrainDataWidget>(GetWorld(), TrainDataWidgetClass);
        if (TrainDataWidgetInstance)
        {
            TrainDataWidgetInstance->AddToViewport();
        }
    }
}

void ATrainHUDActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // 3) 每帧如果有Receiver和Widget，就获取速度更新
    if (UDPReceiverPtr && TrainDataWidgetInstance)
    {
        FTrainData LatestData;
        if (UDPReceiverPtr->GetLatestTrainData(LatestData))
        {
            float Speed_mps = LatestData.CarBodyVx;
            float LongitudinalThroughTrack_m = LatestData.CarBodyLocation.X / 100;

            // 1) 更新速度
            TrainDataWidgetInstance->UpdateSpeed(Speed_mps);

            // 2) 更新里程
            TrainDataWidgetInstance->UpdateTrackS(LongitudinalThroughTrack_m);

        }
    }
}
