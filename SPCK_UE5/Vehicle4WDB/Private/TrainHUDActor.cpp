// TrainHUDActor.cpp
// 发现 UDP Actor 并显示 HUD，并获取 UDP 数据 —— UDPReceiverPtr->GetLatestTrainData(LatestData)
// 无其他数据处理逻辑

#include "TrainHUDActor.h"
#include "TrainDataWidget.h"
#include "UDPReceiver.h"  
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"

ATrainHUDActor::ATrainHUDActor()
{
    PrimaryActorTick.bCanEverTick = true;   // 在Tick里更新UI
    bAutoFindUDPReceiver = false;           // 是否自动寻找 BP_UDP 
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
            // 只拿第一个 AUDPReceiver，也可做更复杂的筛选
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
            float SPCK_time = LatestData.SPCKTime;
            float Speed_mps = LatestData.CarBodyVx;
            float LongitudinalThroughTrack_m = LatestData.TrackS;

            // 0) 更新内部仿真时间
            TrainDataWidgetInstance->UpdateSPCKTime(SPCK_time);

            // 1) 更新速度
            TrainDataWidgetInstance->UpdateSpeed(Speed_mps);

            // 2) 更新里程
            TrainDataWidgetInstance->UpdateTrackS(LongitudinalThroughTrack_m);

        }
    }
}
