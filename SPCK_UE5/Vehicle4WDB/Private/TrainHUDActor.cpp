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

            float SperlingY = LatestData.SperlingYZ[0];
            float SperlingZ = LatestData.SperlingYZ[1];

            float Derailment_w1 = LatestData.DerailmentIndex[0];
            float Derailment_w2 = LatestData.DerailmentIndex[1];

            float Power_w1 = LatestData.InputTorque[0] * LatestData.WheelsRotSpeed[0]; // P = T * n
            float Power_w2 = LatestData.InputTorque[1] * LatestData.WheelsRotSpeed[1];

            // 0) 更新内部仿真时间
            TrainDataWidgetInstance->UpdateSPCKTime(SPCK_time);

            // 1) 更新速度
            TrainDataWidgetInstance->UpdateSpeed(Speed_mps);

            // 2) 更新里程
            TrainDataWidgetInstance->UpdateTrackS(LongitudinalThroughTrack_m);

            // 3) 更新 Sperling 指标
            TrainDataWidgetInstance->UpdateSperlingY(SperlingY);
            TrainDataWidgetInstance->UpdateSperlingZ(SperlingZ);

            // 4) 更新脱轨系数
            TrainDataWidgetInstance->UpdateDerailment_w1(Derailment_w1);
            TrainDataWidgetInstance->UpdateDerailment_w2(Derailment_w2);

            /*
            // 5) 更新电机功率
            TrainDataWidgetInstance->UpdatePower_w1(Power_w1);
            TrainDataWidgetInstance->UpdatePower_w2(Power_w2);
            */

        }
    }
}
