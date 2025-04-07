// TrainHUDActor.cpp
// ���� UDP Actor ����ʾ HUD������ȡ UDP ���� ���� UDPReceiverPtr->GetLatestTrainData(LatestData)
// ���������ݴ����߼�

#include "TrainHUDActor.h"
#include "TrainDataWidget.h"
#include "UDPReceiver.h"  
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"

ATrainHUDActor::ATrainHUDActor()
{
    PrimaryActorTick.bCanEverTick = true;   // ��Tick�����UI
    bAutoFindUDPReceiver = false;           // �Ƿ��Զ�Ѱ�� BP_UDP 
    ManualUDPReceiverRef = nullptr;

    TrainDataWidgetInstance = nullptr;
    UDPReceiverPtr = nullptr;
}

void ATrainHUDActor::BeginPlay()
{
    Super::BeginPlay();

    // 1) �ҵ������� UDPReceiver
    if (bAutoFindUDPReceiver)
    {
        // ���ҹؿ�������AUDPReceiver
        TArray<AActor*> FoundReceivers;
        UGameplayStatics::GetAllActorsOfClass(GetWorld(), AUDPReceiver::StaticClass(), FoundReceivers);
        if (FoundReceivers.Num() > 0)
        {
            // ֻ�õ�һ�� AUDPReceiver��Ҳ���������ӵ�ɸѡ
            UDPReceiverPtr = Cast<AUDPReceiver>(FoundReceivers[0]);
        }
    }
    else
    {
        UDPReceiverPtr = ManualUDPReceiverRef;
    }

    // 2) ����HUD Widget
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

    // 3) ÿ֡�����Receiver��Widget���ͻ�ȡ�ٶȸ���
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

            // 0) �����ڲ�����ʱ��
            TrainDataWidgetInstance->UpdateSPCKTime(SPCK_time);

            // 1) �����ٶ�
            TrainDataWidgetInstance->UpdateSpeed(Speed_mps);

            // 2) �������
            TrainDataWidgetInstance->UpdateTrackS(LongitudinalThroughTrack_m);

            // 3) ���� Sperling ָ��
            TrainDataWidgetInstance->UpdateSperlingY(SperlingY);
            TrainDataWidgetInstance->UpdateSperlingZ(SperlingZ);

            // 4) �����ѹ�ϵ��
            TrainDataWidgetInstance->UpdateDerailment_w1(Derailment_w1);
            TrainDataWidgetInstance->UpdateDerailment_w2(Derailment_w2);

            /*
            // 5) ���µ������
            TrainDataWidgetInstance->UpdatePower_w1(Power_w1);
            TrainDataWidgetInstance->UpdatePower_w2(Power_w2);
            */

        }
    }
}
