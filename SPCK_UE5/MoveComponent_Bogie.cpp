// MoveComponent_Bogie.cpp

#include "MoveComponent_Bogie.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"
#include "UDPReceiver.h"  // ����GetLatestTrainData
#include "TrainData.h"    // FTrainData

static const float DistanceScale = 100.f;

UMoveComponent_Bogie::UMoveComponent_Bogie()
{
    PrimaryComponentTick.bCanEverTick = true;

    // Ĭ��Ϊǰ��ת���
    BogieIndex = 0;

    // Ĭ��ֱ��ʹ����������
    bUseWorldTransform = true;

    // Ĭ�ϲ�ֵ�ٶ�
    InterpSpeed = 5.0f;

    bInitialized = false;

    // �Ƚ���ֵλ�ó�ʼ��Ϊ�㣨BeginPlay�л��ٴ����ã�
    CurrentLocation = FVector::ZeroVector;
    CurrentRotation = FRotator::ZeroRotator;

    // ͬʱ����Ĭ��λ�ú���ת
    SetupDefaultTransform();

    //UE_LOG(LogTemp, Log, TEXT("Bogie%d ���죺Ĭ��λ�� X=%.2f Y=%.2f Z=%.2f"),
     //   BogieIndex, DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z);
}

void UMoveComponent_Bogie::BeginPlay()
{
    Super::BeginPlay();

    // ��������Ĭ�ϱ任��ȷ������ȷ�� BogieIndex
    SetupDefaultTransform();

    // ����ǰλ������ΪĬ��λ�ã���ȷ����ʼ��ʾ��ȷ
    CurrentLocation = DefaultLocation;
    CurrentRotation = DefaultRotation;

    //UE_LOG(LogTemp, Log, TEXT("Bogie%d BeginPlay��Ĭ��λ�� X=%.2f Y=%.2f Z=%.2f"),
        //BogieIndex, DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z);

    // �ȴ���һ�γɹ���ȡ���ݺ��ٽ� bInitialized ��Ϊtrue
    bInitialized = false;
}

void UMoveComponent_Bogie::SetBogieIndex(int32 NewIndex)
{
    if (BogieIndex != NewIndex)
    {
        //UE_LOG(LogTemp, Log, TEXT("Bogie������ %d ����Ϊ %d"), BogieIndex, NewIndex);
        BogieIndex = NewIndex;

        // ��������Ĭ�ϱ任
        SetupDefaultTransform();

        // ����Ѿ���ʼ����������Ҫ����λ��
        if (bInitialized)
        {
            CurrentLocation = DefaultLocation;
            CurrentRotation = DefaultRotation;
        }

        //UE_LOG(LogTemp, Log, TEXT("Bogie%d �������º�Ĭ��λ�� X=%.2f Y=%.2f Z=%.2f"),
            //BogieIndex, DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z);
    }
}

void UMoveComponent_Bogie::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1) ȷ����UDPReceiver����
    if (!UDPReceiverRef)
    {
        // ���û��UDPReceiver��ֱ��ʹ��Ĭ��ֵ���ɱ��־�ֹ�����������
        if (!bInitialized)
        {
            // δ��ʼ��ǰֱ������Ĭ��
            CurrentLocation = DefaultLocation;
            CurrentRotation = DefaultRotation;
            bInitialized = true;

            //UE_LOG(LogTemp, Verbose, TEXT("Bogie%d ��ʼ������UDP��������ʹ��Ĭ��λ�� X=%.2f Y=%.2f Z=%.2f"),
                //BogieIndex, DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z);
        }
        else
        {
            // �ѳ�ʼ������в�ֵ
            CurrentLocation = FMath::VInterpTo(CurrentLocation, DefaultLocation, DeltaTime, InterpSpeed);
            CurrentRotation = FMath::RInterpTo(CurrentRotation, DefaultRotation, DeltaTime, InterpSpeed);
        }
    }
    else
    {
        // 2) ��ȡ����TrainData
        FTrainData TrainData;
        bool bHasData = UDPReceiverRef->GetLatestTrainData(TrainData);

        // ����Ŀ��λ������ת��Ĭ����ʹ��Defaultֵ
        FVector TargetLocation = DefaultLocation;
        FRotator TargetRotation = DefaultRotation;

        // 3) ��������ݣ�����Ŀ��λ������ת
        if (bHasData)
        {
            if (BogieIndex == 0)
            {
                // ǰ��ת�������
                TargetLocation = TrainData.Bogie01Location;
                TargetRotation = TrainData.Bogie01Rotation;
            }
            else
            {
                // ��ת�������
                TargetLocation = TrainData.Bogie02Location;
                TargetRotation = TrainData.Bogie02Rotation;
            }

            // �״��յ�UDP����ʱ��¼��־
            if (!bInitialized)
            {
                //UE_LOG(LogTemp, Log, TEXT("Bogie%d �״�UDP���ݣ�λ�� X=%.2f Y=%.2f Z=%.2f"),
                    //BogieIndex, TargetLocation.X, TargetLocation.Y, TargetLocation.Z);
            }
        }
        else if (!bInitialized)
        {
            // ��UDP����������û������ʱ��¼��־
            //UE_LOG(LogTemp, Verbose, TEXT("Bogie%d��UDP�����������������ݣ�ʹ��Ĭ��λ�� X=%.2f Y=%.2f Z=%.2f"),
                //BogieIndex, DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z);
        }

        // 4) ��ƽ����ֵ����������
        if (!bInitialized)
        {
            // ��һ�Σ�ֱ������Ŀ��λ�ã������ʼ˲��
            CurrentLocation = TargetLocation;
            CurrentRotation = TargetRotation;
            bInitialized = true;
        }
        else
        {
            // λ�ò�ֵ
            CurrentLocation = FMath::VInterpTo(
                CurrentLocation,    // ��һ֡��ֵλ��
                TargetLocation,     // ��֡Ŀ��λ��
                DeltaTime,          // ֡DeltaTime
                InterpSpeed         // ��ֵ�ٶ�
            );

            // ��ת��ֵ
            CurrentRotation = FMath::RInterpTo(
                CurrentRotation,
                TargetRotation,
                DeltaTime,
                InterpSpeed
            );
        }
    }

    // 5) ����ֵ���Ӧ��
    AActor* OwnerActor = GetOwner();
    if (!OwnerActor)
    {
        return;
    }

    if (bUseWorldTransform)
    {
        // ֱ��Set��������
        OwnerActor->SetActorLocationAndRotation(CurrentLocation, CurrentRotation);
    }
    else
    {
        // ʹ���������
        SetRelativeLocationAndRotation(CurrentLocation, CurrentRotation);
    }
}

void UMoveComponent_Bogie::SetupDefaultTransform()
{
    // ����BogieIndex���ò�ͬĬ��λ��
    if (BogieIndex == 0)
    {
        // ǰ��ת���Ĭ��λ��
        DefaultLocation = FVector(DistanceScale * (8.75f), DistanceScale * 0.0f, DistanceScale * (0.43f));
    }
    else
    {
        // ��ת���Ĭ��λ��
        DefaultLocation = FVector(DistanceScale * (-8.75f), DistanceScale * 0.0f, DistanceScale * (0.43f));
    }

    // �������ɶ�Ϊ0
    DefaultRotation = FRotator::ZeroRotator;
} 