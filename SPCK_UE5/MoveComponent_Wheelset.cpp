#include "MoveComponent_Wheelset.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"

// ���UDP������ & ����
#include "UDPReceiver.h"
#include "TrainData.h"
#include "Components/StaticMeshComponent.h"

static const float DistanceScale = 100.f;

UMoveComponent_Wheelset::UMoveComponent_Wheelset()
{
    PrimaryComponentTick.bCanEverTick = true;

    WheelsetIndex = 0;
    bUseWorldTransform = true;
    InterpSpeed = 5.0f;
    WheelRotationSmoothFactor = 2.0f;

    bInitialized = false;
    UDPReceiverRef = nullptr;

    LeftWheelIndex = 0;
    RightWheelIndex = 1;

    CurrentLocation = FVector::ZeroVector;
    CurrentRotation = FRotator::ZeroRotator;

    // ��ʼ���ǶȺ��ٶȱ���
    AccumLeftWheelRotation = 0.f;
    AccumRightWheelRotation = 0.f;
    LeftWheelRotSpeed = 0.f;
    RightWheelRotSpeed = 0.f;
    TargetLeftWheelRotSpeed = 0.f;
    TargetRightWheelRotSpeed = 0.f;

    SetupDefaultTransform();

    LeftWheelMesh = nullptr;
    RightWheelMesh = nullptr;
}

void UMoveComponent_Wheelset::BeginPlay()
{
    Super::BeginPlay();

    SetupDefaultTransform();

    CurrentLocation = DefaultLocation;
    CurrentRotation = DefaultRotation;

    LeftWheelIndex = WheelsetIndex * 2;
    RightWheelIndex = WheelsetIndex * 2 + 1;

    // �Զ����Ҳ����ó�������
    TArray<USceneComponent*> ChildComponents;
    GetChildrenComponents(false, ChildComponents);

    for (USceneComponent* Child : ChildComponents)
    {
        UStaticMeshComponent* MeshComp = Cast<UStaticMeshComponent>(Child);
        if (MeshComp)
        {
            FString CompName = Child->GetName();
            if (CompName.Contains(TEXT("LeftWheel"), ESearchCase::IgnoreCase))
            {
                LeftWheelMesh = MeshComp;
                //UE_LOG(LogTemp, Warning, TEXT("�Զ����� LeftWheelMesh = %s"), *CompName);

                // ȷ�����ֳ�ʼ��ת��ȷ����Ҫ��Z����ת90�ȣ�
                FRotator InitialRotation = MeshComp->GetRelativeRotation();
                InitialRotation.Roll += 90.0f; // ��Z����ת90��
                MeshComp->SetRelativeRotation(InitialRotation);
            }
            else if (CompName.Contains(TEXT("RightWheel"), ESearchCase::IgnoreCase))
            {
                RightWheelMesh = MeshComp;
                //UE_LOG(LogTemp, Warning, TEXT("�Զ����� RightWheelMesh = %s"), *CompName);
            }
        }
    }

    // ��¼��ͼ���趨�������ת�����ֳ�ʼ���룩
    if (LeftWheelMesh)
    {
        InitialLeftWheelRot = LeftWheelMesh->GetRelativeRotation();
        //UE_LOG(LogTemp, Warning, TEXT("���ֳ�ʼ��ת��(P=%.3f, Y=%.3f, R=%.3f)"),
            //InitialLeftWheelRot.Pitch, InitialLeftWheelRot.Yaw, InitialLeftWheelRot.Roll);
    }
    if (RightWheelMesh)
    {
        InitialRightWheelRot = RightWheelMesh->GetRelativeRotation();
        //UE_LOG(LogTemp, Warning, TEXT("���ֳ�ʼ��ת��(P=%.3f, Y=%.3f, R=%.3f)"),
            //InitialRightWheelRot.Pitch, InitialRightWheelRot.Yaw, InitialRightWheelRot.Roll);
    }

    bInitialized = false;

    //UE_LOG(LogTemp, Warning, TEXT("[BeginPlay] Wheelset=%d -> DefaultLocation=(%.3f, %.3f, %.3f), DefaultRotation=(P=%.3f, Y=%.3f, R=%.3f)"),
        //WheelsetIndex,
        //DefaultLocation.X, DefaultLocation.Y, DefaultLocation.Z,
        //DefaultRotation.Pitch, DefaultRotation.Yaw, DefaultRotation.Roll);
}

void UMoveComponent_Wheelset::SetupDefaultTransform()
{
    float xPos = 0.0f;
    switch (WheelsetIndex)
    {
    case 0:
        xPos = 10.0f;  // +10 ��
        break;
    case 1:
        xPos = 7.5f;   // +7.5 ��
        break;
    case 2:
        xPos = -7.5f;  // -7.5 ��
        break;
    case 3:
        xPos = -10.0f; // -10 ��
        break;
    default:
        xPos = 0.0f;
        //UE_LOG(LogTemp, Warning, TEXT("��Ч��WheelsetIndex: %d"), WheelsetIndex);
        break;
    }

    DefaultLocation = FVector(DistanceScale * xPos, 0.0f, DistanceScale * (-0.43f));
    DefaultRotation = FRotator::ZeroRotator;
}

void UMoveComponent_Wheelset::SetWheelsetIndex(int32 NewIndex)
{
    if (NewIndex >= 0 && NewIndex <= 3 && WheelsetIndex != NewIndex)
    {
        WheelsetIndex = NewIndex;
        SetupDefaultTransform();

        LeftWheelIndex = WheelsetIndex * 2;
        RightWheelIndex = WheelsetIndex * 2 + 1;

        if (bInitialized)
        {
            CurrentLocation = DefaultLocation;
            CurrentRotation = DefaultRotation;
        }

        //UE_LOG(LogTemp, Log, TEXT("WheelsetIndex ���� -> %d"), WheelsetIndex);
    }
}

void UMoveComponent_Wheelset::TickComponent(float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1) �������
    if (WheelsetIndex < 0 || WheelsetIndex > 3)
    {
        return;
    }

    // 2) ���û����UDPReceiver����Ĭ��λ��
    if (!UDPReceiverRef)
    {
        if (!bInitialized)
        {
            CurrentLocation = DefaultLocation;
            CurrentRotation = DefaultRotation;
            bInitialized = true;
        }
        else
        {
            CurrentLocation = FMath::VInterpTo(CurrentLocation, DefaultLocation, DeltaTime, InterpSpeed);
            CurrentRotation = FMath::RInterpTo(CurrentRotation, DefaultRotation, DeltaTime, InterpSpeed);
        }

        // û��UDP����ʱ����ת�ٶ���Ϊ0
        TargetLeftWheelRotSpeed = 0.0f;
        TargetRightWheelRotSpeed = 0.0f;
    }
    else
    {
        // 3) ��ȡ���� FTrainData
        FTrainData TrainData;
        bool bHasData = UDPReceiverRef->GetLatestTrainData(TrainData);

        FVector TargetLocation = DefaultLocation;
        FRotator TargetRotation = DefaultRotation;

        // 4) ��������ݣ������
        if (bHasData &&
            WheelsetIndex < TrainData.WheelsetLocations.Num() &&
            WheelsetIndex < TrainData.WheelsetRotations.Num())
        {
            TargetLocation = TrainData.WheelsetLocations[WheelsetIndex];
            TargetRotation = TrainData.WheelsetRotations[WheelsetIndex];

            // 5) ��UDP���ݻ�ȡ������ת�ٶ�
            if (LeftWheelIndex >= 0 && LeftWheelIndex < TrainData.WheelsRotSpeed.Num() &&
                RightWheelIndex >= 0 && RightWheelIndex < TrainData.WheelsRotSpeed.Num())
            {
                // ת��Ϊ�Ƕ�/��
                TargetLeftWheelRotSpeed = TrainData.WheelsRotSpeed[LeftWheelIndex] * RadToDeg;
                TargetRightWheelRotSpeed = TrainData.WheelsRotSpeed[RightWheelIndex] * RadToDeg;
            }
            else
            {
                TargetLeftWheelRotSpeed = 0.0f;
                TargetRightWheelRotSpeed = 0.0f;
            }
        }
        else
        {
            // û�����ݣ���ת�ٶ���Ϊ0
            TargetLeftWheelRotSpeed = 0.0f;
            TargetRightWheelRotSpeed = 0.0f;
        }

        // 6) ƽ����ֵλ�ú���ת
        if (!bInitialized)
        {
            CurrentLocation = TargetLocation;
            CurrentRotation = TargetRotation;
            bInitialized = true;
        }
        else
        {
            CurrentLocation = FMath::VInterpTo(CurrentLocation, TargetLocation, DeltaTime, InterpSpeed);
            CurrentRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaTime, InterpSpeed);
        }
    }

    // 7) Ӧ�ñ任��Actor
    AActor* OwnerActor = GetOwner();
    if (!OwnerActor) return;

    if (bUseWorldTransform)
    {
        OwnerActor->SetActorLocationAndRotation(CurrentLocation, CurrentRotation);
    }
    else
    {
        SetRelativeLocationAndRotation(CurrentLocation, CurrentRotation);
    }

    // 8) ƽ��������ת�ٶ�
    LeftWheelRotSpeed = FMath::FInterpTo(LeftWheelRotSpeed, TargetLeftWheelRotSpeed,
        DeltaTime, WheelRotationSmoothFactor);
    RightWheelRotSpeed = FMath::FInterpTo(RightWheelRotSpeed, TargetRightWheelRotSpeed,
        DeltaTime, WheelRotationSmoothFactor);

    // 9) ������ת�ٶȸ����ۻ��Ƕ�
    AccumLeftWheelRotation += LeftWheelRotSpeed * DeltaTime;
    AccumRightWheelRotation += RightWheelRotSpeed * DeltaTime;

    // �淶����[0,360]���䣬��ֹ����������
    AccumLeftWheelRotation = FMath::Fmod(AccumLeftWheelRotation, 360.0f);
    if (AccumLeftWheelRotation < 0.0f) AccumLeftWheelRotation += 360.0f;

    AccumRightWheelRotation = FMath::Fmod(AccumRightWheelRotation, 360.0f);
    if (AccumRightWheelRotation < 0.0f) AccumRightWheelRotation += 360.0f;

    // 10) Ӧ���ۻ��Ƕȵ�����������
    if (LeftWheelMesh)
    {
        FRotator NewLeftRot = InitialLeftWheelRot + FRotator(0.f, 0.f, AccumLeftWheelRotation);
        LeftWheelMesh->SetRelativeRotation(NewLeftRot);
    }

    if (RightWheelMesh)
    {
        // ע��������ת������Ҫȡ����
        FRotator NewRightRot = InitialRightWheelRot + FRotator(0.f, 0.f, -AccumRightWheelRotation);
        RightWheelMesh->SetRelativeRotation(NewRightRot);
    }
}