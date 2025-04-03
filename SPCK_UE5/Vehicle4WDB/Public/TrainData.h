// TrainData.h

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "TrainData.generated.h"

/**
 * ʾ���ṹ�壬�洢��UDP���յ���������Ϣ��
 * ������ʾ�� 77 �� double �Ĳ�ַ�ʽ��
 */
USTRUCT(BlueprintType)
struct FTrainData
{
    GENERATED_BODY()

public:
    // 1) sim_time, 2) y_spcktime, 3) y_cb_vx
    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    double SimTime;

    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    double SPCKTime;

    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    double CarBodyVx;

    // 4~9) ����6���ɶ� (X,Y,Z, roll,yaw,pitch)
    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    FVector CarBodyLocation;   // (X, Y, Z)

    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    FRotator CarBodyRotation;  // (Pitch, Yaw, Roll) ���� ע�����ʱҪ����

    // 10~17) 8���ֵ���ת�ٶ� rotw
    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    TArray<double> WheelsRotSpeed; // size=8

    // 18~23) ת��� #1��X,Y,Z, roll,yaw,pitch��
    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    FVector Bogie01Location;

    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    FRotator Bogie01Rotation;

    // 24~29) ת��� #2
    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    FVector Bogie02Location;

    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    FRotator Bogie02Rotation;

    // 30~53) 4���ֶ�(ÿ�� 6 DOF)
    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    TArray<FVector> WheelsetLocations; // size=4

    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    TArray<FRotator> WheelsetRotations; // size=4

    // 54~61) 8������ת�� rota
    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    TArray<double> WheelsRotation; // size=8

    // 62~69) 8���ܸ� pitch
    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    TArray<double> BarsPitch; // size=8

    // 70~73) 4���ֶ� vy
    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    TArray<double> WheelsetVY; // size=4

    // 74~77) 4���ֶ� vyaw
    UPROPERTY(BlueprintReadWrite, Category = "TrainData")
    TArray<double> WheelsetVYaw; // size=4

    // ���캯������ʼ������ߴ�
    FTrainData()
    {
        SimTime = 0.0;
        SPCKTime = 0.0;
        CarBodyVx = 0.0;

        CarBodyLocation = FVector::ZeroVector;
        CarBodyRotation = FRotator::ZeroRotator;

        WheelsRotSpeed.SetNum(8);

        Bogie01Location = FVector::ZeroVector;
        Bogie01Rotation = FRotator::ZeroRotator;
        Bogie02Location = FVector::ZeroVector;
        Bogie02Rotation = FRotator::ZeroRotator;

        WheelsetLocations.SetNum(4);
        WheelsetRotations.SetNum(4);

        WheelsRotation.SetNum(8);
        BarsPitch.SetNum(8);

        WheelsetVY.SetNum(4);
        WheelsetVYaw.SetNum(4);
    }
};
