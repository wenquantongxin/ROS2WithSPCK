// TrajectorySpline.h

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SplineComponent.h"
#include "Components/SplineMeshComponent.h"
#include "TrajectorySpline.generated.h"

/**
 * ATrajectorySpline
 *
 * ���ܣ�
 *  1. ���ⲿ JSON �ļ���ȡ�켣�������ߣ�s, x, y, z, psi, phi, slope �ȣ���
 *  2. �������ϵת����SIMPACK -> UE5���뵥λת����m -> cm����
 *  3. �� UE �й������� Spline�������ߡ���졢�ҹ죩���������� SplineMesh ������п��ӻ���
 *  4. ��ѡ�ؽ� slope ֵת��Ϊ������ Pitch���� psi (yaw) �� phi (roll) һ��ӳ�䵽 Spline �����ߡ�
 */
UCLASS()
class VEHICLE4WDB_API ATrajectorySpline : public AActor
{
    GENERATED_BODY()

public:
    // ���캯��
    ATrajectorySpline();

    // ����ͼ��C++��ɵ��ã������ⲿ JSON �켣��ת���� UE5 ����ϵ
    UFUNCTION(BlueprintCallable, Category = "Trajectory")
    bool LoadTrajectoryFromJSON(const FString& FilePath);

    // ����ͼ��C++��ɵ��ã�ʹ���Ѽ��صĹ켣���ݣ����ɹ������
    UFUNCTION(BlueprintCallable, Category = "Trajectory")
    void GenerateTrackWithMesh(UStaticMesh* TrackMesh, float MeshScale = 1.0f);

protected:
    // ��Ϸ��ʼʱִ��
    virtual void BeginPlay() override;

    /*------------------------------------------
       ����ɵ����� (�� UE �༭���п�����)
    -------------------------------------------*/

    // �����(��)�������ɹ������ʱ�ɼӵ����߹�ת��
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    float RailBaseCantDeg;

    // ���뾶����΢��(��λ cm)������ѹ���150 cm��Ϊ 143.5 ��
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    float DeltaHalfGaugeCm;

    // �Ƿ���Ⱦ������
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    bool bRenderCenterLine;

    // �Ƿ��� Spline �� Tangent �н�� slope ֵ���㸩��
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    bool bUseSlopeForSplineTangent;

    /*------------------------------------------
       �������
    -------------------------------------------*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USplineComponent* TrajectorySpline;  // ������

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USplineComponent* LeftRailSpline;    // �����

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USplineComponent* RightRailSpline;   // �ҹ���

    /*------------------------------------------
       �켣���ݣ��� UE5 ��������/��̬�洢��
    -------------------------------------------*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> SValues;   // ��� (cm)�������ο����ɲ�����ת

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> XValues;   // x����(����) [UE����]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> YValues;   // y����(����) [UE����]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> ZValues;   // z����(����) [UE����]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> PsiValues; // ����� yaw (rad) [UE����ϵ��]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> PhiValues; // ����� roll (rad) [UE����ϵ��]

    // �����¶� p(s)=dz/ds����������Spline Tangentʱ������
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> SlopeValues; // slope (������, rad=? ��ʵ�� dz/ds )

    /*------------------------------------------
       ����������������ҹ�����
    -------------------------------------------*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<FVector> LeftRailPoints;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<FVector> RightRailPoints;

    /*------------------------------------------
       �洢���ڿ��ӻ��� SplineMesh Component
    -------------------------------------------*/
    UPROPERTY()
    TArray<USplineMeshComponent*> CenterTrackMeshes;

    UPROPERTY()
    TArray<USplineMeshComponent*> LeftTrackMeshes;

    UPROPERTY()
    TArray<USplineMeshComponent*> RightTrackMeshes;

    /*------------------------------------------
       ��ǰʹ�õĹ��������Դ & ����
    -------------------------------------------*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Track")
    UStaticMesh* CurrentTrackMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Track")
    float MeshScaleFactor;

    /*------------------------------------------
       �ڲ���������
    -------------------------------------------*/

    // �������� (x,y,z,psi,phi) �������ҹ춥������
    void ComputeRailCoordsFromCenterLine();

    // ���� LeftRailPoints / RightRailPoints ������Ӧ Spline
    void CreateRailSplines();

    // Ϊָ���� Spline ������� SplineMesh
    void GenerateMeshesForSpline(
        USplineComponent* InSpline,
        TArray<USplineMeshComponent*>& OutMeshArray,
        bool bIsLeftRail,
        bool bIsCenterLine
    );

    // ���ݵ�ǰ�켣 + Mesh���������й������
    void UpdateTrackMeshes();
};
