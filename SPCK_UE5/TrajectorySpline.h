// TrajectorySpline.h

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SplineComponent.h"
#include "Components/SplineMeshComponent.h"
#include "TrajectorySpline.generated.h"

/**
 * ATrajectorySpline:
 *  从 JSON 仅读取中心线 {s, x, y, z, psi, phi}，
 *  在 C++ 端计算左右轨坐标（考虑超高 & 轨距微调 & 轨底坡），
 *  最后用SplineMesh可视化三条轨道(或仅两条)。
 */
UCLASS()
class VEHICLE4WDB_API ATrajectorySpline : public AActor
{
    GENERATED_BODY()

public:
    ATrajectorySpline();

    // 从JSON文件加载轨迹数据（忽略 left_rail / right_rail）
    UFUNCTION(BlueprintCallable, Category = "Trajectory")
    bool LoadTrajectoryFromJSON(const FString& FilePath);

    // 创建左轨、右轨样条
    UFUNCTION(BlueprintCallable, Category = "Trajectory")
    void CreateRailSplines();

    // 使用静态网格生成(中心+左右)轨道
    UFUNCTION(BlueprintCallable, Category = "Trajectory")
    void GenerateTrackWithMesh(UStaticMesh* TrackMesh, float MeshScale = 1.0f);

    // 更新/重生成 所有轨道网格
    UFUNCTION(BlueprintCallable, Category = "Trajectory")
    void UpdateTrackMeshes();

    // 获取中心线Spline
    UFUNCTION(BlueprintCallable, Category = "Trajectory")
    USplineComponent* GetSplineComponent() const { return TrajectorySpline; }

protected:
    virtual void BeginPlay() override;

    /** 轨底坡(度)，如1:40约=1.432度; 在UE编辑器中可调 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    float RailBaseCantDeg;

    /** 半轨距微调量(单位 cm); 默认0 => b_ref=150 cm */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    float DeltaHalfGaugeCm;

    /**
     * 是否显示中心线网格（现实中并没有中心钢轨，但在开发调试中有时想看）
     * 如果为false，则仅显示左/右轨。
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    bool bRenderCenterLine;

    // ============ 轨迹样条组件 =============
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USplineComponent* TrajectorySpline;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USplineComponent* LeftRailSpline;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USplineComponent* RightRailSpline;

    // ============ 中心线轨迹数据 =============
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> SValues;   // cm (可能暂时用不到)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> XValues;   // cm
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> YValues;   // cm
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> ZValues;   // cm
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> PsiValues; // rad
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> PhiValues; // rad

    // =========== C++中计算的左右轨顶面坐标 =============
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<FVector> LeftRailPoints;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<FVector> RightRailPoints;

    // ============ 存储SplineMesh组件数组 =============
    UPROPERTY()
    TArray<USplineMeshComponent*> CenterTrackMeshes;
    UPROPERTY()
    TArray<USplineMeshComponent*> LeftTrackMeshes;
    UPROPERTY()
    TArray<USplineMeshComponent*> RightTrackMeshes;

    // 当前选用的网格 & 缩放
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Track")
    UStaticMesh* CurrentTrackMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Track")
    float MeshScaleFactor;

    // ============ 内部辅助函数 =============
    /** 基于中心线重新计算左右轨, 考虑超高/轨距微调 */
    void ComputeRailCoordsFromCenterLine();

    /** 生成某条Spline上的SplineMeshComponents */
    void GenerateMeshesForSpline(
        USplineComponent* InSpline,
        TArray<USplineMeshComponent*>& OutMeshArray,
        bool bIsLeftRail,
        bool bIsCenterLine
    );
};
