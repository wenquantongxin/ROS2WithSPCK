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
 * 功能：
 *  1. 从外部 JSON 文件读取轨迹（中心线：s, x, y, z, psi, phi, slope 等）。
 *  2. 完成坐标系转换（SIMPACK -> UE5）与单位转换（m -> cm）。
 *  3. 在 UE 中构建三条 Spline（中心线、左轨、右轨），并可生成 SplineMesh 网格进行可视化。
 *  4. 可选地将 slope 值转化为俯仰角 Pitch，与 psi (yaw) 和 phi (roll) 一起映射到 Spline 的切线。
 */
UCLASS()
class VEHICLE4WDB_API ATrajectorySpline : public AActor
{
    GENERATED_BODY()

public:
    // 构造函数
    ATrajectorySpline();

    // 在蓝图或C++里可调用：加载外部 JSON 轨迹并转换到 UE5 坐标系
    UFUNCTION(BlueprintCallable, Category = "Trajectory")
    bool LoadTrajectoryFromJSON(const FString& FilePath);

    // 在蓝图或C++里可调用：使用已加载的轨迹数据，生成轨道网格
    UFUNCTION(BlueprintCallable, Category = "Trajectory")
    void GenerateTrackWithMesh(UStaticMesh* TrackMesh, float MeshScale = 1.0f);

protected:
    // 游戏开始时执行
    virtual void BeginPlay() override;

    /*------------------------------------------
       轨道可调参数 (在 UE 编辑器中可配置)
    -------------------------------------------*/

    // 轨底坡(度)，在生成轨道网格时可加到超高滚转中
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    float RailBaseCantDeg;

    // 轨距半径额外微调(单位 cm)，如需把轨距从150 cm改为 143.5 等
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    float DeltaHalfGaugeCm;

    // 是否渲染中心线
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    bool bRenderCenterLine;

    // 是否在 Spline 的 Tangent 中结合 slope 值计算俯仰
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail Params")
    bool bUseSlopeForSplineTangent;

    /*------------------------------------------
       样条组件
    -------------------------------------------*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USplineComponent* TrajectorySpline;  // 中心线

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USplineComponent* LeftRailSpline;    // 左轨线

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
    USplineComponent* RightRailSpline;   // 右轨线

    /*------------------------------------------
       轨迹数据（以 UE5 世界坐标/姿态存储）
    -------------------------------------------*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> SValues;   // 里程 (cm)，仅供参考，可不做翻转

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> XValues;   // x坐标(厘米) [UE世界]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> YValues;   // y坐标(厘米) [UE世界]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> ZValues;   // z坐标(厘米) [UE世界]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> PsiValues; // 航向角 yaw (rad) [UE坐标系下]

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> PhiValues; // 横滚角 roll (rad) [UE坐标系下]

    // 竖向坡度 p(s)=dz/ds，可在生成Spline Tangent时做俯仰
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<float> SlopeValues; // slope (无量纲, rad=? 其实是 dz/ds )

    /*------------------------------------------
       由中心线推算的左右轨坐标
    -------------------------------------------*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<FVector> LeftRailPoints;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Trajectory Data")
    TArray<FVector> RightRailPoints;

    /*------------------------------------------
       存储用于可视化的 SplineMesh Component
    -------------------------------------------*/
    UPROPERTY()
    TArray<USplineMeshComponent*> CenterTrackMeshes;

    UPROPERTY()
    TArray<USplineMeshComponent*> LeftTrackMeshes;

    UPROPERTY()
    TArray<USplineMeshComponent*> RightTrackMeshes;

    /*------------------------------------------
       当前使用的轨道网格资源 & 缩放
    -------------------------------------------*/
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Track")
    UStaticMesh* CurrentTrackMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Track")
    float MeshScaleFactor;

    /*------------------------------------------
       内部辅助函数
    -------------------------------------------*/

    // 从中心线 (x,y,z,psi,phi) 计算左右轨顶面坐标
    void ComputeRailCoordsFromCenterLine();

    // 根据 LeftRailPoints / RightRailPoints 构建对应 Spline
    void CreateRailSplines();

    // 为指定的 Spline 逐段生成 SplineMesh
    void GenerateMeshesForSpline(
        USplineComponent* InSpline,
        TArray<USplineMeshComponent*>& OutMeshArray,
        bool bIsLeftRail,
        bool bIsCenterLine
    );

    // 根据当前轨迹 + Mesh，更新所有轨道网格
    void UpdateTrackMeshes();
};
