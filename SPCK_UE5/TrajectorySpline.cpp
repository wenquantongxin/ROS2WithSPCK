/*
 * TrajectorySpline.cpp
 *
 * 功能说明：
 *  1) 从 JSON 文件读取轨道中心线信息（s, x, y, z, psi, phi），并转化成 UE 世界坐标（单位由米→厘米）。
 *  2) 在 C++ 端根据中心线计算左右钢轨的坐标（考虑轨距和超高）。
 *  3) 利用 USplineComponent 构建三条样条（中心线、左轨、右轨），可视化为 SplineMesh。
 *
 * 主要实现：
 *  - LoadTrajectoryFromJSON(const FString& FilePath):
 *      读取 JSON 中的轨迹点数据，存入数组。再调用 ComputeRailCoordsFromCenterLine() 计算左右轨道。
 *      随后将中心线坐标添加到 TrajectorySpline 并更新样条；最后(若已设定 Mesh)，调用 UpdateTrackMeshes() 可视化。
 *  - ComputeRailCoordsFromCenterLine():
 *      针对每个中心线点 (x, y, z, psi, phi)，计算左/右钢轨的 3D 坐标。此处通过在 XY 平面向左右偏移轨距，并在 Z 方向根据超高做差值。
 *      为适配右/左手系差异和超高翻转，本代码示例里对 zShift 在左轨、右轨进行了符号调换，使右转弯时左轨外侧更高。
 *  - GenerateTrackWithMesh(...) / UpdateTrackMeshes():
 *      从三条 Spline(中心/左/右) 逐段生成 USplineMeshComponent，赋予同一 Mesh，设置 Roll(轨底坡 + 超高) 和 UpDir 等参数。
 *      这样可以在 Unreal 中实现连续的轨道网格外观。
 *
 * 注意事项：
 *  - 若需要完整三自由度姿态（包括俯仰 Pitch），当前示例仅对 psi(偏航) + phi(横滚) 做了简化处理，俯仰角尚未显式纳入。
 *  - 若外部坐标系与 UE 坐标系在轴向或正负方向上有冲突，需要在数据导入后统一处理（翻转 Y、翻转 psi、或调整 phi 符号等）。
 *  - RailBaseCantDeg 用于设置轨底坡（如 1:40 ≈ 1.432°）在可视化时可加到超高滚转上，让钢轨进一步“向内倾斜”。
 */

#include "TrajectorySpline.h"
#include "Misc/FileHelper.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Components/SplineMeshComponent.h"

// 若需把弧度->度
static constexpr float RadToDeg = 57.2957795f;
// Unreal Engine 中 1 个单位 = 1 cm，这里将轨道数据(单位:米)放大 100 倍
static constexpr float DistanceScale = 100.f;

ATrajectorySpline::ATrajectorySpline()
{
    PrimaryActorTick.bCanEverTick = false;

    // 创建根组件
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

    // 中心线
    TrajectorySpline = CreateDefaultSubobject<USplineComponent>(TEXT("TrajectorySpline"));
    TrajectorySpline->SetupAttachment(RootComponent);

    // 左/右轨道
    LeftRailSpline = CreateDefaultSubobject<USplineComponent>(TEXT("LeftRailSpline"));
    LeftRailSpline->SetupAttachment(RootComponent);

    RightRailSpline = CreateDefaultSubobject<USplineComponent>(TEXT("RightRailSpline"));
    RightRailSpline->SetupAttachment(RootComponent);

    // 默认值
    CurrentTrackMesh = nullptr;
    MeshScaleFactor = 1.0f;

    RailBaseCantDeg = 1.432f; // ~1:40
    DeltaHalfGaugeCm = 0.0f;
    bRenderCenterLine = true; // 是否可视化中心线
}

void ATrajectorySpline::BeginPlay()
{
    Super::BeginPlay();
}

bool ATrajectorySpline::LoadTrajectoryFromJSON(const FString& FilePath)
{
    FString JsonString;
    if (!FFileHelper::LoadFileToString(JsonString, *FilePath))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to load JSON file: %s"), *FilePath);
        return false;
    }

    TSharedPtr<FJsonObject> JsonObject;
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
    if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to parse JSON data: %s"), *FilePath);
        return false;
    }

    // 清空旧数据
    SValues.Empty();
    XValues.Empty();
    YValues.Empty();
    ZValues.Empty();
    PsiValues.Empty();
    PhiValues.Empty();
    LeftRailPoints.Empty();
    RightRailPoints.Empty();

    // 读取 s, x, y, z, psi, phi
    const TArray<TSharedPtr<FJsonValue>>* SArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* XArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* YArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* ZArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* PsiArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* PhiArray = nullptr;

    bool bOk = (
        JsonObject->TryGetArrayField(TEXT("s"), SArray) &&
        JsonObject->TryGetArrayField(TEXT("x"), XArray) &&
        JsonObject->TryGetArrayField(TEXT("y"), YArray) &&
        JsonObject->TryGetArrayField(TEXT("z"), ZArray) &&
        JsonObject->TryGetArrayField(TEXT("psi"), PsiArray) &&
        JsonObject->TryGetArrayField(TEXT("phi"), PhiArray)
        );
    if (!bOk)
    {
        UE_LOG(LogTemp, Error, TEXT("JSON missing required arrays: s, x, y, z, psi, phi."));
        return false;
    }

    int32 NumPoints = FMath::Min3(SArray->Num(), XArray->Num(), YArray->Num());
    NumPoints = FMath::Min(NumPoints, ZArray->Num());
    NumPoints = FMath::Min(NumPoints, PsiArray->Num());
    NumPoints = FMath::Min(NumPoints, PhiArray->Num());
    if (NumPoints < 2)
    {
        UE_LOG(LogTemp, Error, TEXT("Not enough points in the JSON to form a spline."));
        return false;
    }

    // 填充 & 单位换成 cm
    for (int32 i = 0; i < NumPoints; i++)
    {
        float sVal = (*SArray)[i]->AsNumber();
        float xVal_m = (*XArray)[i]->AsNumber();
        float yVal_m = (*YArray)[i]->AsNumber();
        float zVal_m = (*ZArray)[i]->AsNumber();
        float psiRad = (*PsiArray)[i]->AsNumber();
        float phiRad = (*PhiArray)[i]->AsNumber();

        SValues.Add(sVal * DistanceScale);   // cm
        XValues.Add(xVal_m * DistanceScale); // cm
        YValues.Add(yVal_m * DistanceScale); // cm
        ZValues.Add(zVal_m * DistanceScale); // cm
        PsiValues.Add(psiRad);  // rad
        PhiValues.Add(phiRad);  // rad
    }

    // ----------------------
    // 忽略 JSON 中的 left_rail/right_rail
    // 直接在 C++ 里推算左右轨
    // ----------------------
    ComputeRailCoordsFromCenterLine();

    // 构建“中心线”样条
    TrajectorySpline->ClearSplinePoints(false);
    for (int32 i = 0; i < NumPoints; i++)
    {
        FVector Loc(XValues[i], YValues[i], ZValues[i]);
        TrajectorySpline->AddSplinePoint(Loc, ESplineCoordinateSpace::World, false);

        // 可选：若要设置每点的切线(仅 psi+phi, 忽略 pitch)
        if (i < PsiValues.Num() && i < PhiValues.Num())
        {
            float psi = PsiValues[i];
            float phi = PhiValues[i];
            // direction (简化2D + roll)
            FVector TangentDir(
                FMath::Cos(phi) * FMath::Cos(psi),
                FMath::Cos(phi) * FMath::Sin(psi),
                FMath::Sin(phi)
            );
            // 不再乘100，否则会出现巨大的拉伸
            TrajectorySpline->SetTangentAtSplinePoint(i, TangentDir, ESplineCoordinateSpace::World, false);
        }
    }
    TrajectorySpline->UpdateSpline();

    // 构建左右轨样条
    CreateRailSplines();

    // 如果已经设置了网格，则更新网格显示
    if (CurrentTrackMesh != nullptr)
    {
        UpdateTrackMeshes();
    }

    return true;
}

void ATrajectorySpline::ComputeRailCoordsFromCenterLine()
{
    LeftRailPoints.Empty();
    RightRailPoints.Empty();

    // 钢轨轨距(示例设置) = 150 cm，可根据需要改为 143.5 cm
    float BaseHalfGaugeCm = 0.5f * 150.0f;

    float ActualHalfGaugeCm = BaseHalfGaugeCm + DeltaHalfGaugeCm;

    const int32 Num = XValues.Num();
    LeftRailPoints.Reserve(Num);
    RightRailPoints.Reserve(Num);

    for (int32 i = 0; i < Num; i++)
    {
        FVector C(XValues[i], YValues[i], ZValues[i]);
        float psi = (i < PsiValues.Num()) ? PsiValues[i] : 0.f;
        float phi = (i < PhiValues.Num()) ? PhiValues[i] : 0.f;

        // 在XY平面上, psi 是前进方向; 轨道横向方向 => ( -sin(psi), cos(psi) )
        float sinPsi = FMath::Sin(psi);
        float cosPsi = FMath::Cos(psi);

        // 超高导致的竖向抬升(单位: cm)
        float zShift = ActualHalfGaugeCm * FMath::Sin(phi);

        // 左轨: +halfGauge => 
        //  dx = + halfGauge * (-sinPsi); dy = + halfGauge*(cosPsi)
        //  此处把 Z - zShift, 让左轨成为外轨(弯曲时更高)
        FVector L = C;
        L.X += (+ActualHalfGaugeCm * (-sinPsi));
        L.Y += (+ActualHalfGaugeCm * (cosPsi));
        L.Z -= zShift;  // 与之前相反，左轨提高可通过额外逻辑调试

        // 右轨: -halfGauge
        FVector R = C;
        R.X += (-ActualHalfGaugeCm * (-sinPsi));
        R.Y += (-ActualHalfGaugeCm * (cosPsi));
        R.Z += zShift;  // 右轨相对更低(或不抬升)

        LeftRailPoints.Add(L);
        RightRailPoints.Add(R);
    }
}

void ATrajectorySpline::CreateRailSplines()
{
    // 左轨
    LeftRailSpline->ClearSplinePoints(false);
    for (const FVector& Pt : LeftRailPoints)
    {
        LeftRailSpline->AddSplinePoint(Pt, ESplineCoordinateSpace::World, false);
    }
    LeftRailSpline->UpdateSpline();

    // 右轨
    RightRailSpline->ClearSplinePoints(false);
    for (const FVector& Pt : RightRailPoints)
    {
        RightRailSpline->AddSplinePoint(Pt, ESplineCoordinateSpace::World, false);
    }
    RightRailSpline->UpdateSpline();
}

void ATrajectorySpline::GenerateTrackWithMesh(UStaticMesh* TrackMesh, float MeshScale)
{
    if (!TrackMesh)
    {
        UE_LOG(LogTemp, Error, TEXT("Invalid TrackMesh!"));
        return;
    }
    CurrentTrackMesh = TrackMesh;
    MeshScaleFactor = MeshScale;

    UpdateTrackMeshes();
}

void ATrajectorySpline::UpdateTrackMeshes()
{
    if (!CurrentTrackMesh)
    {
        return;
    }

    // 先清除已有 Mesh
    for (USplineMeshComponent* MeshComp : CenterTrackMeshes)
    {
        if (MeshComp) MeshComp->DestroyComponent();
    }
    CenterTrackMeshes.Empty();

    for (USplineMeshComponent* MeshComp : LeftTrackMeshes)
    {
        if (MeshComp) MeshComp->DestroyComponent();
    }
    LeftTrackMeshes.Empty();

    for (USplineMeshComponent* MeshComp : RightTrackMeshes)
    {
        if (MeshComp) MeshComp->DestroyComponent();
    }
    RightTrackMeshes.Empty();

    // 如果勾选了渲染中心线，则生成
    if (bRenderCenterLine)
    {
        GenerateMeshesForSpline(TrajectorySpline, CenterTrackMeshes, /*bIsLeftRail=*/false, /*bIsCenterLine=*/true);
    }

    // 左轨/右轨
    GenerateMeshesForSpline(LeftRailSpline, LeftTrackMeshes,  /*bIsLeftRail=*/true,  /*bIsCenterLine=*/false);
    GenerateMeshesForSpline(RightRailSpline, RightTrackMeshes, /*bIsLeftRail=*/false, /*bIsCenterLine=*/false);
}

void ATrajectorySpline::GenerateMeshesForSpline(
    USplineComponent* InSpline,
    TArray<USplineMeshComponent*>& OutMeshArray,
    bool bIsLeftRail,
    bool bIsCenterLine
)
{
    if (!InSpline)
        return;

    int32 NumPoints = InSpline->GetNumberOfSplinePoints();
    if (NumPoints < 2)
        return;

    for (int32 i = 0; i < NumPoints - 1; i++)
    {
        FVector StartPos, StartTangent, EndPos, EndTangent;
        InSpline->GetLocationAndTangentAtSplinePoint(i, StartPos, StartTangent, ESplineCoordinateSpace::World);
        InSpline->GetLocationAndTangentAtSplinePoint(i + 1, EndPos, EndTangent, ESplineCoordinateSpace::World);

        // 创建新的 SplineMesh
        USplineMeshComponent* SplineMesh = NewObject<USplineMeshComponent>(this);
        SplineMesh->SetMobility(EComponentMobility::Movable);
        SplineMesh->SetupAttachment(RootComponent);
        SplineMesh->SetStaticMesh(CurrentTrackMesh);

        // 设置段的首尾
        SplineMesh->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent, true);

        // 指定 Up 方向(世界 Z)
        SplineMesh->SetSplineUpDir(FVector(0, 0, 1), false);

        // 计算该段平均 phi
        float StartPhi = (i < PhiValues.Num()) ? PhiValues[i] : 0.f;
        float EndPhi = ((i + 1) < PhiValues.Num()) ? PhiValues[i + 1] : 0.f;
        float avgPhiRad = 0.5f * (StartPhi + EndPhi);
        float avgPhiDeg = avgPhiRad * RadToDeg; // 弧度->度

        float finalRollDeg = 0.f;

        if (bIsCenterLine)
        {
            // 若希望中心线也可视化超高倾斜
            finalRollDeg = avgPhiDeg;
        }
        else
        {
            // 左/右轨 => 超高 + 轨底坡(同向内倾斜)
            // 说明: 选定左轨 = phi - baseCant, 右轨 = phi + baseCant （或相反）
            // 具体符号要结合场景观察。若发现可视上是反的，则调换 +/-。
            if (bIsLeftRail)
            {
                // 左轨倾内 => phi - baseCant
                finalRollDeg = avgPhiDeg - RailBaseCantDeg;
            }
            else
            {
                // 右轨倾内 => phi + baseCant
                finalRollDeg = avgPhiDeg + RailBaseCantDeg;
            }
        }

        // 设置滚转角
        float finalRollRad = FMath::DegreesToRadians(finalRollDeg);
        SplineMesh->SetStartRoll(finalRollRad);
        SplineMesh->SetEndRoll(finalRollRad);

        // 设置网格缩放(建议=1.0，避免过大/过小)
        FVector2D Scale2D(MeshScaleFactor, MeshScaleFactor);
        SplineMesh->SetStartScale(Scale2D);
        SplineMesh->SetEndScale(Scale2D);

        // 注册并存储
        SplineMesh->RegisterComponent();
        OutMeshArray.Add(SplineMesh);
    }
}
