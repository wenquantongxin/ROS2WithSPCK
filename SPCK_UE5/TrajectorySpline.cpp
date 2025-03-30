// TrajectorySpline.cpp

#include "TrajectorySpline.h"
#include "Misc/FileHelper.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Components/SplineMeshComponent.h"

static constexpr float RadToDeg = 57.2957795f; // 弧度→度
static constexpr float DistanceScale = 100.f;       // 米→厘米：1m = 100cm

ATrajectorySpline::ATrajectorySpline()
{
    PrimaryActorTick.bCanEverTick = false;

    // 创建根组件
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

    // 中心线
    TrajectorySpline = CreateDefaultSubobject<USplineComponent>(TEXT("TrajectorySpline"));
    TrajectorySpline->SetupAttachment(RootComponent);

    // 左轨
    LeftRailSpline = CreateDefaultSubobject<USplineComponent>(TEXT("LeftRailSpline"));
    LeftRailSpline->SetupAttachment(RootComponent);

    // 右轨
    RightRailSpline = CreateDefaultSubobject<USplineComponent>(TEXT("RightRailSpline"));
    RightRailSpline->SetupAttachment(RootComponent);

    // 默认参数
    CurrentTrackMesh = nullptr;
    MeshScaleFactor = 1.0f;
    RailBaseCantDeg = 1.432f; // 大约 1:40 轨底坡
    DeltaHalfGaugeCm = 0.f;    // 轨距微调
    bRenderCenterLine = true;
    bUseSlopeForSplineTangent = false;
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
    SlopeValues.Empty();
    LeftRailPoints.Empty();
    RightRailPoints.Empty();

    // 从 JSON 读取数组
    const TArray<TSharedPtr<FJsonValue>>* SArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* XArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* YArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* ZArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* PsiArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* PhiArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* SlopeArr = nullptr; // slope 可能存在

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

    // slope 字段若存在则读取
    if (JsonObject->TryGetArrayField(TEXT("slope"), SlopeArr))
    {
        UE_LOG(LogTemp, Log, TEXT("Detected slope array in JSON, will read it."));
    }

    int32 NumPoints = SArray->Num();
    NumPoints = FMath::Min(NumPoints, XArray->Num());
    NumPoints = FMath::Min(NumPoints, YArray->Num());
    NumPoints = FMath::Min(NumPoints, ZArray->Num());
    NumPoints = FMath::Min(NumPoints, PsiArray->Num());
    NumPoints = FMath::Min(NumPoints, PhiArray->Num());

    // 若 slope 存在，则点数也要保证一致
    if (SlopeArr && SlopeArr->Num() < NumPoints)
    {
        // 如果 slope 点数比其它少，则以 slopeArr->Num() 为准
        NumPoints = FMath::Min(NumPoints, SlopeArr->Num());
    }

    if (NumPoints < 2)
    {
        UE_LOG(LogTemp, Error, TEXT("Not enough points in JSON to form a spline."));
        return false;
    }

    // ===================
    // 1) 逐点读出 SIMPACK 坐标 & 姿态
    // 2) 做坐标系翻转: (X, Y, Z)UE = ( X, Y, -Z )SIM
    // 3) 做航向角(psi)符号翻转: psiUE = -psiSIM
    // 4) 横滚角(phi)可视需求决定取正或取负；本示例保留phiUE = phiSIM
    // 5) slope 若有，则 slopeUE = -slopeSIM （因为 Z 轴翻转）
    // 6) 单位 m->cm
    // ===================
    for (int32 i = 0; i < NumPoints; i++)
    {
        float sVal_m = (*SArray)[i]->AsNumber(); // 里程(米)
        float xVal_m = (*XArray)[i]->AsNumber();
        float yVal_m = (*YArray)[i]->AsNumber();
        float zVal_m = (*ZArray)[i]->AsNumber();
        float psiSim = (*PsiArray)[i]->AsNumber(); // rad
        float phiSim = (*PhiArray)[i]->AsNumber(); // rad

        float slopeSim = 0.f;
        if (SlopeArr && i < SlopeArr->Num())
        {
            slopeSim = (*SlopeArr)[i]->AsNumber();
        }

        // 转到 UE5 坐标系
        float xUE_cm = xVal_m * DistanceScale; // X 不变
        float yUE_cm = yVal_m * DistanceScale; // Y 不变
        float zUE_cm = -zVal_m * DistanceScale; // 翻转 Z

        float psiUE = -psiSim;  // 航向角翻转
        float phiUE = phiSim;  // 可改为 -phiSim, 视外轨/内轨需求
        float slopeUE = -slopeSim; // 竖向坡度翻转

        // 写入属性数组
        SValues.Add(sVal_m * DistanceScale);   // 里程(仅放大，翻转无意义)
        XValues.Add(xUE_cm);
        YValues.Add(yUE_cm);
        ZValues.Add(zUE_cm);
        PsiValues.Add(psiUE);
        PhiValues.Add(phiUE);
        SlopeValues.Add(slopeUE);
    }

    // 计算左右轨坐标
    ComputeRailCoordsFromCenterLine();

    // 更新中心线 Spline
    TrajectorySpline->ClearSplinePoints(false);

    const int32 NumPointsFinal = XValues.Num();
    for (int32 i = 0; i < NumPointsFinal; i++)
    {
        FVector Loc(XValues[i], YValues[i], ZValues[i]);
        TrajectorySpline->AddSplinePoint(Loc, ESplineCoordinateSpace::World, false);
    }

    // 若需要设置每个点的切线/姿态，可在这里处理
    // 这需要 yaw+pitch+roll 的组合；此处 yaw = psiUE, roll = phiUE, pitch = arctan(slopeUE)?
    // UE4/5 中若要给 SplinePoint 设置“局部旋转”，可参考下列简化：
    for (int32 i = 0; i < NumPointsFinal; i++)
    {
        float psi = (i < PsiValues.Num()) ? PsiValues[i] : 0.f;
        float slope = (i < SlopeValues.Num()) ? SlopeValues[i] : 0.f;

        // 计算 pitch = atan(slope)
        float pitchRad = 0.f;
        if (bUseSlopeForSplineTangent)
        {
            pitchRad = FMath::Atan(slope);
        }

        // 计算一个方向向量 (航向+俯仰)
        // yaw = psi, pitch = pitchRad, roll = 0 (暂不在切线上加roll)
        // 先算欧拉变换 -> 方向向量
        FRotator RotDeg(FMath::RadiansToDegrees(pitchRad),
            FMath::RadiansToDegrees(psi),
            0.f);
        FVector Dir = RotDeg.Vector(); // (X=cosP cosY, Y=cosP sinY, Z=sinP)

        // UE Spline Tangent 在世界空间下可直接设置
        // 注意：不要把 Tangent 设置太大，否则会被当成mesh拉伸长度
        // 这里只给个单位方向向量 * 100(或任意)
        FVector TangentWS = Dir * 100.f;

        TrajectorySpline->SetTangentAtSplinePoint(i, TangentWS, ESplineCoordinateSpace::World, false);
    }

    TrajectorySpline->UpdateSpline();

    // 创建左右轨 Spline
    CreateRailSplines();

    // 如果已有 Mesh 设置，则可视化
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

    const int32 Num = XValues.Num();
    LeftRailPoints.Reserve(Num);
    RightRailPoints.Reserve(Num);

    // 设定基础半轨距 = 75 cm (轨距150cm的一半) + 可调节的DeltaHalfGaugeCm
    float halfGauge = 75.f + DeltaHalfGaugeCm;

    for (int32 i = 0; i < Num; i++)
    {
        // 中心点
        float xC = XValues[i];
        float yC = YValues[i];
        float zC = ZValues[i];

        // ============【1】在 XY 平面获取中心线的前进方向============
        // 做相邻点差分:
        float dx = 0.f;
        float dy = 0.f;

        if (i == 0 && Num > 1)
        {
            // 第1个点，用后一点差分来近似
            dx = XValues[i + 1] - XValues[i];
            dy = YValues[i + 1] - YValues[i];
        }
        else
        {
            // 其它点，用前一点差分
            // 当然，你也可以对中点 i-1/i+1 做双边插值，但一般这么写就够用了
            dx = XValues[i] - XValues[i - 1];
            dy = YValues[i] - YValues[i - 1];
        }

        // 若dx,dy全为0(极端情况), 做个保护
        if (FMath::IsNearlyZero(dx) && FMath::IsNearlyZero(dy))
        {
            dx = 1.f; // 默认给个方向
            dy = 0.f;
        }

        // 计算轨道方向和单位法线
        float len = FMath::Sqrt(dx * dx + dy * dy);
        float nx = -dy / len; // 左法线
        float ny = dx / len; // 左法线
        // 右法线就是 -(nx, ny)，也可依场景反转

        // ============【2】超高滚转带来的Z抬升============ 
        // 你之前在 code 里使用 phiUE = PhiValues[i] -> zShift = halfGauge * sin(phiUE)
        // 这里可以继续用:
        float phiUE = (i < PhiValues.Num()) ? PhiValues[i] : 0.f;
        float zShift = halfGauge * FMath::Sin(phiUE);

        // ============【3】构造左右轨坐标============
        // 中心线 C
        FVector centerPos(xC, yC, zC);

        // 左轨: centerPos + halfGauge*(nx, ny) + zShift
        FVector L = centerPos;
        L.X += halfGauge * nx;
        L.Y += halfGauge * ny;
        L.Z -= zShift;

        // 右轨: centerPos - halfGauge*(nx, ny) - zShift
        FVector R = centerPos;
        R.X -= halfGauge * nx;
        R.Y -= halfGauge * ny;
        R.Z += zShift;

        // 保存
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

    // 销毁旧的Mesh
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

    // 中心线
    if (bRenderCenterLine)
    {
        GenerateMeshesForSpline(TrajectorySpline, CenterTrackMeshes,
            /*bIsLeftRail=*/false, /*bIsCenterLine=*/true);
    }

    // 左轨
    GenerateMeshesForSpline(LeftRailSpline, LeftTrackMeshes,
        /*bIsLeftRail=*/true, /*bIsCenterLine=*/false);

    // 右轨
    GenerateMeshesForSpline(RightRailSpline, RightTrackMeshes,
        /*bIsLeftRail=*/false, /*bIsCenterLine=*/false);
}

void ATrajectorySpline::GenerateMeshesForSpline(
    USplineComponent* InSpline,
    TArray<USplineMeshComponent*>& OutMeshArray,
    bool bIsLeftRail,
    bool bIsCenterLine
)
{
    if (!InSpline) return;

    int32 NumPoints = InSpline->GetNumberOfSplinePoints();
    if (NumPoints < 2) return;

    for (int32 i = 0; i < NumPoints - 1; i++)
    {
        FVector StartPos, StartTangent, EndPos, EndTangent;
        InSpline->GetLocationAndTangentAtSplinePoint(i, StartPos, StartTangent, ESplineCoordinateSpace::World);
        InSpline->GetLocationAndTangentAtSplinePoint(i + 1, EndPos, EndTangent, ESplineCoordinateSpace::World);

        USplineMeshComponent* SplineMesh = NewObject<USplineMeshComponent>(this);
        SplineMesh->SetMobility(EComponentMobility::Movable);
        SplineMesh->SetupAttachment(RootComponent);
        SplineMesh->SetStaticMesh(CurrentTrackMesh);

        // 设置段的首尾
        SplineMesh->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent, true);

        // 指定Up方向(世界Z)
        SplineMesh->SetSplineUpDir(FVector::UpVector, false);

        // 根据索引，选取平均phi
        float StartPhi = (i < PhiValues.Num()) ? PhiValues[i] : 0.f;
        float EndPhi = ((i + 1) < PhiValues.Num()) ? PhiValues[i + 1] : 0.f;
        float avgPhiRad = 0.5f * (StartPhi + EndPhi);
        float avgPhiDeg = avgPhiRad * RadToDeg;

        float finalRollDeg = 0.f;
        if (bIsCenterLine)
        {
            // 中心线直接用phi
            finalRollDeg = avgPhiDeg;
        }
        else
        {
            // 轨底坡(度)与超高叠加；左轨倾内 => phi - baseCant；右轨倾内 => phi + baseCant
            // 根据需要可反转
            if (bIsLeftRail)
            {
                finalRollDeg = avgPhiDeg - RailBaseCantDeg;
            }
            else
            {
                finalRollDeg = avgPhiDeg + RailBaseCantDeg;
            }
        }
        float finalRollRad = FMath::DegreesToRadians(finalRollDeg);
        SplineMesh->SetStartRoll(finalRollRad);
        SplineMesh->SetEndRoll(finalRollRad);

        // 设置Mesh缩放
        FVector2D Scale2D(MeshScaleFactor, MeshScaleFactor);
        SplineMesh->SetStartScale(Scale2D);
        SplineMesh->SetEndScale(Scale2D);

        // 注册
        SplineMesh->RegisterComponent();
        OutMeshArray.Add(SplineMesh);
    }
}
