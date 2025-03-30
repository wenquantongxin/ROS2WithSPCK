// TrajectorySpline.cpp

#include "TrajectorySpline.h"
#include "Misc/FileHelper.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Components/SplineMeshComponent.h"

static constexpr float RadToDeg = 57.2957795f; // ���ȡ���
static constexpr float DistanceScale = 100.f;       // �ס����ף�1m = 100cm

ATrajectorySpline::ATrajectorySpline()
{
    PrimaryActorTick.bCanEverTick = false;

    // ���������
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

    // ������
    TrajectorySpline = CreateDefaultSubobject<USplineComponent>(TEXT("TrajectorySpline"));
    TrajectorySpline->SetupAttachment(RootComponent);

    // ���
    LeftRailSpline = CreateDefaultSubobject<USplineComponent>(TEXT("LeftRailSpline"));
    LeftRailSpline->SetupAttachment(RootComponent);

    // �ҹ�
    RightRailSpline = CreateDefaultSubobject<USplineComponent>(TEXT("RightRailSpline"));
    RightRailSpline->SetupAttachment(RootComponent);

    // Ĭ�ϲ���
    CurrentTrackMesh = nullptr;
    MeshScaleFactor = 1.0f;
    RailBaseCantDeg = 1.432f; // ��Լ 1:40 �����
    DeltaHalfGaugeCm = 0.f;    // ���΢��
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

    // ��վ�����
    SValues.Empty();
    XValues.Empty();
    YValues.Empty();
    ZValues.Empty();
    PsiValues.Empty();
    PhiValues.Empty();
    SlopeValues.Empty();
    LeftRailPoints.Empty();
    RightRailPoints.Empty();

    // �� JSON ��ȡ����
    const TArray<TSharedPtr<FJsonValue>>* SArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* XArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* YArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* ZArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* PsiArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* PhiArray = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* SlopeArr = nullptr; // slope ���ܴ���

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

    // slope �ֶ����������ȡ
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

    // �� slope ���ڣ������ҲҪ��֤һ��
    if (SlopeArr && SlopeArr->Num() < NumPoints)
    {
        // ��� slope �����������٣����� slopeArr->Num() Ϊ׼
        NumPoints = FMath::Min(NumPoints, SlopeArr->Num());
    }

    if (NumPoints < 2)
    {
        UE_LOG(LogTemp, Error, TEXT("Not enough points in JSON to form a spline."));
        return false;
    }

    // ===================
    // 1) ������ SIMPACK ���� & ��̬
    // 2) ������ϵ��ת: (X, Y, Z)UE = ( X, Y, -Z )SIM
    // 3) �������(psi)���ŷ�ת: psiUE = -psiSIM
    // 4) �����(phi)�����������ȡ����ȡ������ʾ������phiUE = phiSIM
    // 5) slope ���У��� slopeUE = -slopeSIM ����Ϊ Z �ᷭת��
    // 6) ��λ m->cm
    // ===================
    for (int32 i = 0; i < NumPoints; i++)
    {
        float sVal_m = (*SArray)[i]->AsNumber(); // ���(��)
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

        // ת�� UE5 ����ϵ
        float xUE_cm = xVal_m * DistanceScale; // X ����
        float yUE_cm = yVal_m * DistanceScale; // Y ����
        float zUE_cm = -zVal_m * DistanceScale; // ��ת Z

        float psiUE = -psiSim;  // ����Ƿ�ת
        float phiUE = phiSim;  // �ɸ�Ϊ -phiSim, �����/�ڹ�����
        float slopeUE = -slopeSim; // �����¶ȷ�ת

        // д����������
        SValues.Add(sVal_m * DistanceScale);   // ���(���Ŵ󣬷�ת������)
        XValues.Add(xUE_cm);
        YValues.Add(yUE_cm);
        ZValues.Add(zUE_cm);
        PsiValues.Add(psiUE);
        PhiValues.Add(phiUE);
        SlopeValues.Add(slopeUE);
    }

    // �������ҹ�����
    ComputeRailCoordsFromCenterLine();

    // ���������� Spline
    TrajectorySpline->ClearSplinePoints(false);

    const int32 NumPointsFinal = XValues.Num();
    for (int32 i = 0; i < NumPointsFinal; i++)
    {
        FVector Loc(XValues[i], YValues[i], ZValues[i]);
        TrajectorySpline->AddSplinePoint(Loc, ESplineCoordinateSpace::World, false);
    }

    // ����Ҫ����ÿ���������/��̬���������ﴦ��
    // ����Ҫ yaw+pitch+roll ����ϣ��˴� yaw = psiUE, roll = phiUE, pitch = arctan(slopeUE)?
    // UE4/5 ����Ҫ�� SplinePoint ���á��ֲ���ת�����ɲο����м򻯣�
    for (int32 i = 0; i < NumPointsFinal; i++)
    {
        float psi = (i < PsiValues.Num()) ? PsiValues[i] : 0.f;
        float slope = (i < SlopeValues.Num()) ? SlopeValues[i] : 0.f;

        // ���� pitch = atan(slope)
        float pitchRad = 0.f;
        if (bUseSlopeForSplineTangent)
        {
            pitchRad = FMath::Atan(slope);
        }

        // ����һ���������� (����+����)
        // yaw = psi, pitch = pitchRad, roll = 0 (�ݲ��������ϼ�roll)
        // ����ŷ���任 -> ��������
        FRotator RotDeg(FMath::RadiansToDegrees(pitchRad),
            FMath::RadiansToDegrees(psi),
            0.f);
        FVector Dir = RotDeg.Vector(); // (X=cosP cosY, Y=cosP sinY, Z=sinP)

        // UE Spline Tangent ������ռ��¿�ֱ������
        // ע�⣺��Ҫ�� Tangent ����̫�󣬷���ᱻ����mesh���쳤��
        // ����ֻ������λ�������� * 100(������)
        FVector TangentWS = Dir * 100.f;

        TrajectorySpline->SetTangentAtSplinePoint(i, TangentWS, ESplineCoordinateSpace::World, false);
    }

    TrajectorySpline->UpdateSpline();

    // �������ҹ� Spline
    CreateRailSplines();

    // ������� Mesh ���ã�����ӻ�
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

    // �趨�������� = 75 cm (���150cm��һ��) + �ɵ��ڵ�DeltaHalfGaugeCm
    float halfGauge = 75.f + DeltaHalfGaugeCm;

    for (int32 i = 0; i < Num; i++)
    {
        // ���ĵ�
        float xC = XValues[i];
        float yC = YValues[i];
        float zC = ZValues[i];

        // ============��1���� XY ƽ���ȡ�����ߵ�ǰ������============
        // �����ڵ���:
        float dx = 0.f;
        float dy = 0.f;

        if (i == 0 && Num > 1)
        {
            // ��1���㣬�ú�һ����������
            dx = XValues[i + 1] - XValues[i];
            dy = YValues[i + 1] - YValues[i];
        }
        else
        {
            // �����㣬��ǰһ����
            // ��Ȼ����Ҳ���Զ��е� i-1/i+1 ��˫�߲�ֵ����һ����ôд�͹�����
            dx = XValues[i] - XValues[i - 1];
            dy = YValues[i] - YValues[i - 1];
        }

        // ��dx,dyȫΪ0(�������), ��������
        if (FMath::IsNearlyZero(dx) && FMath::IsNearlyZero(dy))
        {
            dx = 1.f; // Ĭ�ϸ�������
            dy = 0.f;
        }

        // ����������͵�λ����
        float len = FMath::Sqrt(dx * dx + dy * dy);
        float nx = -dy / len; // ����
        float ny = dx / len; // ����
        // �ҷ��߾��� -(nx, ny)��Ҳ����������ת

        // ============��2�����߹�ת������Z̧��============ 
        // ��֮ǰ�� code ��ʹ�� phiUE = PhiValues[i] -> zShift = halfGauge * sin(phiUE)
        // ������Լ�����:
        float phiUE = (i < PhiValues.Num()) ? PhiValues[i] : 0.f;
        float zShift = halfGauge * FMath::Sin(phiUE);

        // ============��3���������ҹ�����============
        // ������ C
        FVector centerPos(xC, yC, zC);

        // ���: centerPos + halfGauge*(nx, ny) + zShift
        FVector L = centerPos;
        L.X += halfGauge * nx;
        L.Y += halfGauge * ny;
        L.Z -= zShift;

        // �ҹ�: centerPos - halfGauge*(nx, ny) - zShift
        FVector R = centerPos;
        R.X -= halfGauge * nx;
        R.Y -= halfGauge * ny;
        R.Z += zShift;

        // ����
        LeftRailPoints.Add(L);
        RightRailPoints.Add(R);
    }
}


void ATrajectorySpline::CreateRailSplines()
{
    // ���
    LeftRailSpline->ClearSplinePoints(false);
    for (const FVector& Pt : LeftRailPoints)
    {
        LeftRailSpline->AddSplinePoint(Pt, ESplineCoordinateSpace::World, false);
    }
    LeftRailSpline->UpdateSpline();

    // �ҹ�
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

    // ���پɵ�Mesh
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

    // ������
    if (bRenderCenterLine)
    {
        GenerateMeshesForSpline(TrajectorySpline, CenterTrackMeshes,
            /*bIsLeftRail=*/false, /*bIsCenterLine=*/true);
    }

    // ���
    GenerateMeshesForSpline(LeftRailSpline, LeftTrackMeshes,
        /*bIsLeftRail=*/true, /*bIsCenterLine=*/false);

    // �ҹ�
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

        // ���öε���β
        SplineMesh->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent, true);

        // ָ��Up����(����Z)
        SplineMesh->SetSplineUpDir(FVector::UpVector, false);

        // ����������ѡȡƽ��phi
        float StartPhi = (i < PhiValues.Num()) ? PhiValues[i] : 0.f;
        float EndPhi = ((i + 1) < PhiValues.Num()) ? PhiValues[i + 1] : 0.f;
        float avgPhiRad = 0.5f * (StartPhi + EndPhi);
        float avgPhiDeg = avgPhiRad * RadToDeg;

        float finalRollDeg = 0.f;
        if (bIsCenterLine)
        {
            // ������ֱ����phi
            finalRollDeg = avgPhiDeg;
        }
        else
        {
            // �����(��)�볬�ߵ��ӣ�������� => phi - baseCant���ҹ����� => phi + baseCant
            // ������Ҫ�ɷ�ת
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

        // ����Mesh����
        FVector2D Scale2D(MeshScaleFactor, MeshScaleFactor);
        SplineMesh->SetStartScale(Scale2D);
        SplineMesh->SetEndScale(Scale2D);

        // ע��
        SplineMesh->RegisterComponent();
        OutMeshArray.Add(SplineMesh);
    }
}
