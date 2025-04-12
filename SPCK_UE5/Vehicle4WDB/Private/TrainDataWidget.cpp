// TrainDataWidget.cpp

/*

������������У��½� �� User Interface �� Widget Blueprint���ڵ�����ѡ�̳� UserWidget�����ú��ٰѸ���ĳ� UTrainDataWidget��

������µ� UMG ��ͼ���������� BP_TrainDataWidget����

�� Designer ���ϸ� ��TextBlock�� ������

������ ��Details�� ��������Ϊ Speed_TextBlock������� UPROPERTY(meta=(BindWidget)) �е�����һ�£���

���������С��λ�õȡ�

���桢����

*/

#include "TrainDataWidget.h"
#include "Components/TextBlock.h"   // ��Ҫ����TextBlock

void UTrainDataWidget::NativeConstruct()
{
    Super::NativeConstruct();

    // Ĭ���� km/h�����ڱ༭���ﹴѡ bUseMPS
    bUseMPS = false;

    if (!Speed_TextBlock)
    {
        UE_LOG(LogTemp, Warning, TEXT("UTrainDataWidget::NativeConstruct: Speed_TextBlock not bound or named incorrectly in UMG"));
    }
}

// ���ζ����ı����·�ʽ
// ���� HUD ʱ����ʾ
void UTrainDataWidget::UpdateSPCKTime(float SPCK_time)
{
    // 1) ʹ����λС����ʽ���ַ���
    FString TimeString = FString::Printf(TEXT("%.3f"), SPCK_time);

    // 2) ������� SPCKTime_TextBlock���������ı�
    if (SPCKTime_TextBlock)
    {
        SPCKTime_TextBlock->SetText(FText::FromString(TimeString));
    }
}

// ���� HUD �ٶ���ʾ
void UTrainDataWidget::UpdateSpeed(float SpeedCmS)
{
    // 1) ���ٶ�ת��Ҫ��ʾ�ĵ�λ����ʱ DisplaySpeed ���� km/h �� m/s ��
    float DisplaySpeed = ConvertSpeed(SpeedCmS);

    // 2) ʹ����λС����ʽ���ַ���
    FString SpeedString = FString::Printf(TEXT("%.3f"), DisplaySpeed);

    // 3) ������� Speed_TextBlock���������ı�
    if (Speed_TextBlock)
    {
        Speed_TextBlock->SetText(FText::FromString(SpeedString));
    }
}

// ���� HUD �����ʾ
void UTrainDataWidget::UpdateTrackS(float InTrackS)
{
    // ������� SPCK �ı�׼��λ��(m)
    FString TrackSString = FString::Printf( TEXT("%.3f"), InTrackS / 1000 );  // ��ʽ����λΪ km

    if (TrackS_TextBlock)
    {
        TrackS_TextBlock->SetText(FText::FromString(TrackSString));
    }
}

// ���� HUD ���� Sperling ָ��
void UTrainDataWidget::UpdateSperlingY(float SperlingY)
{
    FString SperlingY_String = FString::Printf(TEXT("%.3f"), SperlingY);  // ��ʾ3λС��

    if (SperlingY_TextBlock)
    {
        SperlingY_TextBlock->SetText(FText::FromString(SperlingY_String));
    }
}

// ���� HUD ���� Sperling ָ��
void UTrainDataWidget::UpdateSperlingZ(float SperlingZ)
{
    FString SperlingZ_String = FString::Printf(TEXT("%.3f"), SperlingZ);  // ��ʾ3λС��

    if (SperlingZ_TextBlock)
    {
        SperlingZ_TextBlock->SetText(FText::FromString(SperlingZ_String));
    }
}

// ���� HUD ��ǰ�����ѹ�ϵ��
void UTrainDataWidget::UpdateDerailment_w1(float Derailment_w1)
{
    FString Derailment_w1_String = FString::Printf(TEXT("%.3f"), abs(Derailment_w1));  // ��ʾ3λС��

    if (Derailment_w1_TextBlock)
    {
        Derailment_w1_TextBlock->SetText(FText::FromString(Derailment_w1_String));
    }
}

// ���� HUD ��ǰ�����ѹ�ϵ��
void UTrainDataWidget::UpdateDerailment_w2(float Derailment_w2)
{
    FString Derailment_w2_String = FString::Printf(TEXT("%.3f"), abs(Derailment_w2));  // ��ʾ2λС��

    if (Derailment_w2_TextBlock)
    {
        Derailment_w2_TextBlock->SetText(FText::FromString(Derailment_w2_String));
    }
}

/*
// ���� HUD ��ǰ�������
void UTrainDataWidget::UpdatePower_w1(float Power_w1)
{
    FString Power_w1_String = FString::Printf(TEXT("%.2f"), Power_w1/1000);  // ��ʾ2λС������λΪ kW

    if (Power_w1_TextBlock)
    {
        Power_w1_TextBlock->SetText(FText::FromString(Power_w1_String));
    }
}

// ���� HUD ��ǰ�������
void UTrainDataWidget::UpdatePower_w2(float Power_w2)
{
    FString Power_w2_String = FString::Printf(TEXT("%.2f"), Power_w2 / 1000);  // ��ʾ2λС������λΪ kW

    if (Power_w2_TextBlock)
    {
        Power_w2_TextBlock->SetText(FText::FromString(Power_w2_String));
    }
}

*/

// ���Ͼ�Ϊ HUD Widget ��������
// ����Ϊ�ٶȵ����⴦����ʾ��λ��ѡ��
float UTrainDataWidget::ConvertSpeed(float SpeedCmS) const
{
    float AbsVal = FMath::Abs(SpeedCmS);
    float DistanceScale = 100.f;
    // cm/s -> km/h: 0.036
    //return bUseMPS ? ( AbsVal * DistanceScale ) : (AbsVal * DistanceScale * 0.036f);
    if (bUseMPS)
    {
        // �� UE5 �� cm/s �ָ�Ϊ m/s
        return AbsVal * DistanceScale;
    }
    else
    {
        return AbsVal * DistanceScale * 0.036f;
    }
}
