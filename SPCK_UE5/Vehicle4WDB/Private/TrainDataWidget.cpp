// File: TrainDataWidget.cpp

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

// �������ĸ�����̺�����
void UTrainDataWidget::UpdateTrackS(float InTrackS)
{
    // �����ֵ��λ���ף���ֱ����ʾ���ٸ�ʽ��
    // �˴��޸ĵ�λΪ km
    FString TrackSString = FString::Printf( TEXT("%.3f"), InTrackS / 1000 );

    if (TrackS_TextBlock)
    {
        TrackS_TextBlock->SetText(FText::FromString(TrackSString));
    }
}

float UTrainDataWidget::ConvertSpeed(float SpeedCmS) const
{
    float AbsVal = FMath::Abs(SpeedCmS);
    float DistanceScale = 100.f;
    // cm/s -> km/h: 0.036
    return bUseMPS ? ( AbsVal * DistanceScale ) : (AbsVal * DistanceScale * 0.036f);
}


