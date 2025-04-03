// File: TrainDataWidget.cpp

/*

在内容浏览器中，新建 → User Interface → Widget Blueprint，在弹窗中选继承 UserWidget，建好后再把父类改成 UTrainDataWidget：

打开这个新的 UMG 蓝图（比如命名 BP_TrainDataWidget）；

在 Designer 里拖个 “TextBlock” 到画布

将它在 “Details” 里重命名为 Speed_TextBlock（必须和 UPROPERTY(meta=(BindWidget)) 中的名字一致）。

调整字体大小、位置等。

保存、编译

*/

#include "TrainDataWidget.h"
#include "Components/TextBlock.h"   // 需要操作TextBlock

void UTrainDataWidget::NativeConstruct()
{
    Super::NativeConstruct();

    // 默认用 km/h，可在编辑器里勾选 bUseMPS
    bUseMPS = false;

    if (!Speed_TextBlock)
    {
        UE_LOG(LogTemp, Warning, TEXT("UTrainDataWidget::NativeConstruct: Speed_TextBlock not bound or named incorrectly in UMG"));
    }
}

void UTrainDataWidget::UpdateSpeed(float SpeedCmS)
{
    // 1) 将速度转成要显示的单位（此时 DisplaySpeed 已是 km/h 或 m/s ）
    float DisplaySpeed = ConvertSpeed(SpeedCmS);

    // 2) 使用三位小数格式化字符串
    FString SpeedString = FString::Printf(TEXT("%.3f"), DisplaySpeed);

    // 3) 如果存在 Speed_TextBlock，就设置文本
    if (Speed_TextBlock)
    {
        Speed_TextBlock->SetText(FText::FromString(SpeedString));
    }
}

// （新增的更新里程函数）
void UTrainDataWidget::UpdateTrackS(float InTrackS)
{
    // 假设该值单位是米，可直接显示或再格式化
    // 此处修改单位为 km
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


