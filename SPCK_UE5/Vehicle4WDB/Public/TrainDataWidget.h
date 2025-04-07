// TrainDataWidget.h
#pragma once
#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Components/TextBlock.h" // 添加UTextBlock的头文件引用
#include "TrainDataWidget.generated.h"
/**
 * 负责显示列车运行信息的 UI Widget
 */
UCLASS()
class VEHICLE4WDB_API UTrainDataWidget : public UUserWidget
{
    GENERATED_BODY()
public:

    // 外部(TrainHUDActor)调用

    // 传入 SPCK 内部仿真时间
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateSPCKTime(float SPCK_time);

    // 传入车体纵向 cm/s 速度
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateSpeed(float SpeedCmS);

    // 里程数值显示
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateTrackS(float InTrackS);

    // 横向 Sperling 指标显示
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateSperlingY(float SperlingY);
    // 垂向 Sperling 指标显示
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateSperlingZ(float SperlingZ);

    // 左前车轮脱轨系数显示
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateDerailment_w1(float Derailment_w1);
    // 右前车轮脱轨系数显示
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateDerailment_w2(float Derailment_w2);

    /*
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdatePower_w1(float Power_w1);
    // 右前电机功率显示
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdatePower_w2(float Power_w2);   
    */
    // 左前电机功率显示


protected:
    virtual void NativeConstruct() override;

    // 是否使用 m/s，否则用 km/h
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TrainData")
    bool bUseMPS;

    // 依次定义TextBlock,绑定到UMG中的TextBlock，名称要对应Designer中的控件名字
     // 内部时间的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* SPCKTime_TextBlock;

    // 纵向速度的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* Speed_TextBlock; 

    // 纵向里程的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* TrackS_TextBlock;

    // 横向 Sperling 指标的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* SperlingY_TextBlock;
    // 垂向 Sperling 指标的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* SperlingZ_TextBlock;

    // 左前车轮脱轨系数的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* Derailment_w1_TextBlock;
    // 右前车轮脱轨系数的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* Derailment_w2_TextBlock;

    /*
    // 左前电机功率的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* Power_w1_TextBlock;
    // 右前电机功率的TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* Power_w2_TextBlock;
    */

private:
    // 将cm/s转换为显示单位( km/h 或 m/s )
    float ConvertSpeed(float SpeedCmS) const;
};