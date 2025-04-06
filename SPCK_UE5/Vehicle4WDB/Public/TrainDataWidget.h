// TrainDataWidget.h
#pragma once
#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Components/TextBlock.h" // ���UTextBlock��ͷ�ļ�����
#include "TrainDataWidget.generated.h"
/**
 * ������ʾ�г�������Ϣ�� UI Widget
 */
UCLASS()
class VEHICLE4WDB_API UTrainDataWidget : public UUserWidget
{
    GENERATED_BODY()
public:

    // �ⲿ(TrainHUDActor)����

    // ���� SPCK �ڲ�����ʱ��
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateSPCKTime(float SPCK_time);
    // ���복������ cm/s �ٶ�
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateSpeed(float SpeedCmS);
    // �����ֵ��ʾ
    UFUNCTION(BlueprintCallable, Category = "TrainData")
    void UpdateTrackS(float InTrackS);

protected:
    virtual void NativeConstruct() override;

    // �Ƿ�ʹ�� m/s�������� km/h
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TrainData")
    bool bUseMPS;

    // ���ζ���TextBlock,�󶨵�UMG�е�TextBlock������Ҫ��ӦDesigner�еĿؼ�����
     // �ڲ�ʱ���TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* SPCKTime_TextBlock;

    // �����ٶȵ�TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* Speed_TextBlock; 

    // ������̵�TextBlock
    UPROPERTY(meta = (BindWidget))
    UTextBlock* TrackS_TextBlock;

private:
    // ��cm/sת��Ϊ��ʾ��λ( km/h �� m/s )
    float ConvertSpeed(float SpeedCmS) const;
};