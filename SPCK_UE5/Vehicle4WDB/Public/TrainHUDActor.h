// File: TrainHUDActor.h
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrainData.h"    // �����е�FTrainData�ṹ
#include "TrainHUDActor.generated.h"

class UTrainDataWidget;
class AUDPReceiver;

/**
 * ��Actor��BeginPlayʱ����HUD Widget��Tick��ʵʱ��AUDPReceiver��ȡCarBodyVx������UI��
 */
UCLASS()
class VEHICLE4WDB_API ATrainHUDActor : public AActor
{
    GENERATED_BODY()

public:
    ATrainHUDActor();

    // �Ƿ��Զ��ڳ����в��ҵ�һ��AUDPReceiver�����Ϊfalse����ʹ��ManualUDPReceiverRef��
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Train HUD")
    bool bAutoFindUDPReceiver;

    // �ֶ�ָ����UDPReceiver���ã����� bAutoFindUDPReceiver=false ʱ��Ч��
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Train HUD", meta = (EditCondition = "!bAutoFindUDPReceiver"))
    AUDPReceiver* ManualUDPReceiverRef;

    // ָ��Ҫ������Widget�ࣨ���������UTrainDataWidget��ͼ����C++���ࣩ
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Train HUD")
    TSubclassOf<UTrainDataWidget> TrainDataWidgetClass;

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

private:
    // ָ��ʵ�ʴ�����AddToViewport��Widgetʵ��
    UPROPERTY()
    UTrainDataWidget* TrainDataWidgetInstance;

    // ����ʹ�õ�UDPReceiver����
    UPROPERTY()
    AUDPReceiver* UDPReceiverPtr;
};
