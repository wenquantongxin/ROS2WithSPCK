#pragma once
#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "TrainData.h"
#include "MoveComponent_Wheelset.generated.h"

// ǰ������
class AUDPReceiver;
class UStaticMeshComponent;

/**
 * ������ղ���ֵ���� �ֶԵ�����任���Լ����ҳ��ֵ���ת
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class VEHICLE4WDB_API UMoveComponent_Wheelset : public USceneComponent
{
    GENERATED_BODY()
public:
    UMoveComponent_Wheelset();
protected:
    virtual void BeginPlay() override;
public:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;
    /** �ֶԵ�������0~3�������������� FTrainData ��ȡ����λ�á���ת */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    int32 WheelsetIndex;
    /** �Ƿ�ʹ������任��SetActorLocationAndRotation�� */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    bool bUseWorldTransform;
    /** ��ֵ�ٶ� */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    float InterpSpeed;
    /** ������תƽ��ϵ�� (0-5)��ֵԽ��ƽ��Ч��Խ���� */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset", meta = (ClampMin = "0.0", ClampMax = "5.0"))
    float WheelRotationSmoothFactor;
    /** ָ�� UDPReceiver����ȡ���� TrainData */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    AUDPReceiver* UDPReceiverRef;
    /** ��ǰ��ֵ�������λ�� */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheelset")
    FVector CurrentLocation;
    /** ��ǰ��ֵ���������ת */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheelset")
    FRotator CurrentRotation;
    /**
     * ָ����ͼ�У���Actor�У�������ʾ"����"��StaticMeshComponent��
     * �Ա���C++��Tick��ֱ�� SetRelativeRotation��
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    UStaticMeshComponent* LeftWheelMesh;
    /**
     * ָ����ͼ�У���Actor�У�������ʾ"�ҳ���"��StaticMeshComponent��
     * �Ա���C++��Tick��ֱ�� SetRelativeRotation��
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheelset")
    UStaticMeshComponent* RightWheelMesh;
    /**
     * ͨ�������µ����������Ĭ�ϱ任��
     */
    UFUNCTION(BlueprintCallable, Category = "Wheelset")
    void SetWheelsetIndex(int32 NewIndex);
protected:
    /** ��������Ĭ��λ�ú���ת�ĸ������� */
    void SetupDefaultTransform();

    /** Ĭ�ϳ�ʼ����λ�ã�����WheelsetIndex������ */
    FVector DefaultLocation;
    /** Ĭ�ϳ�ʼ������ת */
    FRotator DefaultRotation;
    /** �Ƿ��Ѿ���UDP�յ�����Ч���� */
    bool bInitialized;
    /** ��Ӧ������������������������TrainData.WheelsRotation�е��±꣩ */
    int32 LeftWheelIndex;
    int32 RightWheelIndex;
    /** ��¼��ͼ�и� LeftWheelMesh / RightWheelMesh �趨�ĳ�ʼ�����ת���������ⲿ�Ƕȵ��� */
    FRotator InitialLeftWheelRot;
    FRotator InitialRightWheelRot;

    /** ���ҳ��ֵ�ǰ�ۻ���ת�Ƕȣ��ȣ� */
    float AccumLeftWheelRotation;
    float AccumRightWheelRotation;

    /** ���ҳ��ֵ�ǰ��ת�ٶȣ���/�룩 */
    float LeftWheelRotSpeed;
    float RightWheelRotSpeed;

    /** Ŀ����ת�ٶȣ�ƽ�������ã� */
    float TargetLeftWheelRotSpeed;
    float TargetRightWheelRotSpeed;

    /** ���ȵ��Ƕȵ�ת������ */
    static constexpr float RadToDeg = 57.2957795f;
};