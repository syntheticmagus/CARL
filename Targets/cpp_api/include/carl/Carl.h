#pragma once

#include <array>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>

namespace carl::cpp
{
    struct Vector
    {
        float X{};
        float Y{};
        float Z{};
    };

    struct Quaternion
    {
        float X{};
        float Y{};
        float Z{};
        float W{};
    };

    struct Transform
    {
        Vector Position{};
        Quaternion Orientation{};
    };

    struct Hmd
    {
        Transform Transform{};
    };

    struct Hand
    {
        enum class Joint : uint32_t
        {
            UNUSED_HandJointId_HandThumb0,
            ThumbFingerBase,
            UNUSED_HandJointId_HandThumb2,
            UNUSED_HandJointId_HandThumb3,
            ThumbFingerTip,
            IndexFingerBase,
            UNUSED_HandJointId_HandIndex2,
            UNUSED_HandJointId_HandIndex3,
            IndexFingerTip,
            MiddleFingerBase,
            UNUSED_HandJointId_HandMiddle2,
            UNUSED_HandJointId_HandMiddle3,
            MiddleFingerTip,
            RingFingerBase,
            UNUSED_HandJointId_HandRing2,
            UNUSED_HandJointId_HandRing3,
            RingFingerTip,
            UNUSED_HandJointId_HandPinky0,
            LittleFingerBase,
            UNUSED_HandJointId_HandPinky2,
            UNUSED_HandJointId_HandPinky3,
            LittleFingerTip,
            COUNT
        };

        Transform WristTransform{};
        std::array<Transform, static_cast<uint32_t>(Joint::COUNT)> JointTransforms{};
    };

    struct QuestMotionController
    {
        Transform Transform{};
        float Trigger{};
        Vector Thumbstick{};
        float ButtonAX{};
        float ButtonBY{};
    };

    struct InputSample
    {
        bool IsCoordinateSpaceRightHanded{ true };
        Vector CoordinateSpaceUp{ 0.f, 1.f, 0.f };

        double Timestamp{};
        Hmd Hmd{};
        std::optional<Hand> LeftHand{};
        std::optional<Hand> RightHand{};
        std::optional<QuestMotionController> LeftController{};
        std::optional<QuestMotionController> RightController{};
    };

    class Session;

    class Recognizer
    {
    public:
        ~Recognizer();

        double currentScore() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl{};

        friend class Session;
        Recognizer(std::unique_ptr<Impl>);
    };

    class Session
    {
    public:
        Session(bool isSingleThreaded);
        ~Session();

        std::future<std::unique_ptr<Recognizer>> createRecognizerFromSerializedDefinitionAsync(uint8_t* bytes, size_t bytesLength);

        void AddInputSample(InputSample);

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl{};
    };
}
