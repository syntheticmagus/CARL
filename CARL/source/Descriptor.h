/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once

#include "EgocentricTemporalSpace.h"

#include "carl/InputSample.h"

#include <gsl/span>

#include <array>

namespace carl::descriptor
{
    // This namespace contains small, self-contained math constructs for use ONLY within
    // descriptors. These are needed because Eigen matrices are not trivially copyable,
    // so in order for descriptors to remain trivially copyable they need math primitives
    // that are also trivially copyable. Note that these types should be kept as minimal
    // as possible, containing only what's needed for descriptor comparison; Eigen should
    // still be used for all other math.
    namespace trivial
    {
        struct Point
        {
            NumberT X{};
            NumberT Y{};
            NumberT Z{};

            Point() = default;

            Point(const VectorT& v)
                : X{ v.x() }
                , Y{ v.y() }
                , Z{ v.z() }
            {
            }

            Point(NumberT x, NumberT y, NumberT z)
                : X{ x }
                , Y{ y }
                , Z{ z }
            {
            }

            Point& operator=(const VectorT& v)
            {
                X = v.x();
                Y = v.y();
                Z = v.z();
                return *this;
            }

            Point operator-(const Point& other) const
            {
                return{ X - other.X, Y - other.Y, Z - other.Z };
            }

            NumberT lengthSq() const
            {
                return X * X + Y * Y + Z * Z;
            }

            NumberT length() const
            {
                return std::sqrt(lengthSq());
            }

            NumberT distanceSq(const Point& other) const
            {
                return (operator-(other)).lengthSq();
            }

            NumberT distance(const Point& other) const
            {
                return std::sqrt(distanceSq(other));
            }
        };

        struct Quaternion
        {
            NumberT X{};
            NumberT Y{};
            NumberT Z{};
            NumberT W{};

            Quaternion() = default;

            Quaternion(const QuaternionT& q)
                : X{ q.x() }
                , Y{ q.y() }
                , Z{ q.z() }
                , W{ q.w() }
            {
            }

            Quaternion(NumberT x, NumberT y, NumberT z, NumberT w)
                : X{ x }
                , Y{ y }
                , Z{ z }
                , W{ w }
            {
            }

            Quaternion& operator=(const QuaternionT& other)
            {
                X = other.x();
                Y = other.y();
                Z = other.z();
                W = other.w();
                return *this;
            }

            Quaternion operator*(const Quaternion& other) const
            {
                return{
                    W * other.X + X * other.W + Y * other.Z - Z * other.Y,
                    W * other.Y - X * other.Z + Y * other.W + Z * other.X,
                    W * other.Z + X * other.Y - Y * other.X + Z * other.W,
                    W * other.W - X * other.X - Y * other.Y - Z * other.Z,
                };
            }

            Quaternion conjugate() const
            {
                return{ -X, -Y, -Z, W };
            }

            NumberT angularDistance(const Quaternion& other) const
            {
                auto qt{ operator*(other.conjugate()) };
                Point pt{ qt.X, qt.Y, qt.Z };
                return static_cast<NumberT>(2) * std::atan2(pt.length(), std::abs(qt.W));
            }
        };
    }

    template <typename...>
    struct ArraySizeCount;

    template <typename T, size_t S>
    struct ArraySizeCount<std::array<T, S>>
    {
        static constexpr auto Size = S;
    };

    template <typename T, size_t S, typename... Ts>
    struct ArraySizeCount<std::array<T, S>, Ts...>
    {
        static constexpr auto Size = S + ArraySizeCount<Ts...>::Size;
    };

    template <typename T, size_t l, size_t r>
    constexpr std::array<T, l + r> arrayConcat(const std::array<T, l>& a, const std::array<T, r>& b)
    {
        std::array<T, l + r> ret{};
        for (int idx = 0; idx < l; ++idx)
        {
            ret[idx] = a[idx];
        }

        for (int idx = 0; idx < r; ++idx)
        {
            ret[idx + l] = b[idx];
        }

        return ret;
    }

    template <typename T, size_t l, size_t r, typename... Ts>
    constexpr std::array<T, l + r + ArraySizeCount<Ts...>::Size> arrayConcat(
        const std::array<T, l>& a,
        const std::array<T, r>& b,
        const Ts&... ts)
    {
        return arrayConcat(a, arrayConcat(b, ts...));
    }

    template <typename...>
    struct Tuning;

    template <typename T1, typename T2>
    struct Tuning<T1, T2>
    {
        static constexpr auto DEFAULT_TUNING{ arrayConcat(T1::DEFAULT_TUNING, T2::DEFAULT_TUNING) };

        template <typename T>
        static gsl::span<const double> getTuning(gsl::span<const double> values)
        {
            if constexpr (std::is_same<T, T1>::value)
            {
                return gsl::make_span(values.data(), T1::DEFAULT_TUNING.size());
            }
            else if constexpr (std::is_same<T, T2>::value)
            {
                return gsl::make_span(values.data() + T1::DEFAULT_TUNING.size(), T2::DEFAULT_TUNING.size());
            }
        }
    };

    template <typename T1, typename T2, typename... Ts>
    struct Tuning<T1, T2, Ts...>
    {
        static constexpr auto DEFAULT_TUNING{ arrayConcat(T1::DEFAULT_TUNING, Tuning<T2, Ts...>::DEFAULT_TUNING) };

        template <typename T>
        static gsl::span<const double> getTuning(gsl::span<const double> values)
        {
            constexpr auto size = T1::DEFAULT_TUNING.size();
            if constexpr (std::is_same<T, T1>::value)
            {
                return gsl::make_span(values.data(), size);
            }
            else
            {
                return Tuning<T2, Ts...>::template getTuning<T>(gsl::make_span(values.data() + size, values.size() - size));
            }
        }
    };

    enum class Handedness : uint64_t
    {
        LeftHanded,
        RightHanded,
        TwoHanded,
    };

    template<Handedness Handedness>
    inline TransformT getWristPose(const InputSample& sample)
    {
        constexpr size_t Y_AXIS_JOINT{
            static_cast<size_t>(InputSample::HandJoint::LittleFingerBase) };
        constexpr size_t Z_AXIS_JOINT{
            static_cast<size_t>(InputSample::HandJoint::IndexFingerBase) };

        constexpr bool isLeftHanded = Handedness == Handedness::LeftHanded;
        const auto& jointPositions =
            isLeftHanded ? sample.LeftHandJointPositions.value() : sample.RightHandJointPositions.value();
        const auto& wristPosition = jointPositions[static_cast<size_t>(InputSample::HandJoint::Wrist)];

        auto zAxis = (jointPositions[Z_AXIS_JOINT] - wristPosition).normalized();
        auto yAxis = zAxis.cross((jointPositions[Y_AXIS_JOINT] - wristPosition).cross(zAxis)).normalized();
        return math::LookTransform(zAxis, yAxis, wristPosition);
    }

    template <Handedness Handedness>
    class HandShape
    {
        static constexpr std::array<size_t, 5> JOINTS{
            static_cast<size_t>(InputSample::HandJoint::ThumbFingerTip),
            static_cast<size_t>(InputSample::HandJoint::IndexFingerTip),
            static_cast<size_t>(InputSample::HandJoint::MiddleFingerTip),
            static_cast<size_t>(InputSample::HandJoint::RingFingerTip),
            static_cast<size_t>(InputSample::HandJoint::LittleFingerTip),
        };
        static constexpr size_t NORMALIZATION_JOINT{
            static_cast<size_t>(InputSample::HandJoint::IndexFingerBase) };

    public:
        static constexpr std::array<double, JOINTS.size()> DEFAULT_TUNING{
            1000.,
            1000.,
            1000.,
            1000.,
            1000. };
        static constexpr auto HANDEDNESS{ Handedness };

        static std::optional<HandShape> TryCreate(
            const InputSample& sample,
            const InputSample& priorSample)
        {
            if constexpr (Handedness == Handedness::LeftHanded)
            {
                if (sample.LeftHandJointPositions.has_value())
                {
                    return HandShape{ sample, priorSample };
                }
            }
            else if constexpr (Handedness == Handedness::RightHanded) {
                if (sample.RightHandJointPositions.has_value()) {
                    return HandShape{ sample, priorSample };
                }
            }
            return {};
        }

        static double Distance(const HandShape& a, const HandShape& b, gsl::span<const double> tuning) {
            double distance = 0;
            for (size_t idx = 0; idx < a.m_positions.size(); ++idx) {
                distance += (tuning[idx] * std::pow(a.m_positions[idx].distanceSq(b.m_positions[idx]), 2.));
            }
            return distance;
        }

        HandShape() = default;

    private:
        std::array<trivial::Point, JOINTS.size()> m_positions{};

        HandShape(const InputSample& sample, const InputSample&)
        {
            constexpr bool isLeftHanded = Handedness == Handedness::LeftHanded;
            const auto& jointPositions =
                isLeftHanded ? sample.LeftHandJointPositions.value() : sample.RightHandJointPositions.value();

            auto inverseWristPose = getWristPose<Handedness>(sample).inverse();
            
            float normalization =
                (inverseWristPose * jointPositions[NORMALIZATION_JOINT]).norm();
            for (size_t idx = 0; idx < JOINTS.size(); ++idx)
            {
                m_positions[idx] =
                    (inverseWristPose * jointPositions[JOINTS[idx]]) / normalization;
            }
        }
    };

    template <Handedness Handedness>
    class EgocentricWristOrientation
    {
    public:
        static constexpr std::array<double, 1> DEFAULT_TUNING{ 10. };
        static constexpr auto HANDEDNESS{ Handedness };

        static std::optional<EgocentricWristOrientation> TryCreate(
            const InputSample& sample,
            const InputSample& priorSample)
        {
            if (!sample.HmdPose.has_value())
            {
                return{};
            }

            if constexpr (Handedness == Handedness::LeftHanded)
            {
                if (sample.LeftHandJointPositions.has_value() && priorSample.LeftHandJointPositions.has_value())
                {
                    return EgocentricWristOrientation{ sample, priorSample };
                }
            }
            else if constexpr (Handedness == Handedness::RightHanded)
            {
                if (sample.RightHandJointPositions.has_value() && priorSample.RightHandJointPositions.has_value())
                {
                    return EgocentricWristOrientation{ sample, priorSample };
                }
            }
            return{};
        }

        static double Distance(
            const EgocentricWristOrientation& a,
            const EgocentricWristOrientation& b,
            gsl::span<const double> tuning)
        {
            return tuning[0] *
                std::pow(a.m_egocentricTemporalOrientation.angularDistance(b.m_egocentricTemporalOrientation), 2.);
        }

        EgocentricWristOrientation() = default;

    private:
        trivial::Quaternion m_egocentricTemporalOrientation{};

        EgocentricWristOrientation(const InputSample& sample, const InputSample& priorSample)
        {
            constexpr bool isLeftHanded = Handedness == Handedness::LeftHanded;
            auto wristPose = getWristPose<Handedness>(sample);
            auto& priorJointPositions = 
                isLeftHanded ? priorSample.LeftHandJointPositions.value() : priorSample.RightHandJointPositions.value();
            auto& priorWristPosition = priorJointPositions[static_cast<size_t>(InputSample::HandJoint::Wrist)];
            auto& priorHmdPose = priorSample.HmdPose.value();

            auto ets = EgocentricTemporalSpace::getPose(priorWristPosition, priorHmdPose);
            m_egocentricTemporalOrientation = QuaternionT{ ets.rotation().inverse() * wristPose.rotation() };
        }
    };

    template <Handedness Handedness>
    class HandPose
    {
    public:
        using TuningT = Tuning<HandShape<Handedness>, EgocentricWristOrientation<Handedness>>;
        static constexpr auto DEFAULT_TUNING{ TuningT::DEFAULT_TUNING };
        static constexpr auto HANDEDNESS{ Handedness };

        static std::optional<HandPose> TryCreate(
            const InputSample& sample,
            const InputSample& priorSample)
        {
            auto handShapeSample = HandShape<Handedness>::TryCreate(sample, priorSample);
            auto wristOrientationSample =
                EgocentricWristOrientation<Handedness>::TryCreate(sample, priorSample);
            if (handShapeSample.has_value() && wristOrientationSample.has_value())
            {
                return HandPose{ std::move(handShapeSample.value()), std::move(wristOrientationSample.value()) };
            }
            return{};
        }

        static double Distance(const HandPose& a, const HandPose& b, gsl::span<const double> tuning)
        {
            return HandShape<Handedness>::Distance(
                a.m_handShapeSample,
                b.m_handShapeSample,
                TuningT::template getTuning<HandShape<Handedness>>(tuning)) +
                EgocentricWristOrientation<Handedness>::Distance(
                    a.m_wristOrientationSample,
                    b.m_wristOrientationSample,
                    TuningT::template getTuning<EgocentricWristOrientation<Handedness>>(tuning));
        }

        HandPose() = default;

    private:
        HandShape<Handedness> m_handShapeSample;
        EgocentricWristOrientation<Handedness> m_wristOrientationSample;

        HandPose(
            HandShape<Handedness> handShapeSample,
            EgocentricWristOrientation<Handedness> wristOrientationSample)
            : m_handShapeSample{ std::move(handShapeSample) }
            , m_wristOrientationSample{ std::move(wristOrientationSample) }
        {
        }
    };

    template <Handedness Handedness>
    class EgocentricWristTranslation
    {
    public:
        static constexpr std::array<double, 1> DEFAULT_TUNING{ 10. };
        static constexpr auto HANDEDNESS{ Handedness };

        static std::optional<EgocentricWristTranslation> TryCreate(
            const InputSample& sample,
            const InputSample& priorSample)
        {
            if (!sample.HmdPose.has_value())
            {
                return {};
            }

            if constexpr (Handedness == Handedness::LeftHanded)
            {
                if (sample.LeftHandJointPositions.has_value() && priorSample.LeftHandJointPositions.has_value())
                {
                    return EgocentricWristTranslation{ sample, priorSample };
                }
            }
            else if constexpr (Handedness == Handedness::RightHanded)
            {
                if (sample.RightHandJointPositions.has_value() && priorSample.RightHandJointPositions.has_value())
                {
                    return EgocentricWristTranslation{ sample, priorSample };
                }
            }
            return{};
        }

        static double Distance(
            const EgocentricWristTranslation& a,
            const EgocentricWristTranslation& b,
            gsl::span<const double> tuning)
        {
            return tuning[0] *
                a.m_egocentricTemporalPositionDecimeters.distanceSq(b.m_egocentricTemporalPositionDecimeters);
        }

        EgocentricWristTranslation() = default;

    private:
        trivial::Point m_egocentricTemporalPositionDecimeters{};

        EgocentricWristTranslation(const InputSample& sample, const InputSample& priorSample)
        {
            constexpr bool isLeftHanded = Handedness == Handedness::LeftHanded;
            auto& jointPositions = isLeftHanded ? sample.LeftHandJointPositions.value() : sample.RightHandJointPositions.value();
            auto& wristPosition = jointPositions[static_cast<size_t>(InputSample::HandJoint::Wrist)];
            auto& priorJointPositions = isLeftHanded ? priorSample.LeftHandJointPositions.value() : priorSample.RightHandJointPositions.value();
            auto& priorWristPosition = priorJointPositions[static_cast<size_t>(InputSample::HandJoint::Wrist)];
            auto& priorHmdPose = priorSample.HmdPose.value();

            auto ets = EgocentricTemporalSpace::getPose(priorWristPosition, priorHmdPose);
            m_egocentricTemporalPositionDecimeters = 10.f * (ets.inverse() * wristPosition);
        }
    };

    template <Handedness Handedness>
    class HandGesture
    {
    public:
        using TuningT = Tuning<HandPose<Handedness>, EgocentricWristTranslation<Handedness>>;
        static constexpr auto DEFAULT_TUNING{ TuningT::DEFAULT_TUNING };
        static constexpr auto HANDEDNESS{ Handedness };

        static std::optional<HandGesture> TryCreate(
            const InputSample& sample,
            const InputSample& priorSample) {
            auto handPoseSample = HandPose<Handedness>::TryCreate(sample, priorSample);
            auto wristTranslationSample =
                EgocentricWristTranslation<Handedness>::TryCreate(sample, priorSample);
            if (handPoseSample.has_value() && wristTranslationSample.has_value())
            {
                return HandGesture{
                    std::move(handPoseSample.value()), std::move(wristTranslationSample.value()) };
            }
            return{};
        }

        static double
            Distance(const HandGesture& a, const HandGesture& b, gsl::span<const double> tuning)
        {
            return HandPose<Handedness>::Distance(
                a.m_handPoseSample,
                b.m_handPoseSample,
                TuningT::template getTuning<HandPose<Handedness>>(tuning)) +
                EgocentricWristTranslation<Handedness>::Distance(
                    a.m_wristTranslationSample,
                    b.m_wristTranslationSample,
                    TuningT::template getTuning<EgocentricWristTranslation<Handedness>>(tuning));
        }

        HandGesture() = default;

    private:
        HandPose<Handedness> m_handPoseSample;
        EgocentricWristTranslation<Handedness> m_wristTranslationSample;

        HandGesture(
            HandPose<Handedness> handPoseSample,
            EgocentricWristTranslation<Handedness> wristTranslationSample)
            : m_handPoseSample{ std::move(handPoseSample) }
            , m_wristTranslationSample{ std::move(wristTranslationSample) }
        {
        }
    };

    class EgocentricRelativeWristPosition
    {
    public:
        static constexpr std::array<double, 1> DEFAULT_TUNING{ 10. };
        static constexpr auto HANDEDNESS{ Handedness::TwoHanded };

        static std::optional<EgocentricRelativeWristPosition> TryCreate(
            const InputSample& sample,
            const InputSample& priorSample)
        {
            if (sample.HmdPose.has_value() && 
                sample.LeftHandJointPositions.has_value() &&
                sample.RightHandJointPositions.has_value())
            {
                return EgocentricRelativeWristPosition{ sample, priorSample };
            }
            return{};
        }

        static double Distance(
            const EgocentricRelativeWristPosition& a,
            const EgocentricRelativeWristPosition& b,
            gsl::span<const double> tuning)
        {
            return tuning[0] *
                a.m_egocentricRelativeWristPosition.distance((b.m_egocentricRelativeWristPosition));
        }

        EgocentricRelativeWristPosition() = default;

    private:
        trivial::Point m_egocentricRelativeWristPosition{};

        EgocentricRelativeWristPosition(const InputSample& sample, const InputSample&)
        {
            auto& leftWristPosition = sample.LeftHandJointPositions.value()[static_cast<size_t>(InputSample::HandJoint::Wrist)];
            auto& rightWristPosition = sample.RightHandJointPositions.value()[static_cast<size_t>(InputSample::HandJoint::Wrist)];
            auto& hmdPose = sample.HmdPose.value();

            auto ets = EgocentricTemporalSpace::getPose(leftWristPosition, hmdPose);
            m_egocentricRelativeWristPosition = ets.inverse() * rightWristPosition;
        }
    };

    class TwoHandGesture {
    public:
        using TuningT = Tuning<
            HandGesture<Handedness::LeftHanded>,
            HandGesture<Handedness::RightHanded>,
            EgocentricRelativeWristPosition>;
        static constexpr auto DEFAULT_TUNING{ TuningT::DEFAULT_TUNING };
        static constexpr auto HANDEDNESS{ Handedness::TwoHanded };

        static std::optional<TwoHandGesture> TryCreate(
            const InputSample& sample,
            const InputSample& priorSample)
        {
            auto leftGestureSample = HandGesture<Handedness::LeftHanded>::TryCreate(sample, priorSample);
            auto rightGestureSample = HandGesture<Handedness::RightHanded>::TryCreate(sample, priorSample);
            auto relativeSample = EgocentricRelativeWristPosition::TryCreate(sample, priorSample);
            if (leftGestureSample.has_value() && 
                rightGestureSample.has_value() &&
                relativeSample.has_value())
            {
                return TwoHandGesture{
                    std::move(leftGestureSample.value()),
                    std::move(rightGestureSample.value()),
                    std::move(relativeSample.value()) };
            }
            return{};
        }

        static double
            Distance(const TwoHandGesture& a, const TwoHandGesture& b, gsl::span<const double> tuning)
        {
            return HandGesture<Handedness::LeftHanded>::Distance(
                a.m_leftGestureSample,
                b.m_leftGestureSample,
                TuningT::template getTuning<HandGesture<Handedness::LeftHanded>>(tuning)) +
                HandGesture<Handedness::RightHanded>::Distance(
                    a.m_rightGestureSample,
                    b.m_rightGestureSample,
                    TuningT::template getTuning<HandGesture<Handedness::RightHanded>>(tuning)) +
                EgocentricRelativeWristPosition::Distance(
                    a.m_relativeSample,
                    b.m_relativeSample,
                    TuningT::template getTuning<EgocentricRelativeWristPosition>(tuning));
        }

        TwoHandGesture() = default;

    private:
        HandGesture<Handedness::LeftHanded> m_leftGestureSample;
        HandGesture<Handedness::RightHanded> m_rightGestureSample;
        EgocentricRelativeWristPosition m_relativeSample;

        TwoHandGesture(
            HandGesture<Handedness::LeftHanded> leftGestureSample,
            HandGesture<Handedness::RightHanded> rightGestureSample,
            EgocentricRelativeWristPosition relativeSample)
            : m_leftGestureSample{ std::move(leftGestureSample) }
            , m_rightGestureSample{ std::move(rightGestureSample) }
            , m_relativeSample{ std::move(relativeSample) }
        {
        }
    };
}
