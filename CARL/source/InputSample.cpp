/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <carl/InputSample.h>

#include <gsl/span>

#include <cassert>

namespace carl
{
    InputSample::InputSample(Deserialization& deserialization)
    {
        deserialization >> Timestamp;
        deserialization >> HmdPose;
        deserialization >> LeftHandJointPositions;
        deserialization >> RightHandJointPositions;
    }

    void InputSample::serialize(Serialization& serialization) const
    {
        serialization << Timestamp;
        serialization << HmdPose;
        serialization << LeftHandJointPositions;
        serialization << RightHandJointPositions;
    }

    InputSample InputSample::lerp(const InputSample& a, const InputSample& b, double t)
    {
        InputSample result{};

        result.Timestamp = (1. - t) * a.Timestamp + t * b.Timestamp;

        constexpr auto lerpOptional =
            [](const auto& l, const auto& r, double t) -> std::optional<TransformT> {
            if (l.has_value() && r.has_value())
            {
                return math::Lerp(l.value(), r.value(), static_cast<float>(t));
            }
            else if (l.has_value())
            {
                return l;
            }
            else
            {
                return r;
            }
        };

        result.HmdPose = lerpOptional(a.HmdPose, b.HmdPose, t);

        constexpr auto lerpOptionalArray = [](const auto& l, const auto& r, double t, auto& target) {
            if (l.has_value() && r.has_value())
            {
                gsl::span<const VectorT> lSamples{ l.value() };
                gsl::span<const VectorT> rSamples{ r.value() };
                assert(lSamples.size() == rSamples.size());
                std::array<VectorT, static_cast<size_t>(HandJoint::COUNT)> lerped{};
                for (size_t idx = 0; idx < lSamples.size(); ++idx)
                {
                    lerped[idx] = math::Lerp(lSamples[idx], rSamples[idx], static_cast<float>(t));
                }
                target = std::move(lerped);
            }
            else if (l.has_value())
            {
                target = l.value();
            }
            else if (r.has_value())
            {
                target = r.value();
            }
        };

        lerpOptionalArray(a.LeftHandJointPositions, b.LeftHandJointPositions, t, result.LeftHandJointPositions);
        lerpOptionalArray(a.RightHandJointPositions, b.RightHandJointPositions, t, result.RightHandJointPositions);

        return result;
    }
}
