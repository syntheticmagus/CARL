/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once

#include "carl/Serialization.h"

#include <Eigen/Geometry>

namespace carl
{
    /// <summary>
    /// A snapshot of all data relevant to CARL at a specific point in time.
    /// All 3D data described in an InputSample must be in the same reference
    /// space.
    /// </summary>
    struct InputSample
    {
        enum class HandJoint : uint64_t
        {
            Wrist,
            ThumbFingerBase,
            ThumbFingerTip,
            IndexFingerBase,
            IndexFingerTip,
            MiddleFingerBase,
            MiddleFingerTip,
            RingFingerBase,
            RingFingerTip,
            LittleFingerBase,
            LittleFingerTip,
            COUNT
        };

        InputSample() = default;
        InputSample(Deserialization& deserialization);

        void serialize(Serialization& serialization) const;

        /// <summary>
        /// Timestamp at which the observations within this InputSample were valid.
        /// </summary>
        double Timestamp{};

        std::optional<TransformT> HmdPose{};
        std::optional<std::array<VectorT, static_cast<size_t>(HandJoint::COUNT)>> LeftHandJointPositions{};
        std::optional<std::array<VectorT, static_cast<size_t>(HandJoint::COUNT)>> RightHandJointPositions{};

        static InputSample lerp(const InputSample& a, const InputSample& b, double t);
    };
}
