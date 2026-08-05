#pragma once

#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {

    class MotionCaptureMotionAnalysisImpl;

    class MotionCaptureMotionAnalysis : public MotionCapture {
    public:
        MotionCaptureMotionAnalysis(
                const std::string &hostname,
                int updateFrequency = 100);

        virtual ~MotionCaptureMotionAnalysis();

        const std::string &version() const;

        virtual void waitForNextFrame() override;

        virtual const std::map<std::string, RigidBody> &rigidBodies() const override;

        virtual RigidBody rigidBodyByName(const std::string &name) const override;

        virtual bool supportsRigidBodyTracking() const override;

    private:
        MotionCaptureMotionAnalysisImpl *pImpl;
    };

} // namespace libmotioncapture