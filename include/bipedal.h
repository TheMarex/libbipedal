/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __BIPEDAL_H__
#define __BIPEDAL_H__

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

namespace Eigen {
    typedef Matrix<float, 6, Dynamic> Matrix6Xf;
    typedef Matrix<float, 6, 1> Vector6f;
};

namespace Bipedal {

enum SupportPhase : unsigned short;

template<bool, bool, bool> class PostureController;
using TwoDOFPostureController = PostureController<true, true, false>;
using ThreeDOFPostureController = PostureController<true, true, true>;

class CoPZMPEstimator;
class FallDetector;
class ZMPFallDetector;
class SupportPhaseSensor;
class PushRecovery;
class CapturePointRecovery;
class ZMPPlaner;
class ZMPReferencePlaner;
class ZMPPreviewControl;
class WalkingIK;
class HierarchicalWalkingIK;
class TrajectoryLogger;
class TorqueControllingStabilizer;
class StabilizerFactory;
class CartesianStabilizer;
class FrameAdaptingStabilizer;
class PolynominalFootstepPlaner;
class ZMPPreviewControl;
class FootstepPlaner;
class ZMPPlaner;
class TrajectoryPlayer;
class TrajectoryExporter;
class ReferenceIK;
class KajitaStabilizer;
class DampeningController;
class MultiBodyZMPEstimator;
class CartTableZMPEstimator;
template<typename T> class ControlMatrixEntry;
template<typename T> class ControlMatrixParser;
template<typename T> class ControlPointEntry;
template<typename T> class ControlPointParser;
template<typename T> class ControlValueEntry;
template<typename T> class ControlValueParser;

template<typename T> class CubivBezierCurve;
typedef CubivBezierCurve<Eigen::Vector3f> CubicBezierCurve3f;
typedef CubivBezierCurve<Eigen::Vector2f> CubicBezierCurve2f;

template<typename T, unsigned order, unsigned derivative> class BackwardDerivationEstimator;
template<typename T, unsigned ACCURACY=1> using FirstDerivativeEstimator = BackwardDerivationEstimator<T, ACCURACY, 1>;
template<typename T, unsigned ACCURACY=1> using SecondDerivativeEstimator = BackwardDerivationEstimator<T, ACCURACY, 2>;

typedef ControlPointEntry<Eigen::Vector2f>   ControlPointEntry2f;
typedef ControlPointParser<Eigen::Vector2f>  ControlPointParser2f;
typedef ControlPointEntry<Eigen::Vector3f>   ControlPointEntry3f;
typedef ControlPointParser<Eigen::Vector3f>  ControlPointParser3f;
typedef ControlMatrixEntry<Eigen::Matrix4f>  ControlMatrixEntry4f;
typedef ControlMatrixParser<Eigen::Matrix4f> ControlMatrixParser4f;

typedef boost::shared_ptr<CoPZMPEstimator>             CoPZMPEstimatorPtr;
typedef boost::shared_ptr<ZMPFallDetector>             ZMPFallDetectorPtr;
typedef boost::shared_ptr<FallDetector>                FallDetectorPtr;
typedef boost::shared_ptr<SupportPhaseSensor>          SupportPhaseSensorPtr;
typedef boost::shared_ptr<PushRecovery>                PushRecoveryPtr;
typedef boost::shared_ptr<CapturePointRecovery>        CapturePointRecoveryPtr;
typedef boost::shared_ptr<WalkingIK>                   WalkingIKPtr;
typedef boost::shared_ptr<HierarchicalWalkingIK>       HierarchicalWalkingIKPtr;
typedef boost::shared_ptr<TorqueControllingStabilizer> TorqueControllingStabilizerPtr;
typedef boost::shared_ptr<StabilizerFactory>           StabilizerFactoryPtr;
typedef boost::shared_ptr<FrameAdaptingStabilizer>     FrameAdaptingStabilizerPtr;
typedef boost::shared_ptr<KajitaStabilizer>            KajitaStabilizerPtr;
typedef boost::shared_ptr<CartesianStabilizer>         CartesianStabilizerPtr;
typedef boost::shared_ptr<TrajectoryPlayer>            TrajectoryPlayerPtr;
typedef boost::shared_ptr<TrajectoryExporter>          TrajectoryExporterPtr;
typedef boost::shared_ptr<TrajectoryLogger>            TrajectoryLoggerPtr;
typedef boost::shared_ptr<ReferenceIK>                 ReferenceIKPtr;
typedef boost::shared_ptr<FootstepPlaner>              FootstepPlanerPtr;
typedef boost::shared_ptr<ZMPPlaner>                   ZMPPlanerPtr;
typedef boost::shared_ptr<PolynominalFootstepPlaner>   PolynominalFootstepPlanerPtr;
typedef boost::shared_ptr<ZMPPreviewControl>           ZMPPreviewControlPtr;
typedef boost::shared_ptr<TwoDOFPostureController>     TwoDOFPostureControllerPtr;
typedef boost::shared_ptr<ThreeDOFPostureController>   ThreeDOFPostureControllerPtr;
typedef boost::shared_ptr<ZMPPlaner>                   ZMPPlanerPtr;
typedef boost::shared_ptr<ZMPReferencePlaner>          ZMPReferencePlanerPtr;
typedef boost::shared_ptr<ZMPPreviewControl>           ZMPPreviewControlPtr;
typedef boost::shared_ptr<MultiBodyZMPEstimator>       MultiBodyZMPEstimatorPtr;
typedef boost::shared_ptr<CartTableZMPEstimator>       CartTableZMPEstimatorPtr;

}

#endif
