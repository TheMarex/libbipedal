#include <fstream>
#include <string>
#include <sstream>

#include <Eigen/Dense>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Nodes/RobotNode.h>

#include <MMM/Motion/Motion.h>
#include <MMM/XMLTools.h>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

#include "utils/mmm/TrajectoryExporter.h"
#include "utils/mmm/ControlPoint.h"
#include "utils/mmm/ControlMatrix.h"
#include "utils/mmm/ControlValue.h"
#include "utils/Estimation.h"

namespace Bipedal
{

void TrajectoryExporter::exportToMMM(const std::string& path)
{
    boost::filesystem::path targetPath(path);
    boost::filesystem::path baseDir = targetPath.parent_path();
    std::string relRobotPath = MMM::XML::make_relative(baseDir.string(), pathToRobot);

    MMM::MotionPtr motion(new MMM::Motion("Walking pattern"));

    motion->setJointOrder(jointNames);

    int size = bodyTrajectory.cols();
    int ndof = bodyTrajectory.rows();

    Eigen::MatrixXf bodyVelocity     = Bipedal::slopeEstimation(bodyTrajectory, timestep);
    Eigen::MatrixXf bodyAcceleratoin = Bipedal::slopeEstimation(bodyVelocity, timestep);

    auto intervalIter = intervals.begin();

    for (int i = 0; i < size; i++)
    {
        while (i >= intervalIter->endIdx)
        {
            intervalIter = std::next(intervalIter);
        }

        // we need rootPos in mm
        MMM::MotionFramePtr frame(new MMM::MotionFrame(ndof));
        Eigen::Matrix4f rootPose = leftFootTrajectory[i];
        frame->setRootPose(rootPose);
        frame->setRootPos(rootPose.block(0, 3, 3, 1));
        frame->joint = bodyTrajectory.col(i);
        frame->joint_vel = bodyVelocity.col(i);
        frame->joint_acc = bodyAcceleratoin.col(i);
        frame->timestep = timestep * i;
        frame->addEntry("CoM",
                        boost::make_shared<ControlPointEntry3f>("CoM",
                                comTrajectory.col(i),
                                comVelocity.col(i),
                                comAcceleration.col(i)
                                                               )
                       );
        frame->addEntry("ZMP",
                        boost::make_shared<ControlPointEntry2f>("ZMP",
                                computedZMPTrajectory.col(i)
                                                               )
                       );
        frame->addEntry("ReferenceZMP",
                        boost::make_shared<ControlPointEntry2f>("ReferenceZMP",
                                referenceZMPTrajectory.col(i)
                                                               )
                       );
        frame->addEntry("LeftFoot",
                        boost::make_shared<ControlMatrixEntry4f>("LeftFoot",
                                leftFootTrajectory[i]
                                                                )
                       );
        frame->addEntry("RightFoot",
                        boost::make_shared<ControlMatrixEntry4f>("RightFoot",
                                rightFootTrajectory[i]
                                                                )
                       );
        frame->addEntry("Chest",
                        boost::make_shared<ControlMatrixEntry4f>("Chest",
                                chestTrajectory[i]
                                                                )
                       );
        frame->addEntry("Pelvis",
                        boost::make_shared<ControlMatrixEntry4f>("Pelvis",
                                pelvisTrajectory[i]
                                                                )
                       );
        frame->addEntry("SupportPhase",
                        boost::make_shared<ControlValueEntry<int>>("SupportPhase",
                                intervalIter->phase
                                                                  )
                       );

        motion->addMotionFrame(frame);
    }

    MMM::ModelPtr model(new MMM::Model());
    model->filename = relRobotPath;
    motion->setModel(model);

    std::ofstream out(path.c_str());
    out << "<MMM>"
        << motion->toXML()
        << "</MMM>";
}

}

