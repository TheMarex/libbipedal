#include <MMM/Motion/Motion.h>
#include <MMM/Motion/MotionFrame.h>
#include <MMM/Motion/MotionReaderXML.h>

#include "utils/mmm/TrajectoryPlayer.h"
#include "utils/mmm/ControlPoint.h"
#include "utils/mmm/ControlValue.h"
#include "utils/mmm/ControlMatrix.h"

bool TrajectoryPlayer::update(float dt)
{
    if (isRunning && frameCounter <= motion->getMotionFrames().size())
    {
        time += dt;

        float remaining = nextFrame->timestep - time;

        if (remaining < 0)
        {
            if (frameCounter < motion->getMotionFrames().size())
            {
                currentFrame = nextFrame;
                nextFrame = motion->getMotionFrame(frameCounter);
                frameCounter++;
            }
            else
            {
                isRunning = false;
            }
        }
    }

    return isRunning;
}

void TrajectoryPlayer::reset()
{
    isRunning = false;
    nextFrame = motion->getMotionFrame(0);
    frameCounter = 1;
    time = 0;
}

bool TrajectoryPlayer::loadMotion(const std::string& motionPath, const std::string& goalMotionName)
{
    MMM::MotionReaderXMLPtr reader(new MMM::MotionReaderXML());
    std::vector<std::string> motionNames = reader->getMotionNames(motionPath);

    std::string motionName = goalMotionName;
    if (goalMotionName == "")
    {
        motionName = motionNames[0];
    }
    else
    {
        auto iter = std::find(motionNames.begin(), motionNames.end(), motionName);
        if (iter == motionNames.end())
        {
            std::cout << "Error: Motion " << motionName << " not found. Using first motion instead!" << std::endl;
            return false;
        }
    }

    boost::shared_ptr<ControlPointParser3f> comParser (new ControlPointParser3f("CoM"));
    boost::shared_ptr<ControlPointParser2f> zmpParser (new ControlPointParser2f("ZMP"));
    boost::shared_ptr<ControlMatrixParser4f> leftFootParser (new ControlMatrixParser4f("LeftFoot"));
    boost::shared_ptr<ControlMatrixParser4f> rightFootParser (new ControlMatrixParser4f("RightFoot"));
    boost::shared_ptr<ControlMatrixParser4f> chestParser (new ControlMatrixParser4f("Chest"));
    boost::shared_ptr<ControlMatrixParser4f> pelvisParser (new ControlMatrixParser4f("Pelvis"));
    boost::shared_ptr<ControlValueParser<int>> phaseParser (new ControlValueParser<int>("SupportPhase"));

    reader->registerMotionFrameXMLTag("CoM", comParser);
    reader->registerMotionFrameXMLTag("ZMP", zmpParser);
    reader->registerMotionFrameXMLTag("LeftFoot", leftFootParser);
    reader->registerMotionFrameXMLTag("RightFoot", rightFootParser);
    reader->registerMotionFrameXMLTag("Chest", chestParser);
    reader->registerMotionFrameXMLTag("Pelvis", pelvisParser);
    reader->registerMotionFrameXMLTag("SupportPhase", phaseParser);

    motion = reader->loadMotion(motionPath, motionName);
    if (motion->getMotionFrames().size() < 2)
    {
        std::cout << "Error: Motion needs at least 2 frames!" << std::endl;
        return false;
    }
    reset();

    return true;
}

