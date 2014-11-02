/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __CONTROL_VALUE_H_
#define __CONTROL_VALUE_H_

#include <sstream>
#include <VirtualRobot/Robot.h>
#include <Eigen/Dense>
#include <MMM/rapidxml.hpp>
#include <MMM/Motion/Motion.h>
#include <MMM/Motion/MotionReaderXML.h>

namespace Bipedal
{

/*
 * This implements an extension to MMM to add scalar control values.
 */
template<typename ScalarT>
class ControlValueEntry : public MMM::MotionFrameEntry
{
public:
    ControlValueEntry(const std::string& name, ScalarT value)
        : MMM::MotionFrameEntry(name)
        , value(value)
    {
    }

    virtual std::string toXML()
    {
        std::stringstream ss;
        ss << "\t\t\t\t<" << tagName << ">" << value << "</" << tagName << ">" << std::endl;

        return ss.str();
    }

    ScalarT value;
};

template<typename ScalarT>
class ControlValueParser : public MMM::XMLMotionFrameTagProcessor
{
public:
    ControlValueParser(const std::string& tagName)
        : XMLMotionFrameTagProcessor()
        , tagName(tagName)
    {
    }

    virtual bool processMotionFrameXMLTag(rapidxml::xml_node<>* tag,
                                          MMM::MotionFramePtr motionFrame)
    {
        if (!tag)
        {
            return false;
        }

        if (tag->name() != tagName)
        {
            return false;
        }

        ScalarT value;
        std::stringstream ss;
        ss << tag->value();
        ss >> value;

        boost::shared_ptr<ControlValueEntry<ScalarT>> entry(
                    new ControlValueEntry<ScalarT>(tagName, value)
                );

        return motionFrame->addEntry(tagName, entry);
    }

    std::string tagName;
};

template<typename ScalarT>
bool GetControlValue(const MMM::MotionFramePtr& frame, const std::string& name, ScalarT& value)
{
    if (!frame->hasEntry(name))
    {
        return false;
    }

    MMM::MotionFrameEntryPtr e = frame->getEntry(name);
    boost::shared_ptr<ControlValueEntry<ScalarT>> r = boost::dynamic_pointer_cast<ControlValueEntry<ScalarT>>(e);

    if (!r)
    {
        return false;
    }

    value = r->value;
    return true;
}

}

#endif
