/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __CONTROL_POINT_H_
#define __CONTROL_POINT_H_

#include <sstream>
#include <Eigen/Dense>
#include <MMM/rapidxml.hpp>
#include <MMM/Motion/Motion.h>
#include <MMM/Motion/MotionReaderXML.h>

namespace Bipedal
{

/*
 * This implements an extension to the MMM motion format adding cartesian control
 * points to the frame.
 */
template<typename VectorT>
class ControlPointEntry : public MMM::MotionFrameEntry
{
public:
    enum ControlType
    {
        TYPE_NONE         = 0,
        TYPE_POSITION     = 1,
        TYPE_VELOCITY     = 2,
        TYPE_ACCELERATION = 4
    };

    ControlPointEntry(const std::string& name,
                      const VectorT& pos)
        : MMM::MotionFrameEntry(name)
        , position(pos)
        , type(TYPE_POSITION)
    {
    }

    ControlPointEntry(const std::string& name,
                      const VectorT& pos,
                      const VectorT& vel)
        : MMM::MotionFrameEntry(name)
        , position(pos)
        , velocity(vel)
        , type(static_cast<ControlType>(TYPE_POSITION | TYPE_VELOCITY))
    {
    }

    ControlPointEntry(const std::string& name,
                      const VectorT& pos,
                      const VectorT& vel,
                      const VectorT& acc)
        : MMM::MotionFrameEntry(name)
        , position(pos)
        , velocity(vel)
        , acceleration(acc)
        , type(static_cast<ControlType>(TYPE_POSITION | TYPE_VELOCITY | TYPE_ACCELERATION))
    {
    }

    ControlPointEntry(const std::string& name,
                      ControlType type,
                      const VectorT& pos,
                      const VectorT& vel,
                      const VectorT& acc)
        : MMM::MotionFrameEntry(name)
        , position(pos)
        , velocity(vel)
        , acceleration(acc)
        , type(type)
    {
    }

    void printVector(const std::string& name, const VectorT& vec, std::stringstream& ss)
    {
        ss << "\t\t\t\t\t<" << name << ">";

        for (int i = 0; i < vec.rows(); i++)
        {
            ss << vec(i, 0) << " ";
        }

        ss << "</" << name  << ">" << std::endl;
    }

    virtual std::string toXML()
    {
        std::stringstream ss;
        ss << "\t\t\t\t<" << tagName << ">" << std::endl;

        if (type & TYPE_POSITION)
        {
            printVector("Position", position, ss);
        }

        if (type & TYPE_VELOCITY)
        {
            printVector("Velocity", velocity, ss);
        }

        if (type & TYPE_ACCELERATION)
        {
            printVector("Acceleration", acceleration, ss);
        }

        ss << "\t\t\t\t</" << tagName << ">" << std::endl;

        return ss.str();
    }

    ControlType type;
    VectorT position;
    VectorT velocity;
    VectorT acceleration;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename VectorT>
class ControlPointParser : public MMM::XMLMotionFrameTagProcessor
{
public:
    ControlPointParser(const std::string& tagName)
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

        int type = 0;
        VectorT pos;
        VectorT vel;
        VectorT acc;

        rapidxml::xml_node<>* node = tag->first_node("Position", 0, false);

        if (node)
        {
            type |= ControlPointEntry<VectorT>::TYPE_POSITION;
            parseVector(node, pos);
        }

        node = tag->first_node("Velocity", 0, false);

        if (node)
        {
            type |= ControlPointEntry<VectorT>::TYPE_VELOCITY;
            parseVector(node, vel);
        }

        node = tag->first_node("Acceleration", 0, false);

        if (node)
        {
            type |= ControlPointEntry<VectorT>::TYPE_ACCELERATION;
            parseVector(node, acc);
        }

        boost::shared_ptr<ControlPointEntry<VectorT>> entry(
                    new ControlPointEntry<VectorT>(tagName,
                            static_cast<typename ControlPointEntry<VectorT>::ControlType>(type),
                            pos,
                            vel,
                            acc)
                );
        return motionFrame->addEntry(tagName, entry);
    }

    void parseVector(rapidxml::xml_node<>* node, VectorT& vector)
    {
        std::string value = node->value();
        std::stringstream ss;
        ss << value;

        for (int i = 0; i < vector.rows(); i++)
        {
            float val;
            ss >> val;
            vector(i, 0) = val;
        }
    }

    std::string tagName;
};

template<typename VectorT>
bool GetControlPointPosition(const MMM::MotionFramePtr& frame, const std::string& name, VectorT& pos)
{
    if (!frame->hasEntry(name))
    {
        return false;
    }

    MMM::MotionFrameEntryPtr e = frame->getEntry(name);
    boost::shared_ptr<ControlPointEntry<VectorT>> r = boost::dynamic_pointer_cast<ControlPointEntry<VectorT>>(e);

    if (!r)
    {
        return false;
    }

    pos = r->position;
    return true;
}

template<typename VectorT>
bool GetControlPointVelocity(const MMM::MotionFramePtr& frame, const std::string& name, VectorT& vel)
{
    if (!frame->hasEntry(name))
    {
        return false;
    }

    MMM::MotionFrameEntryPtr e = frame->getEntry(name);
    boost::shared_ptr<ControlPointEntry<VectorT>> r = boost::dynamic_pointer_cast<ControlPointEntry<VectorT>>(e);

    if (!r)
    {
        return false;
    }

    vel = r->velocity;
    return true;
}

template<typename VectorT>
bool GetControlPointAcceleration(const MMM::MotionFramePtr& frame, const std::string& name, VectorT& acc)
{
    if (!frame->hasEntry(name))
    {
        return false;
    }

    MMM::MotionFrameEntryPtr e = frame->getEntry(name);
    boost::shared_ptr<ControlPointEntry<VectorT>> r = boost::dynamic_pointer_cast<ControlPointEntry<VectorT>>(e);

    if (!r)
    {
        return false;
    }

    acc = r->acceleration;
    return true;
}

}

#endif
