#ifndef __CONTROL_POINT_H_
#define __CONTROL_POINT_H_

#include <sstream>
#include <VirtualRobot/Robot.h>
#include <Eigen/Dense>
#include <MMM/rapidxml.hpp>
#include <MMM/Motion/Motion.h>
#include <MMM/Motion/MotionReaderXML.h>

/*
 * This implements an extension to MMM to add matrix control values.
 *
 */
template<typename MatrixT>
class ControlMatrixEntry : public MMM::MotionFrameEntry
{
public:
    ControlMatrixEntry(const std::string& name, const MatrixT& matrix)
    : MMM::MotionFrameEntry(name)
    , matrix(matrix)
    {
    }

    // stores the matrix in column major format
    void writeMatrix(std::stringstream& ss, const MatrixT& m)
    {
        for (int i = 0; i < m.size()-1; i++)
        {
            ss << m.data()[i] << " ";
        }
        ss << m.data()[m.size() - 1];
    }

    virtual std::string toXML()
    {
        std::stringstream ss;
        ss << "\t\t\t\t<" << tagName << "rows='" << matrix.rows() << "' cols='" << matrix.cols() << "'>";
        writeMatrix(ss, matrix);
        ss << "</" << tagName << ">" << std::endl;

        return ss.str();
    }

    MatrixT matrix;
};

template<typename MatrixT>
class ControlMatrixParser : public MMM::XMLMotionFrameTagProcessor
{
public:
    ControlMatrixParser(const std::string& tagName)
    : XMLMotionFrameTagProcessor()
    , tagName(tagName)
    {
    }

    void parseMatrix(std::stringstream& ss, unsigned rows, unsigned cols, MatrixT& m)
    {
        m.resize(rows, cols);
        for (unsigned i = 0; i < m.size(); i++)
        {
            ss >> m.data()[i];
        }
    }

    virtual bool processMotionFrameXMLTag(rapidxml::xml_node<>* tag,
                                          MMM::MotionFramePtr motionFrame)
    {
        if (!tag)
            return false;

        if (tag->name() != tagName)
            return false;
        rapidxml::xml_attribute<>* attr_rows = tag->first_attribute("rows", 0, false);
        if (!attr_rows)
            return false;
        rapidxml::xml_attribute<>* attr_cols = tag->first_attribute("cols", 0, false);
        if (!attr_cols)
            return false;

        MatrixT m;
        unsigned rows, cols;
        std::stringstream ss;
        ss << attr_rows->value();
        ss >> rows;
        ss << attr_cols->value();
        ss >> cols;
        ss << tag->value();
        parseMatrix(ss, rows, cols, m);

        boost::shared_ptr<ControlMatrixEntry<MatrixT>> entry(
            new ControlMatrixEntry<MatrixT>(tagName, m)
        );

        return motionFrame->addEntry(tagName, entry);
    }

    std::string tagName;
};

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
            return false;

        if (tag->name() != tagName)
            return false;

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
            ss << vec(i, 0) << " ";
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
            return false;

        if (tag->name() != tagName)
            return false;

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

typedef ControlPointEntry<Eigen::Vector2f>  ControlPointEntry2f;
typedef ControlPointParser<Eigen::Vector2f> ControlPointParser2f;
typedef ControlPointEntry<Eigen::Vector3f>  ControlPointEntry3f;
typedef ControlPointParser<Eigen::Vector3f> ControlPointParser3f;

template<typename VectorT>
bool GetControlPointPosition(const MMM::MotionFramePtr& frame, const std::string& name, VectorT& pos)
{
    if (!frame->hasEntry(name))
        return false;

    MMM::MotionFrameEntryPtr e = frame->getEntry(name);
    boost::shared_ptr<ControlPointEntry<VectorT>> r = boost::dynamic_pointer_cast<ControlPointEntry<VectorT>>(e);
    if (!r)
        return false;

    pos = r->position;
    return true;
}

template<typename VectorT>
bool GetControlPointVelocity(const MMM::MotionFramePtr& frame, const std::string& name, VectorT& vel)
{
    if (!frame->hasEntry(name))
        return false;

    MMM::MotionFrameEntryPtr e = frame->getEntry(name);
    boost::shared_ptr<ControlPointEntry<VectorT>> r = boost::dynamic_pointer_cast<ControlPointEntry<VectorT>>(e);
    if (!r)
        return false;

    vel = r->velocity;
    return true;
}

template<typename VectorT>
bool GetControlPointAcceleration(const MMM::MotionFramePtr& frame, const std::string& name, VectorT& acc)
{
    if (!frame->hasEntry(name))
        return false;

    MMM::MotionFrameEntryPtr e = frame->getEntry(name);
    boost::shared_ptr<ControlPointEntry<VectorT>> r = boost::dynamic_pointer_cast<ControlPointEntry<VectorT>>(e);
    if (!r)
        return false;

    acc = r->acceleration;
    return true;
}

template<typename ScalarT>
bool GetControlValue(const MMM::MotionFramePtr& frame, const std::string& name, ScalarT& value)
{
    if (!frame->hasEntry(name))
        return false;

    MMM::MotionFrameEntryPtr e = frame->getEntry(name);
    boost::shared_ptr<ControlValueEntry<ScalarT>> r = boost::dynamic_pointer_cast<ControlValueEntry<ScalarT>>(e);
    if (!r)
        return false;

    value = r->value;
    return true;
}

template<typename MatrixT>
bool GetControlMatrix(const MMM::MotionFramePtr& frame, const std::string& name, MatrixT& value)
{
    if (!frame->hasEntry(name))
        return false;

    MMM::MotionFrameEntryPtr e = frame->getEntry(name);
    boost::shared_ptr<ControlMatrixEntry<MatrixT>> r = boost::dynamic_pointer_cast<ControlMatrixEntry<MatrixT>>(e);
    if (!r)
        return false;

    value = r->matrix;
    return true;
}

#endif
