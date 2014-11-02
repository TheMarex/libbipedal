/*

Copyright (c) 2014, Patrick Niklaus, others
All rights reserved.

This file is licensed under the simplified 2-clause BSD license as provided
in by LICENSE file.

*/
#ifndef __CONTROL_MATRIX_H_
#define __CONTROL_MATRIX_H_

#include <sstream>
#include <Eigen/Dense>
#include <MMM/rapidxml.hpp>
#include <MMM/Motion/Motion.h>
#include <MMM/Motion/MotionReaderXML.h>

namespace Bipedal
{

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
        for (int i = 0; i < m.size() - 1; i++)
        {
            ss << m.data()[i] << " ";
        }

        ss << m.data()[m.size() - 1];
    }

    virtual std::string toXML()
    {
        std::stringstream ss;
        ss << "\t\t\t\t<" << tagName << " rows='" << matrix.rows() << "' cols='" << matrix.cols() << "'>";
        writeMatrix(ss, matrix);
        ss << "</" << tagName << ">" << std::endl;

        return ss.str();
    }

    MatrixT matrix;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
        {
            return false;
        }

        if (tag->name() != tagName)
        {
            return false;
        }

        rapidxml::xml_attribute<>* attr_rows = tag->first_attribute("rows", 0, false);

        if (!attr_rows)
        {
            return false;
        }

        rapidxml::xml_attribute<>* attr_cols = tag->first_attribute("cols", 0, false);

        if (!attr_cols)
        {
            return false;
        }

        MatrixT m;
        unsigned rows, cols;
        std::stringstream ss;
        ss << attr_rows->value() << " " << attr_cols->value() << " " << tag->value();
        ss >> rows;
        ss >> cols;
        parseMatrix(ss, rows, cols, m);

        boost::shared_ptr<ControlMatrixEntry<MatrixT>> entry(
                    new ControlMatrixEntry<MatrixT>(tagName, m)
                );

        return motionFrame->addEntry(tagName, entry);
    }

    std::string tagName;
};

template<typename MatrixT>
bool GetControlMatrix(const MMM::MotionFramePtr& frame, const std::string& name, MatrixT& value)
{
    if (!frame->hasEntry(name))
    {
        return false;
    }

    MMM::MotionFrameEntryPtr e = frame->getEntry(name);
    boost::shared_ptr<ControlMatrixEntry<MatrixT>> r = boost::dynamic_pointer_cast<ControlMatrixEntry<MatrixT>>(e);

    if (!r)
    {
        return false;
    }

    value = r->matrix;
    return true;
}

}

#endif
