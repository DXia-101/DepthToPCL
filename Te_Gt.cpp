#include "Te_Gt.h"
#include <QDebug>

Te_Gt::Te_Gt(QString tag) :label(tag)
{

}


TeContour& Te_Gt::GetContourVec()
{
    return this->contour_vec;
}

TeContourContact& Te_Gt::GetHierarchy()
{
    return this->hierarchy;
}

QString Te_Gt::GetLabel()
{
    return label;
}

