/* 
 * File:   TemporalSmoother.h
 * Author: ros
 *
 * Created on 20. Mai 2011, 09:13
 */

#ifndef TEMPORALSMOOTHER_H
#define	TEMPORALSMOOTHER_H

#include <list>
#include "CvShapes.h"

template<class T, int capacity = 10> class TemporalSmoother
{
public:
    TemporalSmoother() { }
    TemporalSmoother(const TemporalSmoother& orig) { }
    virtual ~TemporalSmoother() { }
    
    void push_back(const T& object)
    {
        _content.push_back(object);
        if(_content.size() > capacity) _content.pop_front();
    }
    
    void Clear()
    {
        _content.clear();
    }
    
    T getValue()
    {
        T sum;
        for(auto it = _content.begin(); it != _content.end(); it++)
        {
            sum += (*it);
        }
        T smoothed = sum / _content.size();
        return smoothed;
    }

private:
    std::list<T> _content;
};

#endif	/* TEMPORALSMOOTHER_H */

