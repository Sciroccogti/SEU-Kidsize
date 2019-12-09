#ifndef __DATADEF_HPP
#define __DATADEF_HPP

#include <common/PlayerInfo.h>


struct CameraParams
{
    float fx, fy;
    float cx, cy;
    float k1, k2;
    float p1, p2;
};

struct DetObject
{
    int id;
    float prob;
    int x, y, w, h;
    DetObject(int i = 0, float p = 1, int x_ = 0, int y_ = 0, 
                int w_ = 0, int h_ = 0)
        : id(i), prob(p), x(x_), y(y_), w(w_), h(h_) {}
    bool operator< (const DetObject &obj)
    {
        return prob < obj.prob;
    }
};

#define TC_COMM_PORT 6868
#define TC_STRUCT_HEADER "SEU"
struct TeamCommData
{
    char header[3] = {'S', 'E', 'U'};
    common::PlayerInfo player;
};

#endif
