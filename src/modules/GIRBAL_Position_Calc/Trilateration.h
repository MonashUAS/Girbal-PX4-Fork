//
//  Trilateration.h
//
//  Created by waynewang on 19/4/14.
//  Copyright (c) 2014 waynewang. All rights reserved.
//

#ifndef __Trilateration__
#define __Trilateration__

#include "Public.h"
#include "Position.h"

class EXPORT_API Trilateration
{
public:
    Pos3d CalculateLocation3d(const PosAndDistance3dVec& anchors, Pos3d& location);
    Pos3d CalculateLocation3dWithError(const PosAndDistance3dVec& anchors, Pos3d& location);
};

#endif /* defined(__Trilateration__) */