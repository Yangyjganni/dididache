//
//  index_map.h
//  getPointPos
//
//  Created by Fan Qu on 10/31/18.
//  Copyright © 2018 GF. All rights reserved.
//

#ifndef index_map_h
#define index_map_h

#include <stdio.h>
#include "FindRoad.h"

#define map_width 270
extern const unsigned char use_map[map_width][map_width];
#define max_e_num 85
extern const unsigned char e_be[max_e_num], e_ed[max_e_num];
#define max_points_num 56



extern const int points_x[max_points_num], points_y[max_points_num];
extern const short points_angle[max_points_num];

#ifdef DEBUG_MODE
extern const short min_dis_map[max_points_num][max_points_num];
#endif

#endif /* index_map_h */
