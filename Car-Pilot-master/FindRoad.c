//
// Created by Fan Qu on 10/31/18.
//

#include "FindRoad.h"
#include "index_map.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define MIN_DIS 5

const short SHORT_INF = (short)0x0f0f;

short edgeMap[max_points_num + 1][max_points_num + 1];


CarMove movePool[CARMOVE_POOL_LEN];

static int carMovePoolTop = 0;


Edge getEdge(int a, int b, int w)
{
    Edge tempEdge;
    tempEdge.a = a; tempEdge.b = b; tempEdge.w = w;
    return tempEdge;
}



void addEdge(int a, int b, int w)
{

}

EdgeList initEdgeList(int num)
{
    EdgeList tempList;
		static int edgePoolTop = 0;
#ifdef PRINT_INFO
		printf("prepare to malloc %d Edge total %lu Bytes \n", num, num * sizeof(Edge) );
#endif
    tempList.data = (Edge*)malloc(num * sizeof(Edge));
    tempList.num = num;
    return tempList;
}

void deleteEdgeList(EdgeList *list)
{
    if(list) {
        if(list->data)
            free(list->data);
        list->data = NULL;
        list->num = 0;
    }
}



MoveList initMoveList(int num)
{
    MoveList tempList;
	#ifdef PRINT_INFO
		printf("prepare to malloc %d of CarMove \n", num);
	#endif
		//carMovePoolTop = 0;
		tempList.data = movePool + carMovePoolTop;
		carMovePoolTop += num;
	//tempList.data = (CarMove*)malloc(num * sizeof(CarMove));
	
    tempList.num = num;
    return tempList;
}

void deleteMoveList(MoveList *moveList)
{
    if(moveList) {
#ifdef PRINT_INFO

#endif
        if(moveList->data)
            //free(moveList->data);
						carMovePoolTop -= moveList->num;
        //moveList->data = NULL;
        moveList->num = 0;
    }
}


short getDis(int x1, int y1, int x2, int y2)
{
    short dis = (short)(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
    return dis;
}

short min(short x, short y)
{
    return x < y ? x : y;
}


void initDis()
{
    memset(edgeMap, 0x0f, sizeof(edgeMap));
    int n = max_points_num;
    for(int i = 0; i < max_e_num; ++i) {
        int st = e_be[i], ed = e_ed[i];
        edgeMap[st][ed] = getDis(points_x[st - 1], points_y[st - 1], points_x[ed - 1], points_y[ed - 1]);
    }
    for(int i = 0; i <= n; ++i)
        edgeMap[i][i] = 0;

}


char vis[max_points_num + 1];
short minDis[max_points_num + 1];
int preNode[max_points_num + 1];
EdgeList dijkstra(int st, int ed)
{
	
    initDis();
#ifdef PRINT_INFO
    printf("after init dis in dijkstra\n");
#endif
    int closestIndex = 0;
    int n = max_points_num;
    short tempMinDis;
    memset(vis, 0, sizeof(vis));
    memset(minDis, 0x0f, sizeof(minDis));
    memset(preNode, 0, sizeof(preNode));
    minDis[st] = 0;
    for(int i = 1; i <=n; ++i) {
        tempMinDis = 0x0f0f;
        for(int j = 1; j <=n; ++j) {
            if (!vis[j] && minDis[j] < tempMinDis) {
                tempMinDis = minDis[j];
                closestIndex = j;
            }
        }
        vis[closestIndex] = 1;
        for(int j = 1; j <= n; ++j) {
            if (minDis[j] > tempMinDis + edgeMap[closestIndex][j]) {
                minDis[j] = tempMinDis + edgeMap[closestIndex][j];
                preNode[j] = closestIndex;
            }
        }

    }
    int pathLen = 0;
    for(int i = ed; i != st; ) {
        pathLen++;
        i = preNode[i];
    }
    EdgeList edgeList = initEdgeList(pathLen);
    for(int i = ed, j = 1; i !=st; j++, i = preNode[i]) {

        edgeList.data[pathLen - j] =  getEdge(preNode[i], i, edgeMap[preNode[i]][i]);

    }

    return edgeList;
    /*
    for(int i = 1; i <= n; ++i)
        if(minDis[i] != min_dis_map[st - 1][i - 1])
            printf("Not same between %d and %d before %d the other %d\n", st, i, minDis[i], min_dis_map[st][i]);
    */

}

float EuclideanDis(int x1, int y1, int x2, int y2)
{
    return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

float myCos(float angle)
{
    return cosf(angle / 180.0f * MY_PI);
}

float mySin(float angle)
{
    return sinf(angle / 180.0f * MY_PI);
}

float crossProduct(float x1, float y1, float x2, float y2)
{
    return x1 * y2 - x2 * y1;
}

void computeIntersection(float x1, float y1, float angle1, float x2, float y2, float angle2, float* point_x, float* point_y)
{
    float c1 = myCos(angle1), c2 = myCos(angle2), s1 = mySin(angle1);
    float t = (x1 - x2) / (c2 - c1);
    *point_x = x1 + c1 * t;
    *point_y = y1 + s1 * t;
}


/**
 * 判断从(x1, y1)到(x2, y2)是否是type3的carmove，即运动前后车头方向不变，运动轨迹平移
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return
 */
int isType3Move(int x1, int y1, int x2, int y2)
{
    if(getDis(x1, y1, points_x[47], points_y[47]) < MIN_DIS && getDis(x2, y2, points_x[18], points_y[18]) < MIN_DIS)
        return 1;
    if(getDis(x1, y1, points_x[38], points_y[38]) < MIN_DIS && getDis(x2, y2, points_x[46], points_y[46]) < MIN_DIS)
        return 1;
    return 0;
}

/**
 * 判断从(x1, y1)到(x2, y2)是否是type=4的进入环岛动作
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return 1为是 0为非
 */
int isRoundAbout(int x1, int y1, int x2, int y2)
{
    if(getDis(x1, y1, points_x[5], points_y[5]) < MIN_DIS || getDis(x1, y1, points_x[6], points_y[6]) < MIN_DIS || getDis(x1, y1, points_x[7], points_y[7]) < MIN_DIS)
        return 1;
    return 0;
}

CarMove coreGetCarMove(int x1, int y1, int x2, int y2, short angle1, short angle2)
{
#ifdef PRINT_INFO
		printf("core GetCarMove from (%d, %d) to (%d, %d) angle1: %d, angle2: %d \n",x1, y1, x2, y2, angle1, angle2);
#endif
    short delta_angle = angle2 - angle1, turn_angle = 0;
    // int direction = 0;
    float moveDis = 0, turn_r = 0;
    short moveType = 0;
    //
    if(delta_angle % 180 == 0) {
        if(delta_angle == 0) {
            if(isType3Move(x1, y1, x2, y2))
                moveType = 3;
            else moveType = 1;
            moveDis = EuclideanDis(x1, y1, x2, y2);
        }
        else {
            moveType = 2;
            float xx1 = myCos(angle1), yy1 = mySin(angle1);
            float xx2 = x2 - x1, yy2 = y2 - y1;
            if(crossProduct(xx1, yy1, xx2, yy2) > 0) {
                //direction = -1;
                turn_angle = -180;
            }
            else {
                //direction = 1;
                turn_angle = 180;
            }
            turn_r = EuclideanDis(x1, y1, x2, y2) / 2;
        }
    }
    else  {
        if(delta_angle > 0) {
            if(delta_angle > 180) {
                turn_angle = (short)360 - delta_angle;
                // direction = 1;
            }
            else {
                turn_angle = -delta_angle;
                //direction = -1;
            }
        }
        else {
            if(delta_angle < -180) {
                turn_angle = delta_angle + (short)180;
                // direction = -1;
            }
            else {
                turn_angle = -delta_angle;
                // direction = 1;
            }
        }
        float i_point_x, i_point_y;
        computeIntersection(x1, y1,angle1, x2, y2, angle2, &i_point_x, &i_point_y);
        turn_r = EuclideanDis(x1, y1, x2, y2) / 2;
        moveType = 2;
    }
    if(isRoundAbout(x1, y1, x2, y2))
        moveType = 4;
    CarMove carMove;
    carMove.angle = turn_angle; carMove.dis = moveDis; carMove.r = turn_r; carMove.type = moveType;
    carMove.dest_x = x2; carMove.dest_y = y2;
    carMove.start_x = x1; carMove.start_y = y1;
    return carMove;
}

/**
 * 获得从point1 运动到 point2 的方法，point1和point2为一条路的两个端点
 * @param point1 点1的index
 * @param point2
 * @return
 */
CarMove getCarMoveFromPoints(int point1, int point2)
{

    short angle1 = points_angle[point1 - 1], angle2 = points_angle[point2 - 1];
    int x1 = points_x[point1 - 1], x2 = points_x[point2 - 1], y1 = points_y[point1 - 1], y2 = points_y[point2 - 1];
#ifdef PRINT_INFO
		printf("get carMove from point index %d (%d, %d), point %d (%d, %d) ", point1, x1, y1, point2, x2, y2);
#endif
    return coreGetCarMove(x1, y1, x2, y2, angle1, angle2);
}



MoveList find_road(int st_x, int st_y, int ed_x, int ed_y)
{
#ifdef PRINT_INFO
		printf("begin find_road \n");
#endif
	int be_edge_index = use_map[st_x][st_y], ed_edge_index = use_map[ed_x][ed_y];
    int s1 = e_ed[be_edge_index - 1], s2 = e_be[ed_edge_index - 1];
    int haveFirstMove = 0, haveLastMove = 0;
    short angle1, angle2;
    MoveList moveList;

    if(getDis(st_x, st_y, points_x[e_be[be_edge_index - 1]- 1], points_y[e_be[be_edge_index - 1] - 1]) < getDis(st_x, st_y, points_x[e_ed[be_edge_index - 1] - 1], points_y[e_ed[be_edge_index - 1] - 1]))
        angle1 = points_angle[e_be[be_edge_index - 1] - 1];
    else angle1 = points_angle[e_ed[be_edge_index - 1] - 1];
    if(getDis(ed_x, ed_y, points_x[e_be[ed_edge_index - 1] - 1], points_y[e_be[ed_edge_index - 1] - 1]) < getDis(ed_x, ed_y, points_x[e_ed[ed_edge_index - 1] - 1], points_y[e_ed[ed_edge_index - 1] - 1]))
        angle2 = points_angle[e_be[ed_edge_index - 1] - 1];
    else angle2 = points_angle[e_ed[ed_edge_index - 1] - 1];
    if(be_edge_index == ed_edge_index || getDis(st_x, st_y, ed_x, ed_y) <= MIN_DIS) {
        CarMove onlyMove = coreGetCarMove(st_x, st_y, ed_x, ed_y, angle1, angle2);
        moveList = initMoveList(1);
        moveList.data[0] = onlyMove;
    }
    else {
        CarMove firstMove, lastMove;
        if (getDis(st_x, st_y, points_x[s1 - 1], points_y[s1 - 1]) > 5) {
            haveFirstMove = 1;
            firstMove = coreGetCarMove(st_x, st_y, points_x[s1 - 1], points_y[s1 - 1], angle1, points_angle[s1 - 1]);
        }
        if (getDis(ed_x, ed_y, points_x[s2 - 1], points_y[s2 - 1]) > 5) {
            haveLastMove = 1;
            lastMove = coreGetCarMove(points_x[s2 - 1], points_y[s2 - 1], ed_x, ed_y, points_angle[s2 - 1], angle2);
        }
#ifdef PRINT_INFO
				printf("prepare dijkstra \n");
#endif
        EdgeList edgeList = dijkstra(s1, s2);
#ifdef PRINT_INFO
        printf("prepare to init MoveList \n");
#endif
        moveList = initMoveList(edgeList.num + haveFirstMove + haveLastMove);
#ifdef PRINT_INFO
		printf("after initMoveList \n");
#endif
        if (haveFirstMove == 1)
            moveList.data[0] = firstMove;
        if (haveLastMove == 1)
            moveList.data[moveList.num - 1] = lastMove;
        for (int i = 0; i < edgeList.num; ++i) {
            moveList.data[i + haveFirstMove] = getCarMoveFromPoints(edgeList.data[i].a, edgeList.data[i].b);
        }
#ifdef PRINT_INFO
		printf("before delete EdgeList \n");
#endif
        deleteEdgeList(&edgeList);
    }
		
    return moveList;
}

int getNearestPoint(int x1, int y1, int edgeIndex)
{
    int _st = e_be[edgeIndex - 1], _ed = e_ed[edgeIndex - 1];
    if(getDis(x1, y1, points_x[_st - 1], points_y[_st - 1]) < MIN_DIS)
        return _st;
    if(getDis(x1, y1, points_x[_ed - 1], points_y[_ed - 1]) < MIN_DIS)
        return _ed;
    return _ed;
}

int inMap(short x, short y)
{
    if(x >= 0 && y >= 0 && x < map_width && y < map_width)
        return 1;
    return 0;
}

#define vis_map_len 400

int hasVisited(short x, short y, int lo, int hi, short *vis_map_x, short *vis_map_y)
{
    for(int i = lo; i <= hi; ++i)
        if(vis_map_x[i] == x && vis_map_y[i] == y)
            return 1;
    return 0;
}

int findNearsetEdgeIndex(int x1, int y1)
{
    const int queue_len = 100;
    static short p_queue_x[queue_len], p_queue_y[queue_len];
    short ux , uy;
    const short xx[4] = {-1, 0, 1, 0}, yy[4] = {0, 1, 0, -1};
    int h = 0, t = 1;
    int map_lo = 0, map_hi = -1;
    int findFlag = 0, foundindex = 0;
    static short vis_map_x[vis_map_len], vis_map_y[vis_map_len];
    p_queue_x[h] = (short)x1, p_queue_y[h] = (short)y1;
    ++map_hi;
    vis_map_x[map_hi] = x1, vis_map_y[map_hi] = y1;
    while(h < t) {
        if(findFlag)
            break;
        ux = p_queue_x[h], uy = p_queue_y[h];
        h++;
        for(int i = 0; i < 4; ++i) {
            short temp_x = ux + xx[i], temp_y = uy + yy[i];
            if(inMap(temp_x, temp_y) && !hasVisited(temp_x, temp_y, map_lo, map_hi, vis_map_x, vis_map_y)) {
                if(use_map[temp_x][temp_y] != 0) {
                    findFlag = 1;
                    foundindex = use_map[temp_x][temp_y];
                    break;
                }
                ++map_hi;
                vis_map_x[map_hi] = temp_x, vis_map_y[map_hi] = temp_y;
                p_queue_x[t] = temp_x, p_queue_y[t] = temp_y;
                ++t;
            }
        }
    }
    if(findFlag)
        return foundindex;
    return 0;
}


MoveList find_road_with_angle(int st_x, int st_y, int ed_x, int ed_y, short curAngle)
{
#ifdef PRINT_INFO
    printf("begin find_road from (%d, %d) to (%d, %d) cur Angle %d\n", st_x, st_y, ed_x, ed_y, curAngle);
#endif
    int be_edge_index = use_map[st_x][st_y], ed_edge_index = use_map[ed_x][ed_y];
    if(be_edge_index == 0)
        be_edge_index = findNearsetEdgeIndex(st_x, st_y);
    if(be_edge_index == 0)
        printf("Error! cannot find road in (%d, %d) ", st_x, st_y);
    if(ed_edge_index == 0)
        ed_edge_index = findNearsetEdgeIndex(ed_x, ed_y);
    if(ed_edge_index == 0)
        printf("Error! cannot find road in (%d, %d) ", ed_x, ed_y);

		int s1 = getNearestPoint(st_x, st_y, be_edge_index), s2 = e_be[ed_edge_index - 1];
#ifdef PRINT_INFO
		printf("be_edge_index %d ed_edge_index %d s1 index %d, s2 index %d \n", be_edge_index, ed_edge_index, s1, s2);
#endif
    int haveFirstMove = 0, haveLastMove = 0;
    short angle1, angle2;
    MoveList moveList;
    if(curAngle < 0) {
        if (getDis(st_x, st_y, points_x[e_be[be_edge_index - 1] - 1], points_y[e_be[be_edge_index - 1] - 1]) <
            getDis(st_x, st_y, points_x[e_ed[be_edge_index - 1] - 1], points_y[e_ed[be_edge_index - 1] - 1]))
            angle1 = points_angle[e_be[be_edge_index - 1] - 1];
        else angle1 = points_angle[e_ed[be_edge_index - 1] - 1];
    }
    else angle1 = curAngle;
    if(getDis(ed_x, ed_y, points_x[e_be[ed_edge_index - 1] - 1], points_y[e_be[ed_edge_index - 1] - 1]) < getDis(ed_x, ed_y, points_x[e_ed[ed_edge_index - 1] - 1], points_y[e_ed[ed_edge_index - 1] - 1]))
        angle2 = points_angle[e_be[ed_edge_index - 1] - 1];
    else angle2 = points_angle[e_ed[ed_edge_index - 1] - 1];
    if(be_edge_index == ed_edge_index || getDis(st_x, st_y, ed_x, ed_y) <= MIN_DIS) {
#ifdef PRINT_INFO
				printf("prepare to get only car Move from (%d, %d) to (%d, %d) ", st_x, st_y, ed_x, ed_y);
#endif
				CarMove onlyMove = coreGetCarMove(st_x, st_y, ed_x, ed_y, angle1, angle2);
        moveList = initMoveList(1);
        moveList.data[0] = onlyMove;
    }
    else {
        CarMove firstMove, lastMove;
        if (getDis(st_x, st_y, points_x[s1 - 1], points_y[s1 - 1]) > 5) {
            haveFirstMove = 1;
#ifdef PRINT_INFO
				printf("prepare to get first car Move from (%d, %d) to (%d, %d) ", st_x, st_y, points_x[s1 - 1], points_y[s1 - 1]);
#endif					
            firstMove = coreGetCarMove(st_x, st_y, points_x[s1 - 1], points_y[s1 - 1], angle1, points_angle[s1 - 1]);
        }
        if (getDis(ed_x, ed_y, points_x[s2 - 1], points_y[s2 - 1]) > 5) {
            haveLastMove = 1;
#ifdef PRINT_INFO
				printf("prepare to get last car Move from (%d, %d) to (%d, %d) ", points_x[s2 - 1], points_y[s2 - 1], ed_x, ed_y);
#endif
            lastMove = coreGetCarMove(points_x[s2 - 1], points_y[s2 - 1], ed_x, ed_y, points_angle[s2 - 1], angle2);
        }
#ifdef PRINT_INFO
        printf("prepare dijkstra \n");
#endif
        EdgeList edgeList = dijkstra(s1, s2);
#ifdef PRINT_INFO
        printf("prepare to init MoveList \n");
#endif
        moveList = initMoveList(edgeList.num + haveFirstMove + haveLastMove);
#ifdef PRINT_INFO
        printf("after initMoveList \n");
#endif
        if (haveFirstMove == 1)
            moveList.data[0] = firstMove;
        if (haveLastMove == 1)
            moveList.data[moveList.num - 1] = lastMove;
        for (int i = 0; i < edgeList.num; ++i) {
            moveList.data[i + haveFirstMove] = getCarMoveFromPoints(edgeList.data[i].a, edgeList.data[i].b);
        }
#ifdef PRINT_INFO
        printf("before delete EdgeList \n");
#endif
        deleteEdgeList(&edgeList);
    }

    return moveList;
}


void print_move_list(MoveList moveList)
{
    for (int i = 0; i < moveList.num; ++i) {
        printf("type: %d moveDis: %f turn_angle %d dest_x:%d dest_y:%d turn_r: %f\n", moveList.data[i].type,
               moveList.data[i].dis, moveList.data[i].angle, moveList.data[i].dest_x, moveList.data[i].dest_y,
               moveList.data[i].r);
    }
}

CarMove get_next_move(int st_x, int st_y, int ed_x, int ed_y)
{
#ifdef PRINT_INFO
		printf("begin get_next_move \n");
#endif
    MoveList moveList = find_road(st_x, st_y, ed_x, ed_y);
#ifdef PRINT_INFO
    print_move_list(moveList);
    printf("after find_road \n");
#endif

    if(moveList.num > 0) {
        CarMove carMove = moveList.data[0];
        deleteMoveList(&moveList);
#ifdef PRINT_INFO
			printf("after delete moveList \n");
#endif
        return carMove;
    }
    else {
        CarMove carMove;
        carMove.type = 0;
        carMove.dis = 0;
        carMove.angle = 0;
        carMove.dest_x = st_x;
        carMove.dest_y = st_y;
        carMove.start_x = st_x;
        carMove.start_y = st_y;
        return carMove;
    }
}

CarMove GetNextMove(MessageInfo info)
{
		#ifdef PRINT_INFO
			printf("begin getNextMove\n");
		#endif
    int onCarPass = -1, oppoHasPass = -1;
    for(int i = 0; i < info.passengerNum; ++i) {
        if(info.pass_status[i] == 1) {
            if(info.is_a)
                onCarPass = i;
            else oppoHasPass = i;
        }
        else if(info.pass_status[i] == 2) {
            if(info.is_a)
                oppoHasPass = i;
            else onCarPass = i;
        }
    }
    if(onCarPass != -1) {
        return get_next_move(info.my_x, info.my_y, info.xe_pos[onCarPass], info.ye_pos[onCarPass]);
    }
    else {
        short minDis = SHORT_INF;
        int targetPass = 0;
        for(int i = 0; i < info.passengerNum; ++i) {
            short dis1 = getDis(info.my_x, info.my_y, info.xs_pos[i], info.ys_pos[i]);
            if(info.pass_status[i] == 0 && dis1 < minDis) {
                if(oppoHasPass != -1 || dis1 <= getDis(info.oppo_x, info.oppo_y, info.xs_pos[i], info.ys_pos[i])) {
                    minDis = dis1;
                    targetPass = i;
                }
            }
        }
#ifdef PRINT_INFO
				printf("before get_next_move \n");
#endif
        return get_next_move(info.my_x, info.my_y, info.xs_pos[targetPass], info.ys_pos[targetPass]);

    }
}

CarMove GetNextMoveWithAngle(MessageInfo info, short curAngle)
{
#ifdef PRINT_INFO
    printf("begin getNextMove\n");
#endif
    int onCarPass = -1, oppoHasPass = -1;
    for(int i = 0; i < info.passengerNum; ++i) {
        if(info.pass_status[i] == 1) {
            onCarPass = i;
        }
        else if(info.pass_status[i] == 2)
            oppoHasPass = i;
    }
    if(onCarPass != -1) {
        return get_next_move_with_angle(info.my_x, info.my_y, info.xe_pos[onCarPass], info.ye_pos[onCarPass], curAngle);
    }
    else {
        short minDis = SHORT_INF;
        int targetPass = 0;
        for(int i = 0; i < info.passengerNum; ++i) {
            short dis1 = getDis(info.my_x, info.my_y, info.xs_pos[i], info.ys_pos[i]);
            if(info.pass_status[i] == 0 && dis1 < minDis) {
                if(oppoHasPass != -1 || dis1 <= getDis(info.oppo_x, info.oppo_y, info.xs_pos[i], info.ys_pos[i])) {
                    minDis = dis1;
                    targetPass = i;
                }
            }
        }
#ifdef PRINT_INFO
        printf("before get_next_move \n");
#endif
        return get_next_move_with_angle(info.my_x, info.my_y, info.xs_pos[targetPass], info.ys_pos[targetPass], curAngle);

    }
}

MoveList get_moveList_with_angle(int st_x, int st_y, int ed_x, int ed_y, short curAngle)
{
#ifdef PRINT_INFO
    printf("begin get_next_move_list \n");
#endif
    MoveList moveList = find_road_with_angle(st_x, st_y, ed_x, ed_y, curAngle);
    return moveList;
}

MoveList GetMoveListWithAngle(MessageInfo info, short curAngle)
{
#ifdef PRINT_INFO
    printf("begin getNextMove\n");
#endif
    int onCarPass = -1, oppoHasPass = -1;
    for(int i = 0; i < info.passengerNum; ++i) {
        if(info.pass_status[i] == 1) {
            onCarPass = i;
        }
        else if(info.pass_status[i] == 2)
            oppoHasPass = i;
    }
    if(onCarPass != -1) {
        return get_moveList_with_angle(info.my_x, info.my_y, info.xe_pos[onCarPass], info.ye_pos[onCarPass], curAngle);
    }
    else {
        short minDis = SHORT_INF;
        int targetPass = 0;
        for(int i = 0; i < info.passengerNum; ++i) {
            short dis1 = getDis(info.my_x, info.my_y, info.xs_pos[i], info.ys_pos[i]);
            if(info.pass_status[i] == 0 && dis1 < minDis) {
                if(oppoHasPass != -1 || dis1 <= getDis(info.oppo_x, info.oppo_y, info.xs_pos[i], info.ys_pos[i])) {
                    minDis = dis1;
                    targetPass = i;
                }
            }
        }
#ifdef PRINT_INFO
        printf("before get_next_move \n");
#endif
        return get_moveList_with_angle(info.my_x, info.my_y, info.xs_pos[targetPass], info.ys_pos[targetPass], curAngle);

    }
}


CarMove get_next_move_with_angle(int st_x, int st_y, int ed_x, int ed_y, short curAngle)
{
#ifdef PRINT_INFO
    printf("begin get_next_move \n");
#endif
    MoveList moveList = find_road_with_angle(st_x, st_y, ed_x, ed_y, curAngle);
#ifdef PRINT_INFO
    print_move_list(moveList);
    printf("after find_road \n");
#endif

    if(moveList.num > 0) {
        CarMove carMove = moveList.data[0];
        deleteMoveList(&moveList);
#ifdef PRINT_INFO
        printf("after delete moveList \n");
#endif
        return carMove;
    }
    else {
        CarMove carMove;
        carMove.type = 0;
        carMove.dis = 0;
        carMove.angle = 0;
        carMove.dest_x = st_x;
        carMove.dest_y = st_y;
        carMove.start_x = st_x;
        carMove.start_y = st_y;
        return carMove;
    }
}


#ifdef DEBUG_MODE
short minDisMap[max_points_num+1][max_points_num + 1];
int test2D[2][2] = {1, 2, 4, 5};



void FindAllDis()
{
    initDis();
    memcpy(minDisMap, edgeMap, sizeof(edgeMap));
    //printf("%d %d %d %d\n", test2D[0][0], test2D[0][1], test2D[1][0], test2D[1][1]);
    int n = max_points_num;
    for(int k = 1; k <= n; ++k) {
        for (int i = 1; i <= n; ++i)
            for (int j = 1; j <= n; ++j)
                minDisMap[i][j] = min(minDisMap[i][j], minDisMap[i][k] + minDisMap[k][j]);
    }
    /*
    while(1) {
        printf("type in two points\n");
        int x, y;
        scanf("%d %d", &x, &y);
        printf("Dis is %d\n", minDisMap[x][y]);
    }
     */
    FILE *dis_fp = fopen("/users/fanqu/programs/eec/min_dis.txt", "w");
    if(dis_fp == NULL)
        printf("cannot create file\n");
    else {
        fprintf(dis_fp, "{");
        for (int i = 1; i <= n; ++i) {
            for (int j = 1; j <= n; ++j)
                if(!(i == n && j == n))
                    fprintf(dis_fp, "%d, ", minDisMap[i][j]);
                else fprintf(dis_fp, "%d};", minDisMap[i][j]);
            //fprintf(dis_fp, "\n");
        }

    }

}

#endif