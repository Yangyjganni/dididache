//
// Created by Fan Qu on 10/31/18.
//

#include "FindRoad.h"
#include "index_map.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

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
		printf("prepare to malloc %d of CarMove", num);
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

CarMove coreGetCarMove(int x1, int y1, int x2, int y2, short angle1, short angle2)
{
    short delta_angle = angle2 - angle1, turn_angle = 0;
    // int direction = 0;
    float moveDis = 0, turn_r = 0;
    short moveType = 0;
    //
    if(delta_angle % 180 == 0) {
        if(delta_angle == 0) {
            moveType = 1;
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
    return coreGetCarMove(x1, y1, x2, y2, angle1, angle2);
}

#define MIN_DIS 5

MoveList find_road(int st_x, int st_y, int ed_x, int ed_y)
{
#ifdef PRINT_INFO
		printf("begin find_road\n");
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
				printf("prepare dijkstra");
#endif
        EdgeList edgeList = dijkstra(s1, s2);
				#ifdef PRINT_INFO
				printf("prepare to init MoveList\n");
				#endif
        moveList = initMoveList(edgeList.num + haveFirstMove + haveLastMove);
				#ifdef PRINT_INFO
				printf("after initMoveList");
				#endif
        if (haveFirstMove == 1)
            moveList.data[0] = firstMove;
        if (haveLastMove == 1)
            moveList.data[moveList.num - 1] = lastMove;
        for (int i = 0; i < edgeList.num; ++i) {
            moveList.data[i + haveFirstMove] = getCarMoveFromPoints(edgeList.data[i].a, edgeList.data[i].b);
        }
				#ifdef PRINT_INFO
				printf("before delete EdgeList");
				#endif
        deleteEdgeList(&edgeList);
    }
		
    return moveList;
}

CarMove get_next_move(int st_x, int st_y, int ed_x, int ed_y)
{
#ifdef PRINT_INFO
		printf("begin get_next_move");
#endif
    MoveList moveList = find_road(st_x, st_y, ed_x, ed_y);
#ifdef PRINT_INFO
		printf("after find_road");
#endif

    if(moveList.num > 0) {
        CarMove carMove = moveList.data[0];
        deleteMoveList(&moveList);
#ifdef PRINT_INFO
			printf("after delete moveList\n");
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
            onCarPass = i;
        }
        else if(info.pass_status[i] == 2)
            oppoHasPass = i;
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
				printf("before get_next_move\n");
#endif
        return get_next_move(info.my_x, info.my_y, info.xs_pos[targetPass], info.ys_pos[targetPass]);

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