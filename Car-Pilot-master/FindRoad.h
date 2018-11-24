//
// Created by Fan Qu on 10/31/18.
//

#ifndef FINDROAD_H
#define FINDROAD_H

//#define DEBUG_MODE
#define PRINT_INFO

#ifndef MY_PI
#define MY_PI 3.141592654F
#endif

#define CARMOVE_POOL_LEN 100
#define EDGE_POOL_LEN 100

/**
 * 操作类型
 * type代表类型,type 为1代表直行，type为0代表停车， type为2代表转弯
 * 如果type为2， 则r代表转弯半径，angle代表转弯角度，角度为-180~180度之间，负数代表左转，正数为右转
 * dis为直行距离
 * dest_x代表这一动作的目的地x坐标, dest_y代表目的y坐标
 * start_x为起始x，start_y为起始y
 */
struct _CarMove{
    float dis,r;
    int dest_x, dest_y, start_x, start_y;
    short type, angle;

};

typedef struct _CarMove CarMove;


struct _move_list{
    int num;
    CarMove *data;
		//CarMove data[CARMOVE_POOL_LEN];
};
typedef struct _move_list MoveList;

struct _Edge{
    int a, b, w;
};

typedef struct _Edge Edge;

struct _edge_list{
    int num;
    Edge* data;
};

/**
 * 通信协议提取信息。
 *
 */
struct _MessageInfo{
    int passengerNum; //乘客数
    int xs_pos[5], ys_pos[5]; //乘客的起点x坐标和y坐标
    int xe_pos[5], ye_pos[5];
    int my_x, my_y, oppo_x, oppo_y; //我的x,y坐标和对手坐标
    char pass_status[7]; //乘客状态0为未上车，1为在我的车,2为在对手车
    char is_a; //当is_a为0时我的车是B车,当is_a==1时我的车是A车
};

typedef struct _MessageInfo MessageInfo;

typedef struct _edge_list EdgeList;

/**
 * 获取从(st_x, st_y)坐标到(ed_x, ed_y)的MoveList
 * @param st_x
 * @param st_y
 * @param ed_x
 * @param ed_y
 * @return
 */
MoveList find_road(int st_x, int st_y, int ed_x, int ed_y);

CarMove get_next_move(int st_x, int st_y, int ed_x, int ed_y);

CarMove get_next_move_with_angle(int st_x, int st_y, int ed_x, int ed_y, short curAngle);


CarMove GetNextMove(MessageInfo info);

CarMove GetNextMoveWithAngle(MessageInfo info, short curAngle);

#ifdef DEBUG_MODE
void FindAllDis();
#endif

EdgeList dijkstra(int st, int ed);

void deleteEdgeList(EdgeList *list);

void deleteMoveList(MoveList *moveList);



#endif //FINDROAD_H
