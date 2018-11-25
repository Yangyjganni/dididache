#include <stdio.h>
#include <string.h>
#include "index_map.h"
#include "FindRoad.h"
#include "decodemessage.h"


void testDij()
{
    printf("enter in two points\n");
    int a, b;
    scanf("%d%d", &a, &b);
    EdgeList pathList = dijkstra(a, b);
    printf("%d ", a);
    for(int i = 0; i < pathList.num; ++i) {
        printf("%d ", pathList.data[i].b);
    }
    printf("\n");
    deleteEdgeList(&pathList);
}

void testFindRoad()
{
    int x1, x2, y1, y2;
    do {
        printf("enter in two points x and y\n");

        scanf("%d%d%d%d", &x1, &y1, &x2, &y2);
        /*
        MoveList moveList = find_road(x1, y1, x2, y2);
        for (int i = 0; i < moveList.num; ++i) {
            printf("type: %d moveDis: %f turn_angle %d dest_x:%d dest_y:%d turn_r: %f\n", moveList.data[i].type,
                   moveList.data[i].dis, moveList.data[i].angle, moveList.data[i].dest_x, moveList.data[i].dest_y,
                   moveList.data[i].r);
        }
         */
        while(x1 != x2 && y1 != y2) {
            CarMove curMove = get_next_move(x1, y1, x2, y2);
            printf("type: %d moveDis: %f turn_angle %d start_x:%d start_y:%d dest_x:%d dest_y:%d turn_r: %f\n", curMove.type,
                   curMove.dis, curMove.angle, curMove.start_x, curMove.start_y, curMove.dest_x, curMove.dest_y, curMove.r);
            x1 = curMove.dest_x; y1 = curMove.dest_y;
        }
    }while(x1 >= 0);
}

void testFwrite()
{
    CarMove c1;
    memset(&c1, 0, sizeof(c1));
    c1.dest_x = 2;
    c1.dis = 1.23;
    FILE *test_fp = fopen("./dump.txt", "w");
    if(test_fp != NULL)
        fwrite(&c1, 1, sizeof(c1), test_fp);
    else printf("can not create file");
}

void testDecodeMessage()
{
    MessageInfo msg;
    CarMove move;
    msg.my_x = 187, msg.my_y = 58;
    msg.oppo_x = 17, msg.oppo_y = 270;
    msg.passengerNum = 2;
    memset(msg.pass_status, 0, 7);
    msg.is_a = 1;
    msg.pass_status[2] = 0;
    msg.xs_pos[0] = 54, msg.ys_pos[0] = 170;
    msg.xe_pos[0] = 117, msg.ye_pos[0] = 133;

    msg.xs_pos[1] = 41, msg.ys_pos[1] = 12;
    msg.xe_pos[1] = 146, msg.ye_pos[1] = 30;

    msg.xs_pos[2] = 105, msg.ys_pos[2] = 238;
    msg.xe_pos[2] = 174, msg.ye_pos[2] = 149;

    msg.xs_pos[3] = 238, msg.ys_pos[3] = 105;
    msg.xe_pos[3] = 149, msg.ye_pos[3] = 174;

    msg.xs_pos[4] = 8, msg.ys_pos[4] = 225;
    msg.xe_pos[4] = 113, msg.ye_pos[4] = 96;
    while(1) {

        move = GetNextMoveWithAngle(msg, -1);
        msg.my_x = move.dest_x, msg.my_y = move.dest_y;
        printf("\n");
        printf("type %d dest_x %d dest_y %d angle %d move.dis %f\n", move.type,  move.dest_x, move.dest_y,  move.angle, move.dis);
    }

}



int main() {
    //FindAllDis();
    //testFwrite();
    //testFindRoad();
    //testDij();
    testDecodeMessage();
    printf("END\n");
    //printf("Hello, World!\n");
    return 0;
}