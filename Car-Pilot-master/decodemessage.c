#include "decodemessage.h"

void change(int *p, int *q, int *r){
    *q=*q-1;
    *r=*r+1;
    if(*q<0){
        *p=*p+1;
        *q=7;
    }
}

void msgrefresh(char* b, struct _MessageInfo* msg, uint8_t who){
    int p=2,q=3,r=11;
    static unsigned char a[64];
    for(int i = 0; i < 64; ++i)
        a[i] = (unsigned char)b[i];

 //AX AY BX BY
    if(who==1){
        msg->my_x=((a[2]&(1<<7)) << 1)+a[5];
        msg->my_y=((a[2]&(1<<6)) << 2) +a[6];
        msg->oppo_x=((a[2]&(1<<5)) << 3)+a[7];
        msg->oppo_y=((a[2]&(1<<4)) << 4)+a[8];
    }
    else{
         msg->oppo_x = ((a[2] & (1 << 7)) << 1) + a[5];
         //msg->oppo_x=(a[2]&(1<<7))/(1<<7)*256+a[5];
         msg->oppo_y=((a[2]&(1<<6)) << 2) +a[6];
         msg->my_x = ((a[2] & (1 << 5)) << 3)+ a[7];
         //msg->my_x=(a[2]&(1<<5))/(1<<5)*256+a[7];
         msg->my_y=((a[2]&(1<<4)) << 4)+a[8];
    }

 //nSX nSY nEX nEY(N=1:5)
    for(int num=0;num<5;num++){
        msg->xs_pos[num]=((a[p]&(1<<q)) >> q << 8)+a[r];
        change(&p, &q, &r);
        msg->ys_pos[num]=((a[p]&(1<<q)) >> q << 8)+a[r];
        change(&p, &q, &r);
        msg->xe_pos[num]=((a[p]&(1<<q)) >> q << 8)+a[r];
        change(&p, &q, &r);
        msg->ye_pos[num]=((a[p]&(1<<q)) >> q << 8)+a[r];
        change(&p, &q, &r);
    }


//ncondition totalnumber(00 ??? 01A? 10B?)
    msg->passengerNum=(a[9]&252)/4;
    msg->pass_status[0]=a[9]&3;
    msg->pass_status[1]=a[10]&(3<<6);
    msg->pass_status[2]=a[10]&(3<<4);
    msg->pass_status[3]=a[10]&(3<<2);
    msg->pass_status[4]=a[10]&3;

 /*
 //A/Bcheat A/Bscore
 int Acheat,Bcheat,Ascore,Bscore;
 Acheat=a[31];
 Bcheat=a[32];
 Ascore=a[33]*256+a[34];
 Bscore=a[35]*256+a[36];
 */
}

