#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

typedef struct {
	float theta;  
	int length;
}Line;

typedef struct {
	int x;  
	int y;
	int length;
}Point;

#define A_LINE 	200  
#define B_LINE 	100
#define STEP_LINE 5
#define CIRCLE_QUANTITY 10000
#define ACCELE 5  		//调梯形加减速的加速度
#define PREPARE_TIME 10 //从机械臂初始位置到用户设定起点的时间

//#define TIMER_SET 10000 //arr是7199的话，psc=10000/频率-1


void line(int x0,int y0,int x1,int y1,int v);//算出路径上每点对应的各关节角度
void set_mc_param(float* time,int steps,int max_v);
bool err_judge(int x0,int y0,int x1,int y1);
float *calc_time(int steps, int v);


int *pos_step0 = NULL;  //在line()用来存角度的脉冲量 因为用余弦算的,所以存绝对量比较方便 
int *pos_step1 = NULL;
int *freq0 = NULL;      //每一小步期间的频率
int *freq1 = NULL;
float *time = NULL;       //加减速时的时间划分

Line a_line, b_line;
int main()
{
    int x0, y0, x1, y1, v;

    a_line.length=A_LINE;
    b_line.length=B_LINE;


	do{
        printf("A_LINE 200 B_LINE 100\n");
		printf("请按照x0 y0 x1 y1 speed输入,请输入整数\n");	
    	scanf("%d %d %d %d %d",& x0,& y0,& x1,& y1,& v);
	}
	while(!err_judge(x0, y0, x1, y1));


    line( x0, y0, x1, y1,v);
	printf("\n");
	free(time);							time = NULL;
	free(freq0); free(freq1);			freq0 = NULL;freq1 = NULL;
	free(pos_step0); free(pos_step1);	pos_step0 = NULL;pos_step1 = NULL;
    system("pause");
    return 0;
}

//如果我想让直线为匀速运动的话,我就必须要在等时间内完成line()算出的每段的不均匀的脉冲量
//要将那些脉冲量分别存起来,但是分段是自定义的所以只能malloc
//取用脉冲量/规定的时间就是定时器频率.每过一段就要改一次定时器频率
//如果规定时间内达不成怎么办呢.要先把整段路上的脉冲量都规划好,判断有没有无法达成的量,然后报警或调整最高速度吗?按最大的脉冲量设定规定时间
//在规定时间内是均分还是加减速呢,每段变换的时候好像不妙啊.不对,速度是不会突变的,所以每段的定时器频率不会改动很大.

//输出pos_step0/1 存角度的脉冲量
void line(int x0,int y0,int x1,int y1,int v) 
{
	Point now;  
	float loading,length;
	float theta;//过程变量
	int steps;//分段
    float *time;

    length=sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
	steps=length/STEP_LINE;    //总长度所分的份数 

	if(NULL == (pos_step0=(int *)malloc((steps+1)*sizeof(int))) || NULL == (pos_step1=(int *)malloc((steps+1)*sizeof(int))))
	{
		perror("error...");
		exit(1);
	}

    for(int i=0;i<=steps;i++)//每一截对应的关节角度
    {
        loading=(float)i/steps;
        now.x=(x1-x0)*loading+x0;
        now.y=(y1-y0)*loading+y0;
        printf("(%d,%d)\t",now.x,now.y);  printf("\n");
		now.length=sqrt(now.x*now.x+now.y*now.y);
  
        //算角度
		theta=acos((float)(a_line.length*a_line.length+b_line.length*b_line.length-now.length*now.length)/(2*a_line.length*b_line.length));
		//printf("\ntheta %f",theta*180/3.14); ///
        a_line.theta=atan(now.y/now.x)+acos((float)(a_line.length*a_line.length+now.length*now.length-b_line.length*b_line.length)/(2*a_line.length*now.length));
		b_line.theta=3.14-theta;
        
		pos_step0[i]=a_line.theta*CIRCLE_QUANTITY/6.28;
		pos_step1[i]=b_line.theta*CIRCLE_QUANTITY/6.28;
		//printf("1号电机脉冲绝对量%d \n2号电机脉冲绝对量%d\n\n",pos_step0[i],pos_step1[i]);

	}
	//calc_time()
	time = calc_time( steps,  v);
	set_mc_param(time,steps, v);
}

//梯形加减速下等路段的时间是多少
float* calc_time( int steps,int v)
{
    int S_acc;//加速时的路程
	int i ,Segment,Uniform_stage;
	int *s = NULL;
	float last_time = 0;
	float Uniform_speed = 0;

	S_acc = (float)(v * v) / (2*ACCELE);
	Segment = S_acc / STEP_LINE;

	if(NULL == (s=(int *)calloc(Segment+1,sizeof(int))) )
	{
		perror("error...");
		exit(1);
	}
	if(NULL == (time=(float *)calloc((steps+1),sizeof(float))) )
	{
		perror("error...");
		exit(1);
	}
	if(Segment*2<steps)
	{	
		for ( i = 1;i <= Segment;i++)
		{
			s[i] = STEP_LINE*i;
			time[i] = sqrt((float)(2 * s[i]) / ACCELE) - last_time;
			last_time += time[i];

			time[steps + 1 - i] = time[i];
		}
 		Uniform_stage =steps + 1 - i;
		Uniform_speed=(float)STEP_LINE / v;
		while(i <=  Uniform_stage)
		{
			time[i++] = Uniform_speed;
		}
		time[0] = PREPARE_TIME;
	}
	else
	{
		printf("速度太大了\n");	
		system("pause");
	}
 

	free(s);			
	s = NULL;
	return time;
}



void set_mc_param(float* time,int steps,int max_v)
{
	
	int last_pos0 = 0, last_pos1 = 0;



	if(NULL == (freq0=(int *)malloc((steps+1)*sizeof(int))) || NULL == (freq1=(int *)malloc((steps+1)*sizeof(int))))
	{
		perror("error...");
		exit(1);
	}
	for (int i = 0; i <= steps;i++)
	{
		//机械臂初始状态得是0啊 然后会先去起点
		freq0[i] = (pos_step0[i]-last_pos0)/time[i];//频率=每time秒要走()个脉冲 这个公式的前提是定时器按我车库代码那样设定
		freq1[i] = (pos_step1[i]-last_pos1)/time[i];
		//printf("1号电机第%d端频率%d \n2号电机第%d端频率%d\n\n",i,freq0[i],i,freq1[i]);
		last_pos0 = pos_step0[i];
		last_pos1 = pos_step1[i];
		//PSC[i]=TIME_SET/freq0[i]-1;
		printf("这段路要消耗的时间%f\n\n",time[i]);
		printf("1号电机第%d段频率%d \n2号电机第%d段频率%d\n\n",i,freq0[i],i,freq1[i]);
		printf("1号电机脉冲绝对量%d \n2号电机脉冲绝对量%d\n\n",pos_step0[i],pos_step1[i]);

	}

}

bool err_judge(int x0,int y0,int x1,int y1)
{
	if(x0*x0+y0*y0>(A_LINE+B_LINE)*(A_LINE+B_LINE) || x1*x1+y1*y1>(A_LINE+B_LINE)*(A_LINE+B_LINE))
	{
		printf("超出运行范围\n");	
		return false;
	}
    else if (x0*x0+y0*y0<(a_line.length-b_line.length)*(a_line.length-b_line.length) || x1*x1+y1*y1<(a_line.length-b_line.length)*(a_line.length-b_line.length))
	{
		printf("坐标点在运行盲区\n");	
		return false;
	}
	else if ((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0)<STEP_LINE*STEP_LINE)
	{
		printf("运行距离过短\n");	
		return false;
	}
	return true;
}
