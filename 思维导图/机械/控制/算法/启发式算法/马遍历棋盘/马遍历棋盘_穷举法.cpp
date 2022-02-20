#include<iostream>

using namespace std;

//坐标固定的马有八种走的方式
//用数组进行存储，方便在for中使用
int fx[8]= {1,2,2,1,-1,-2,-2,-1};
int fy[8]= {2,1,-1,-2,-2,-1,1,2};

static int mCount;
const static int n=5,m=4;
int a[n][m];  //用int二维数组来表示走的路劲

void mFind(int x,int y,int dep);//寻找路径的递归
int mCheck(int x,int y);//判断坐标是否出界，是否已经走过
void output();//打印路径

int main()
{
    int x=0,y=0;//选择(0,0)为初始点
    mCount=0;
    for(int i=0; i<n; i++)
        for(int j=0; j<m; j++)
            a[i][j]=0;
    a[x][y]=1;

    mFind(x,y,2);

    if(mCount==0)
        cout<<"Non solution"<<endl;
    else
        cout<<endl<<"final count = "<<mCount<<endl;
    getchar(); 

}

void mFind(int x,int y,int dep)
{
    int xx,yy;
    for(int i=0; i<8; i++)
    {
        xx=x+fx[i];
        yy=y+fy[i];
        if(mCheck(xx,yy)==1)
        {
            a[xx][yy]=dep;
            if(dep==n*m)
                output();    //如果深度为n*m，那么表示遍历结束，就打印
            else
                mFind(xx,yy,dep+1);
            a[xx][yy]=0;     //回溯，恢复未走坐标。（如果走错，要将走错的路径还原）
        }
    }
}

int mCheck(int x,int y)
{
    //判断坐标是否出界，是否已经走过
    if(x<0||y<0||x>=n||y>=m||a[x][y]!=0)
        return 0;
    return 1;
}

void output()
{
    mCount++;
    cout<<endl;
    cout<<"count= "<<mCount;
    for(int i=0; i<n; i++)
    {
        cout<<endl;
        for(int j=0; j<m; j++)
            cout<<a[i][j]<<"\t";
    }
}

