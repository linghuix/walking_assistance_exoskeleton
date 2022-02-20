
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.util.*;


//Main Class


public class GreedSnake extends KeyAdapter{


JFrame mainFrame;
Canvas paintCanvas;
JLabel labelScore;//计分牌
SnakeModel snakeModel=null;// 蛇
public static final int DEFAULT_WIDTH=500;
public static final int DEFAULT_HEIGHT=300;
public static final int nodeWidth=10;
public static final int nodeHeight=10;


//GreedSnake():初始化游戏界面



public GreedSnake(){


//设置界面元素
mainFrame = new JFrame("贪吃蛇游戏");
Container cp=mainFrame.getContentPane();
labelScore=new JLabel("所得分数为:",JLabel.CENTER);
cp.add(labelScore,BorderLayout.NORTH);
paintCanvas=new Canvas();
paintCanvas.setSize(DEFAULT_WIDTH+1,DEFAULT_HEIGHT+1);
paintCanvas.addKeyListener(this);
cp.add(paintCanvas,BorderLayout.CENTER);
JPanel panelButtom=new JPanel();
panelButtom.setLayout(new BorderLayout());
JLabel labelHelp;// 帮助信息
labelHelp=new JLabel("按 PageUP 或 PageDown 键改变速度",JLabel.CENTER);
panelButtom.add(labelHelp,BorderLayout.NORTH);
labelHelp=new JLabel("按 Enter 或 S 键重新开始游戏",JLabel.CENTER);
panelButtom.add(labelHelp,BorderLayout.CENTER);
labelHelp=new JLabel("按 SPACE 键或 P 键暂停游戏",JLabel.CENTER);
panelButtom.add(labelHelp,BorderLayout.SOUTH);
cp.add(panelButtom,BorderLayout.SOUTH);
mainFrame.addKeyListener(this);
mainFrame.pack();
mainFrame.setResizable(false);//设置窗口大小不能变化
mainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
mainFrame.setVisible(true);
begin();
}


//keyPressed():按键检测


public void keyPressed(KeyEvent e){


int keyCode=e.getKeyCode();
if(snakeModel.running)
switch(keyCode){
case KeyEvent.VK_UP:
snakeModel.changeDirection(SnakeModel.UP);
break;
case KeyEvent.VK_DOWN:
snakeModel.changeDirection(SnakeModel.DOWN);
break;
case KeyEvent.VK_LEFT:
snakeModel.changeDirection(SnakeModel.LEFT);
break;
case KeyEvent.VK_RIGHT:
snakeModel.changeDirection(SnakeModel.RIGHT);
break;
case KeyEvent.VK_ADD:
case KeyEvent.VK_PAGE_UP:
snakeModel.speedUp();// 加速
break;
case KeyEvent.VK_SUBTRACT:
case KeyEvent.VK_PAGE_DOWN:
snakeModel.speedDown();// 减速
break;
case KeyEvent.VK_SPACE:
case KeyEvent.VK_P:
snakeModel.changePauseState();// 暂停或继续
break;
default:
}
//重新开始
if(keyCode==KeyEvent.VK_S || keyCode==KeyEvent.VK_ENTER){
snakeModel.running=false;
begin();
}
}


//repaint（）：绘制游戏界面（包括蛇和食物）


void repaint(){
Graphics g=paintCanvas.getGraphics();
//draw background
g.setColor(Color.LIGHT_GRAY);
g.fillRect(0,0,DEFAULT_WIDTH,DEFAULT_HEIGHT);
//draw the snake
g.setColor(Color.BLACK);
LinkedList na=snakeModel.nodeArray;
Iterator it=na.iterator();
while(it.hasNext()){


Node n=(Node)it.next();
drawNode(g,n);
}
// draw the food
g.setColor(Color.RED);
Node n=snakeModel.food;
drawNode(g,n);
updateScore();
}


//drawNode（）：绘画某一结点（蛇身或食物）


private void drawNode(Graphics g,Node n){
g.fillRect(n.x*nodeWidth,n.y*nodeHeight,nodeWidth-1,nodeHeight-1);
}


//updateScore（）：改变计分牌


public void updateScore(){
String s="所得分数为: "+snakeModel.score;
labelScore.setText(s);
}


//begin（）：游戏开始，放置贪吃蛇


void begin(){


if(snakeModel==null||!snakeModel.running){
snakeModel=new SnakeModel(this,DEFAULT_WIDTH/nodeWidth,
DEFAULT_HEIGHT/nodeHeight);
(new Thread(snakeModel)).start();
}
}


//main（）：主函数


public static void main(String[] args){


GreedSnake gs=new GreedSnake();
}
}


//Node:结点类


class Node{
int x;
int y;
Node(int x,int y){
this.x=x;
this.y=y;
}
}


//SnakeModel:贪吃蛇模型


class SnakeModel implements Runnable{


GreedSnake gs;
boolean[][] matrix;// 界面数据保存在数组里
LinkedList nodeArray=new LinkedList();
Node food;
int maxX;//最大宽度
int maxY;//最大长度
int direction=2;//方向
boolean running=false;
int timeInterval=200;// 间隔时间（速度）
double speedChangeRate=0.75;// 速度改变程度
boolean paused=false;// 游戏状态
int score=0;
int countMove=0;
// UP和DOWN是偶数，RIGHT和LEFT是奇数
public static final int UP=2;
public static final int DOWN=4;
public static final int LEFT=1;
public static final int RIGHT=3;


//GreedModel():初始化界面


public SnakeModel(GreedSnake gs,int maxX,int maxY){


this.gs=gs;
this.maxX=maxX;
this.maxY=maxY;
matrix=new boolean[maxX][];
for(int i=0;i<maxX;++i){
matrix[i]=new boolean[maxY];
Arrays.fill(matrix[i],false);// 没有蛇和食物的地区置false
}
//初始化贪吃蛇
int initArrayLength=maxX>20 ? 10 : maxX/2;
for(int i=0;i<initArrayLength;++i){
int x=maxX/2+i;
int y=maxY/2;
nodeArray.addLast(new Node(x,y));
matrix[x][y]=true;// 蛇身处置true
}
food=createFood();
matrix[food.x][food.y]=true;// 食物处置true
}


//changeDirection():改变运动方向


public void changeDirection(int newDirection){


if(direction%2!=newDirection%2){// 避免冲突
direction=newDirection;
}
}


//moveOn():贪吃蛇运动函数


public boolean moveOn(){


Node n=(Node)nodeArray.getFirst();
int x=n.x;
int y=n.y;
switch(direction){
case UP:
y--;
break;
case DOWN:
y++;
break;
case LEFT:
x--;
break;
case RIGHT:
x++;
break;
}
if((0<=x&&x<maxX)&&(0<=y&&y<maxY)){


if(matrix[x][y]){// 吃到食物或者撞到身体


if(x==food.x&&y==food.y){// 吃到食物


nodeArray.addFirst(food);// 在头部加上一结点

//计分规则与移动长度和速度有关
int scoreGet=(10000-200*countMove)/timeInterval;
score+=scoreGet>0 ? scoreGet : 10;
countMove=0;
food=createFood();

matrix[food.x][food.y]=true;
return true;
}
else return false;// 撞到身体
}
else{//什么都没有碰到
nodeArray.addFirst(new Node(x,y));// 加上头部
matrix[x][y]=true;
n=(Node)nodeArray.removeLast();// 去掉尾部
matrix[n.x][n.y]=false;
countMove++;
return true;
}
}
return false;//越界（撞到墙壁）
}


/*发了一份贪吃蛇游戏的代码，谁能帮我解释一下，贪吃蛇程序的运行过程，运行步骤*/
//run():贪吃蛇运动线程


public void run(){
running=true;
while(running){
try{
Thread.sleep(timeInterval);
}
catch(Exception e){
break;
}
if(!paused){


if(moveOn()){// 未结束
gs.repaint();
}
else{//游戏结束
JOptionPane.showMessageDialog(null,"GAME OVER",
"Game Over",JOptionPane.INFORMATION_MESSAGE);
break;
}
}
}
running=false;
}


//createFood():生成食物及放置地点


private Node createFood(){


int x=0;
int y=0;
do{
Random r=new Random();
x=r.nextInt(maxX);
y=r.nextInt(maxY);
}
while(matrix[x][y]);
return new Node(x,y);
}


//speedUp():加快蛇运动速度


public void speedUp(){
timeInterval*=speedChangeRate;
}


//speedDown():放慢蛇运动速度


public void speedDown(){


timeInterval/=speedChangeRate;
}


//changePauseState(): 改变游戏状态（暂停或继续）


public void changePauseState(){


paused=!paused;
}
}