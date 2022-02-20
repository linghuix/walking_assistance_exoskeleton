import java.io.*;
/**
 *蚁群优化算法，用来求解TSP问题
 */
public class ACO {
   
    ant []ants; //定义蚂蚁群
    int antcount;//蚂蚁的数量
    int [][]distance;//表示城市间距离
    double [][]tao;//信息素矩阵
    int citycount;//城市数量
    int[]besttour;//求解的最佳路径
    int bestlength;//求的最优解的长度
	
/**
 * 读取数据文件
 * 同时完成蚂蚁类，城市坐标，信息素的初始化工作
 * @param filename tsp数据文件
 * @param antnum 系统用到蚂蚁的数量
 */
    //
    //
    public void init(String filename,int antnum) throws FileNotFoundException, IOException{
        antcount=antnum;
        ants=new ant[antcount];
        //读取数据tsp里的数据包括第I个城市与城市的X,Y坐标
        int[] x;
        int[] y;
        String strbuff;
        BufferedReader tspdata = new BufferedReader(new InputStreamReader(new FileInputStream(filename)));
        strbuff = tspdata.readLine();//读取第一行，城市总数（按文件格式读取）
        citycount = Integer.valueOf(strbuff);
        distance = new int[citycount][citycount];
        x = new int[citycount];
        y = new int[citycount];
        for (int citys = 0; citys < citycount; citys++) {
            strbuff = tspdata.readLine();
            String[] strcol = strbuff.split(" ");
            x[citys] = Integer.valueOf(strcol[1]);//读取每排数据的第2二个数字即横坐标
            y[citys] = Integer.valueOf(strcol[2]);
        }
        //计算两个城市之间的距离矩阵，并更新距离矩阵
        for (int city1 = 0; city1 < citycount - 1; city1++) {
            distance[city1][city1] = 0;
            for (int city2 = city1 + 1; city2 < citycount; city2++) {
                distance[city1][city2] = (int) (Math.sqrt((x[city1] - x[city2]) * (x[city1] - x[city2])
                        + (y[city1] - y[city2]) * (y[city1] - y[city2])));
                distance[city2][city1] = distance[city1][city2];//距离矩阵是对称矩阵
            }
        }
        distance[citycount - 1][citycount - 1] = 0;
        //初始化信息素矩阵
        tao=new double[citycount][citycount];
        for(int i=0;i<citycount;i++)
        {
            for(int j=0;j<citycount;j++){
                tao[i][j]=0.1;
            }
        }
        bestlength=Integer.MAX_VALUE;
        besttour=new int[citycount+1];
        //随机放置蚂蚁
        for(int i=0;i<antcount;i++){
            ants[i]=new ant();
            ants[i].RandomSelectCity(citycount);
        }
    }
/**
 * 计算蚂蚁总的路径长度，更新信息素矩阵，最佳路径长度
 * 同时重新随机设置蚂蚁进行下一次循环
 * @param maxgen ACO的最多循环次数
 */
    public void run(int maxgen){
        for(int runtimes=0;runtimes<maxgen;runtimes++){
            //每次迭代，所有蚂蚁都要更新一遍，走一遍
            //System.out.print("no>>>"+runtimes);
            //每一只蚂蚁移动的过程
            for(int i=0;i<antcount;i++){
                for(int j=1;j<citycount;j++){
                    ants[i].SelectNextCity(j,tao,distance);//每只蚂蚁的城市规划
                }
                //计算蚂蚁获得的路径长度
                ants[i].CalTourLength(distance);
                if(ants[i].tourlength<bestlength){
                    //保留最优路径
                    bestlength=ants[i].tourlength;
                    //runtimes仅代表最大循环次数，但是只有当，有新的最优路径的时候才会显示下列语句。
                    //如果后续没有更优解（收敛），则最后直接输出。
                    System.out.println("第"+runtimes+"代(次迭代)，发现新的最优路径长度："+bestlength);
                    for(int j=0;j<citycount+1;j++)
                        besttour[j]=ants[i].tour[j];//更新路径
                }
            }
            //更新信息素矩阵
            UpdateTao();
            //重新随机设置蚂蚁
            for(int i=0;i<antcount;i++){
                ants[i].RandomSelectCity(citycount);
            }
        }
       }
    /**
     * 更新信息素矩阵
     */
    private void UpdateTao(){
        double rou=0.5;
        //信息素挥发
        for(int i=0;i<citycount;i++)
            for(int j=0;j<citycount;j++)
                tao[i][j]=tao[i][j]*(1-rou);
        //信息素更新
        for(int i=0;i<antcount;i++){
            for(int j=0;j<citycount;j++){
                tao[ants[i].tour[j]][ants[i].tour[j+1]]+=1.0/ants[i].tourlength;//所有蚂蚁走过时产生的信息素
            }
        }
    }
    /* 输出程序运行结果
     */
    public void ReportResult(){
        System.out.println("最优路径长度是"+bestlength);
        System.out.println("蚁群算法最优路径输出："); 
		System.out.print(besttour[0]);
        for(int j=1;j<citycount+1;j++)
            System.out.print(">>"+besttour[j]);//输出最优路径
		
    }
}