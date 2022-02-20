import java.util.Random;
/*
 蚂蚁类
 */
public class ant {
    /**
     * 蚂蚁获得的路径
     */
    public int[]tour;//记录参观城市顺序
    
    int[] unvisitedcity;//unvisitedcity 取值是0或1，1表示没有访问过，0表示访问过
    /**
     * 蚂蚁获得的路径长度
     */
    public int tourlength;//某蚂蚁所走路程总长度。
    int citys;//城市个数
/**
 * 随机分配蚂蚁到某个城市中
 * 同时完成蚂蚁包含字段的初始化工作
 * @param citycount 总的城市数量
 */
    public void RandomSelectCity(int citycount){
        citys=citycount;
        unvisitedcity=new int[citycount];
        tour=new int[citycount+1];
        tourlength=0;
        for(int i=0;i<citycount;i++){
            tour[i]=-1;
            unvisitedcity[i]=1;
        }//初始化各个变量
        
        long r1 = System.currentTimeMillis();//获取当前时间
        Random rnd = new Random(r1);
        int firstcity = rnd.nextInt(citycount);//随机指定第一个城市
        unvisitedcity[firstcity]=0;//0表示访问过
        tour[0]=firstcity;//起始城市
    }
    /**
     * 选择下一个城市
     * @param index 需要选择第index个城市了
     * @param tao   全局的信息素信息
     * @param distance  全局的距离矩阵信息
     */
    public void SelectNextCity(int index,double[][]tao,int[][]distance){
        double []p;
        p=new double[citys];//下一步要走的城市的选中概率
        //计算选中概率所需系数。
        double alpha=1.0;
        double beta=2.0;
        double sum=0;
        int currentcity=tour[index-1];//蚂蚁所处当前城市
        //计算公式中的分母部分（为下一步计算选中概率使用）
        for(int i=0;i<citys;i++){
            if(unvisitedcity[i]==1)//没走过的城市
                sum+=(Math.pow(tao[currentcity][i], alpha)*
                        Math.pow(1.0/distance[currentcity][i], beta));
        }
        //计算每个城市被选中的概率
        for(int i=0;i<citys;i++){
            if(unvisitedcity[i]==0)
                p[i]=0.0;//城市走过了，选中概率就是0
            else{
                //没走过，下一步要走这个城市的概率是？
                p[i]=(Math.pow(tao[currentcity][i], alpha)*
                        Math.pow(1.0/distance[currentcity][i], beta))/sum;
            }
        }
        long r1 = System.currentTimeMillis();
        Random rnd=new Random(r1);
        double selectp=rnd.nextDouble();
        //轮盘赌选择一个城市；
        double sumselect=0;
        int selectcity=-1;
        //城市选择随机，直到n个概率加起来大于随机数，则选择该城市
        for(int i=0;i<citys;i++){//每次都是顺序走。。。。。
            sumselect+=p[i];
            if(sumselect>=selectp){
                selectcity=i;
                break;
            }
        }
        if (selectcity==-1)//这个城市没有走过
            System.out.println();
        tour[index]=selectcity;
        unvisitedcity[selectcity]=0;
    }
    /**
     * 计算蚂蚁获得的路径的长度
     * @param distance  全局的距离矩阵信息
     */
    public void CalTourLength(int [][]distance){
        tourlength=0;
        tour[citys]=tour[0];//第一个城市等于最后一个要到达的城市
        for(int i=0;i<citys;i++){
            tourlength+=distance[tour[i]][tour[i+1]];//从A经过每个城市仅一次，最后回到A的总长度。
        }    
    }
}