public class ant{
	public Tour tour = new Tour；
	
	//根据信息素，初始化tour
	public void ant(){
		tour.generateIndividual();
	}
	
	//根据，tour.distance ,nessagemanager
	//更新tour
	public void update(){
		public Tour tour1 = new Tour；
		
		//获取当前时间
		long r1 = System.currentTimeMillis();
		random r = new random(r1);
		
		//起始城市
		int startindex  = r.nextInt(Tourmanager.numberOfCities);
		tour1.setCity(0,TourManager.getCity(startindex))
	 /**
     * 选择下一个城市
     * @param index 需要选择第index个城市了
     * @param messagemanager   全局的信息素信息
     * @param Tour  全局的距离矩阵信息
     */
		for (index = 1;index <= Tourmanager.numberOfCities ,index++){
			double []p;
        p=new double[TourManager.numberOfCities];//下一步要走的城市的选中概率
		
        //计算选中概率所需系数
        double sum=0;
        int currentcity=tour1[index-1];//蚂蚁所处当前城市
		
        //计算公式中的分母部分（为下一步计算选中概率使用）
        for( int i=0; i<TourManager.numberOfCities; i++){
            if(tour1.containsCity(i)==false)//没走过的城市
                sum+=(Math.pow(getmess(currentcity,i), messagemanager.alpha)*
                      Math.pow(1.0/distanceTo(tour1.getCity(i)), messagemanager.beta));
        }
		
        //计算每个城市被选中的概率
        for(int i = 0;i < TourManager.numberOfCities; i++){
            if(tour1.containsCity(i)==false)
				
				//城市走过了，选中概率就是0
                p[i]=0.0;
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
        for(int i=0;i<TourManager.numberOfCities;i++){//每次都是顺序走。。。。。
            sumselect+=p[i];
            if(sumselect>=selectp){
                selectcity=i;
                break;
            }
        }
		
        tour1.setCity(index, TourManager.getCity(index));
		}
        
		tour = tour1.getTour;
    }
} 