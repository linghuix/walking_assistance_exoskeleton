public class messagemanager{
	private static double message = new double [][];
	private static double alpha = 1.0;
    private static double beta = 2.0;
	
	//初始化信息素
	public static void messagemanager(){
        for (i = 0; i < TourManager.numberOfCities; i++){
			 for (j = 0; j < TourManager.numberOfCities; j++){
				message[i][j] = 0.1; 
			}
		}
     }
	 
	// 更新信息素放在主函数中，因为需要用到Tour类
	
	public static double getmess(i,j){
		return message[i][j];
	}
}