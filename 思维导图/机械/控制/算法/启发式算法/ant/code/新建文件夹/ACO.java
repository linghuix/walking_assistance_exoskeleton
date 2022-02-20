public class ACO{
	
	public static void main(String[] args) {
		 City city = new City(60, 200);
         TourManager.addCity(city);
         City city2 = new City(180, 200);
         TourManager.addCity(city2);
         City city3 = new City(80, 180);
         TourManager.addCity(city3);
         City city4 = new City(140, 180);
         TourManager.addCity(city4);
         City city5 = new City(20, 160);
         TourManager.addCity(city5);
         City city6 = new City(100, 160);
         TourManager.addCity(city6);
         City city7 = new City(200, 160);
         TourManager.addCity(city7);
         City city8 = new City(140, 140);
         TourManager.addCity(city8);
         City city9 = new City(40, 120);
         TourManager.addCity(city9);
         City city10 = new City(100, 120);
         TourManager.addCity(city10);
         City city11 = new City(180, 100);
         TourManager.addCity(city11);
         City city12 = new City(60, 80);
         TourManager.addCity(city12);
         City city13 = new City(120, 80);
         TourManager.addCity(city13);
         City city14 = new City(180, 60);
         TourManager.addCity(city14);
         City city15 = new City(20, 40);
         TourManager.addCity(city15);
         City city16 = new City(100, 40);
         TourManager.addCity(city16);
         City city17 = new City(200, 40);
         TourManager.addCity(city17);
         City city18 = new City(20, 20);
         TourManager.addCity(city18);
         City city19 = new City(60, 20);
         TourManager.addCity(city19);
         City city20 = new City(160, 20);
         TourManager.addCity(city20);
		 
		public ArrayList Ant = new ArrayList<ant>();
		ant ant1 = new ant();
        Ant.addCity(ant1);
		ant ant2 = new ant();
        Ant.addCity(ant2);	 
		ant ant3 = new ant();
        Ant.addCity(ant3);	
		
		for(j=0; j<200; ++j){
			double best = 100000;
			for (i=0;1<3;++i){
				if(best > Ant[i].tour.getDistance){
					best = Ant[i].tour.getDistance;
				}
			}
			
			for (i=0;1<3;++i){
				Ant[i].update()
			}
			
		}
		
		System.out.println("Final distance: " + best);
	}
}