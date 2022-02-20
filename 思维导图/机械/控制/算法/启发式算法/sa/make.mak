s : sa\SimulatedAnnealing.class sa\City.class sa\Tour.class sa\TourManager.class
	java sa.SimulatedAnnealing

sa\City.class : City.java
	javac -d . City.java
sa\Tour.class : Tour.java
	javac -d . Tour.java City.java TourManager.java
sa\TourManager.class:TourManager.java
	javac -d . TourManager.java City.java
sa\SimulatedAnnealing.class : SimulatedAnnealing.java
	javac -d . SimulatedAnnealing.java Tour.java City.java TourManager.java
clean :
	rm edit Main ACO ant