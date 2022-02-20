edit : Main.class ACO.class ant.class
	java Main ant ACO

Main.class : Main.java
	javac Main.java
ACO.class : ACO.java
	javac ACO.java
ant.class : ant.java
	javac ant.java
clean :
	rm edit Main ACO ant