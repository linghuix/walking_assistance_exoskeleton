import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
//蚁群算法求解旅行商问题，TSP数据来源
//http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/
//数据中包括城市总量，每个城市的横纵坐标
public class Main {
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        ACO aco;
        aco=new ACO();
        try {
            aco.init("att.txt", 100);//城市信息文件，蚂蚁数量
            aco.run(1000);//迭代次数
            aco.ReportResult();
        } catch (FileNotFoundException ex) {
            Logger.getLogger(Main.class.getName()).log(Level.SEVERE, null, ex);
        } catch (IOException ex) {
            Logger.getLogger(Main.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
}