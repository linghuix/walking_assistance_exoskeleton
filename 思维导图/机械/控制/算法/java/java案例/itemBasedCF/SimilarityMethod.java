package itemBasedCF;
/**
 * an interface of SimilarityMethod
 * @author Wang Menghan
 * @version 1.0 
 * @Time 2015-11-4 15:48:39
 */
public interface SimilarityMethod{
	
    public double[][] calSimilarity(double [][]uiRating);
	
}