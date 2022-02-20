package itemBasedCF;

/**
 * Describes the Adjusted Cosine Similarity measure to be used by a ModelBuilder to build an item-based model
 * 
 * 
 * 
 */

public class AdjCosineSimilarityMethod implements SimilarityMethod {	
	@Override
	public double[][] calSimilarity(double [][]uiRating) {
		
		//@ TO-DO, your code here.
		int useritem = uiRating.length;
		int item = uiRating[0].length; 
		double[][]similarityResult = new double[item][item];
		double[][] arg =  new double[item][item];
		int[][] count = new int[item][item];

		
		
		for (int i = 0; i < item; i++) {
			
			for (int j = i+1 ; j < item; j++) {

				
				for (int k = 0; k < useritem; k++) {
					
					if((uiRating[k][i] > 0)&&(uiRating[k][j] > 0)) {
					count[i][j] +=1; 
					arg[i][j]+=(uiRating[k][i]+uiRating[k][j])/2;
				}
			
				
				}
			}
		}
		
		for (int i = 0; i < count.length; i++) {
			for (int j = i+1; j < count.length; j++) {
				if(count[i][j] == 0) {
					arg[i][j] = 0;
				}
				else
				arg[i][j] = arg[i][j]/count[i][j];
				//System.out.println(arg[i]);
			}
			
		}
		
		for (int i = 0; i < item; i++) {
			
			for (int j = i+1; j < item; j++) {
				double den1 = 0.0,den2 = 0.0,num=0.0;
				
				for (int k = 0; k < useritem; k++) {
					double diff1 = 0.0;
					double diff2 = 0.0;
					
					if((uiRating[k][i] != 0)&&uiRating[k][j]!= 0) {
						
						diff1 = uiRating[k][i]-arg[i][j];
						diff2 = uiRating[k][j]-arg[i][j];
						}
					
					if(diff1<0)		diff1 = -diff1;
					if(diff2<0)		diff2 = -diff2;
					den1+= diff1*diff1;
					den2 += diff2*diff2;
					num += diff1*diff2;
				}
				double den = Math.sqrt(den1) * Math.sqrt(den2);
				
				if(den == 0) {
					similarityResult[i][j] = 0.0;
					similarityResult[j][i] = 0.0;
				}
				else {
				similarityResult[i][j] = num/den;
				//System.out.println(den);
				//System.out.println(num);
				similarityResult[j][i] = num/den;
				}
			}
		}
		
		
		return similarityResult;
	}
	
}