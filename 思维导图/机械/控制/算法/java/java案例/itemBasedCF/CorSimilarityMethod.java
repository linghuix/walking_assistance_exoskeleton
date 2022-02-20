package itemBasedCF;

/**
 * Describes the Correlation-based Similarity measure to be used by a ModelBuilder to build an item-based model
 * @author Wang Menghan
 * @version 1.0 
 * @Time 2015-11-4 15:48:39
 */

public class CorSimilarityMethod implements SimilarityMethod {	
	@Override
	public double[][] calSimilarity(double [][]uiRating) {
		
		// calculate the avg ratings of items.
		int userNum = uiRating.length;
		int itemNum = uiRating[0].length;
		double[][] similarityResult = new double[itemNum][itemNum]; // store the item-item similarity
		
		// calculate the average score for each item, if we meet 0 (means this item is unrated by the user), we ignore it.
		double [] []  avg = new double[itemNum][itemNum];
		int [][] nonZeroNum = new int[itemNum][itemNum];
		for (int i = 0; i < itemNum; i++) {
			
			for (int j = i+1 ; j < itemNum; j++) {

				for (int k = 0; k < userNum; k++) {
					
					if((uiRating[k][i] > 0)&&(uiRating[k][j] > 0)) {
					nonZeroNum[i][j] +=1; 
					nonZeroNum[j][i] +=1; 
					avg[i][j]+=uiRating[k][i];
					avg[j][i]+=uiRating[k][j];
				   }
				}
			}
		}
		
		for (int i = 0; i <nonZeroNum.length; i++) {
			for (int j = i+1; (j < nonZeroNum.length); j++) {
				if(nonZeroNum[i][j] == 0) {
					avg[i][j] = 0;
					avg[j][i] = 0;
				}
				else
				avg[i][j] = avg[i][j]/nonZeroNum[i][j];
				avg[j][i] = avg[j][i]/nonZeroNum[j][i];
//				System.out.println(avg[i][j]);
			}
		}
//		for(int i = 0;i<itemNum;i++){
//			for(int j = 0;j<userNum;j++){
//				avg[i]+=uiRating[j][i];
//				
//				if(uiRating[j][i]>0){
//					nonZeroNum[i]+=1;
//				}
//					
//			}
//		}
		
//		for(int i = 0;i<itemNum;i++){
//			avg[i]/=nonZeroNum[i];	
//		}
		
		//for each item-item pair, we calculate the Correlation-based Similarity using the method stated in Badrul Sarwar's paper - 
		//"Item-Based Collaborative Filtering Recommendation Algorithms"
		for(int i = 0;i<itemNum;i++){
			
			for(int j = i+1;j<itemNum;j++){
				double num = 0.0, den1 = 0.0, den2 = 0.0;
				
				for(int k=0;k<userNum;k++){					
					double diff1 = 0.0;
					double diff2 = 0.0;
//					if(uiRating[k][i]>0)
//						diff1 = uiRating[k][i] - avg[i];
//					if(uiRating[k][j]>0)
//						diff2 = uiRating[k][j] - avg[j];
					if(uiRating[k][i]>0&&uiRating[k][j]>0) {
						
					diff1 = uiRating[k][i] - avg[i][j];
					diff2 = uiRating[k][j] - avg[j][i];
					}
					if(diff1<0)		diff1 = -diff1;
					if(diff2<0)		diff2 = -diff2;
		            num += diff1 * diff2;//协方差
		            den1 += diff1 * diff1;//离差平方和 
		            den2 += diff2 * diff2;//离差平方和
					
				}
				double den = Math.sqrt(den1) * Math.sqrt(den2);
				
		        if (den == 0.0){					// zero case	
		        	similarityResult[i][j]=0.0;
		        	similarityResult[j][i]=0.0;
		        }
		        else {
		        similarityResult[i][j] = num / den;	// we store the similarities in a symmetric matrix for later convenience
		        similarityResult[j][i] = num / den; //即求得二个数据之间的相关系数
		        }
			}
		}
		
		return similarityResult;
    }

	

}
