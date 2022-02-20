import java.util.regex.Matcher;
import java.util.regex.Pattern;
public class RegexTest{
	public static void main(String[] args){
		
		String Nextline = "		 println	 a hhhsdd dwdw ";
		String regex = "(\\S+)\\b";
		Matcher matcher = Pattern.compile(regex).matcher(Nextline);
		if(matcher.find()){
			System.out.println(matcher.group(1));
		}
		else System.out.println("not found");
		
	}
}