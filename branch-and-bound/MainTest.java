package branchandboundexam;

import java.util.Arrays;

public class MainTest {

	public static void main(String[] args) {
		
		
		IPProblem problem=new IPProblem();
		BTree btree=new BTree(problem);
		double[] solution=btree.solve();
		
		if(solution!=null)
		{
			System.out.println("最终结果: ");
			 System.out.println(Arrays.toString(solution));
		}
		
		
		
	}

}
