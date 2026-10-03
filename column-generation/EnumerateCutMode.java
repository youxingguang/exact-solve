package BranchAndPrice;
import java.util.List;

import ilog.concert.*;
import ilog.cplex.*;

import java.util.ArrayList;
import java.util.Arrays;
public class EnumerateCutMode {

	//枚举所有可能的切割方案——再用完整模型求解
	/**
	 * 
	 * @param demand_amount 每种木材需求数量
	 * @param demand_size 每种成品尺寸
	 * @param W 原料长度
	 * @return
	 */
	public List<int[]> enumerateMode(double[] demand_amount,double[] demand_size,double W)
	{
		List<int[]> ans=new ArrayList<>();
		int nRow=demand_amount.length;
		int[] path=new int[nRow];
		dfs(demand_amount,demand_size,W,0,path,ans);
		return ans;
	}
	//按组合的做法
	public void dfs(double[] demand_amount,double[] demand_size,double W,int i,int[] path,List<int[]> ans)
	{
		//枚举最后结束
		if (i == demand_size.length) 
		{
            ans.add(Arrays.copyOf(path, path.length));
            return;
        }

        int maxCount = (int) (W / demand_size[i]); // 当前尺寸最多可切多少个
        for (int k = 0; k <= maxCount; k++) 
        {
            path[i] = k;
            double used = k * demand_size[i];
            if (used > W + 1e-6) continue; // 超过长度，跳过
            dfs(demand_amount, demand_size, W - used, i + 1, path, ans);
        }
	}
	//在已知所有切割方案,然后计算最少需要木料数
	public static void main(String[] args) {
		try {
		 double[] demand_amount= {48,35,24,10,8};//每种木材需求数量
		 double[] demand_size= {2.0,4.5,5.0,5.5,7.5};//每种成品尺寸
		 double W=11;//原料长度
		 EnumerateCutMode ECM=new EnumerateCutMode();
		 List<int[]> ans=ECM.enumerateMode(demand_amount,demand_size,W);
		 
		 //验证
		 IloCplex model=new IloCplex();
		 int n=demand_size.length;
		 IloObjective Obj=model.addMinimize();
		 IloRange[] rng=new IloRange[n];
		 for(int i=0;i<n;i++)
		 {
			 rng[i]=model.addRange(demand_amount[i], Double.MAX_VALUE);
		 }
		 //j——列
		 IloNumVar[] X=new IloNumVar[ans.size()];
		 for(int j=0;j<ans.size();j++)
		 {
			 IloColumn column=model.column(Obj,1.0);
			 int[] arr=ans.get(j);
			 for(int i=0;i<arr.length;i++)
			 {
				column=column.and(model.column(rng[i],arr[i])); 
			 }
			 X[j]=model.numVar(column,0.0,Double.MAX_VALUE,IloNumVarType.Int);
		 }
		 if(model.solve())
		 {
			 System.out.println("最少花费："+model.getObjValue());
			 for(int j=0;j<ans.size();j++)
			 {
				 System.out.println("可能的切割方案："+Arrays.toString(ans.get(j)));
				 System.out.println("0-未选中,否则选中："+model.getValue(X[j]));
			 }
		 }
		}catch(IloException exc) 
		{
			   System.out.println("Concert exception caught:"+exc);
		 }

	}

}
