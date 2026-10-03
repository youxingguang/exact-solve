package BranchAndPrice;

import ilog.concert.*;
import ilog.cplex.*;

public class ColumnGeneration {
	/*
	 * RMP
	 * min \sum_i{1-5} x_i
	 * 5x1>=48  //[11/2]=5  切成2m长的木品5个
	 * 2x2>=35
	 * 2x3>=24
	 * 2x4>=10
	 * x5>=8
	 * x_i>=0
	 */
	static double[] demand_amount= {48,35,24,10,8};//每种木材需求数量
	static double[] demand_size= {2.0,4.5,5.0,5.5,7.5};//每种成品尺寸
	static double W=11;//原料长度
	static int nRow=demand_amount.length;//约束数
	static int nCol=demand_size.length;//变量数  5种单切方案
	
	IloCplex master;
	IloRange[] rng;
	double eps=1.0e-6;//设定
	 IloNumVarArray Cut;
	 IloObjective masterObj;
	//按列建模——RMP
	public void buildModelByColumn() throws IloException
	{
		master=new IloCplex();
		master.setParam(IloCplex.Param.RootAlgorithm,IloCplex.Algorithm.Primal);
		masterObj=master.addMinimize();//主问题目标
		rng=new IloRange[demand_amount.length];
	   for(int i=0;i<demand_amount.length;i++)
	   {
		   //添加约束上下界
		   rng[i]=master.addRange(demand_amount[i], Double.MAX_VALUE);
	   }
	  
	    Cut=new IloNumVarArray();//切割方案 X
	   for(int j=0;j<nCol;j++)
	   {
		   IloColumn col=master.column(masterObj,1.0);
		   col=col.and(master.column(rng[j],(int)(W/demand_size[j])));
	       Cut.add(master.numVar(col,0.0,Double.MAX_VALUE));//变量初始化
	   }
	}
	//定价子问题——生成新的列
	/*
	 * 检验数 c_j-u^t A_j
	 * min 1.0-u^t A_j
	 */
	IloCplex sub;
	IloObjective ReducedCost;
	IloNumVar[] NonB;
	public void buildSp() throws IloException
	{
		sub=new IloCplex();
		NonB=sub.numVarArray(nRow, 0.,Double.MAX_VALUE,IloNumVarType.Int);//A_j
		
		
		//约束 L_i cut_i<=W 一根木料切出的成品总长不超过原木料的程度
		sub.addRange(-Double.MAX_VALUE,sub.scalProd(demand_size,NonB),W);	
	}
	public void solve() throws IloException
	{
		buildModelByColumn();
		
		buildSp();
		for(;;)
		{
			master.solve();
			double[] shadowPrice=master.getDuals(rng);
			
			//更新目标子问题目标函数(或者说更新检验数)
			 //ReducedCost.setExpr(sub.diff(1.0, sub.scalProd(NonB, shadowPrice)));
				if(ReducedCost!=null)
				{
					sub.delete(ReducedCost);
					ReducedCost=null;
				}
				IloNumExpr subObj=sub.diff(1.0, sub.scalProd(NonB, shadowPrice));
			    ReducedCost=sub.addMinimize(subObj);
			    
			    if(sub.solve())
			    {
			    	//对于min 检验数>=0停止
			    	if(sub.getObjValue()>-eps)
			    	{
			    		break;
			    	}
			    	double[] newPatt=sub.getValues(NonB);
			    	//向RMP添加新的一列
				   IloColumn column=master.column(masterObj,1.0);
				   for(int i=0;i<newPatt.length;i++)
				   {
					   column=column.and(master.column(rng[i],newPatt[i]));
				   }
				   Cut.add(master.numVar(column,0.0,Double.MAX_VALUE));
			    }
		}
		//转换变量类型：IloNumVarArray--IloNumVar[]
		   for ( int i = 0; i < Cut.getSize(); i++ ) 
		   {
			   //当补齐主问题的列,可以设定求解变量类型
	           master.add(master.conversion(Cut.getElement(i),
	                                              IloNumVarType.Int));
	        }
		   master.solve();
		  
		   System.out.println("masterObj="+master.getObjValue());
		   for(int i=0;i<Cut.getSize();i++)
		   {
			   System.out.println("Cut"+i+" = "+master.getValue(Cut.getElement(i)));
		   }
		   //master.exportModel("cutPattern.lp");
		   master.end();
		   sub.end();
	}
	 //为防止变量长度不够，设计"自动增长"
	   static class IloNumVarArray{
		   int _num=0;
		   IloNumVar[] _array=new IloNumVar[32];
		   void add(IloNumVar ivar)
		   {
			   if(_num>=_array.length)
			   {
				   IloNumVar[] array=new IloNumVar[2*_array.length];
				   System.arraycopy(_array, 0, array, 0, _num);//将旧数组内容复制到新数组
				   _array=array;
			   }
			   _array[_num++]=ivar;
		   }
		   IloNumVar getElement(int i)
		   {
			   return _array[i];//统计变量值
		   }
		   int getSize()
		   {
			   return _num; //统计变量数
		   }
		   
	   }
	
	public static void main(String[] args) throws IloException {
		ColumnGeneration cg=new ColumnGeneration();
		cg.solve();

	}

}
