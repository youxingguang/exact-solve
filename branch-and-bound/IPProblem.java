package branchandboundexam;

import ilog.cplex.*;
import ilog.concert.*;

//定义整数问题-参数和变量
public class IPProblem {
   
	//建立松弛模型
	/*
	 * min 500x1+200x2+200x3+300x4
	 *     6000x1+2000x2+1500x3+1800x4>=3000
	 *     x\in {0,1}
	 */
	
	
	static double[] c= {500,200,200,300};//目标系数
	static double[] A= {6000,2000,1500,1800};//约束系数
	static double b=3000;
	static int nCol=4;
	boolean[] isIntegerVar;//与x同长度 用来说明哪个是整数变量
	static double[] initial_lb= {0,0,0,0};//
	static double[] initial_ub= {1,1,1,1};//
	//初始模型
	public ModelAndVars initialModel() throws IloException
	{
		IloCplex model=new IloCplex();
		IloNumVar[] x=new IloNumVar[nCol];
		
	    //为变量指定边界
		for(int i=0;i<nCol;i++)
		{
			x[i]=model.numVar(initial_lb[i],initial_ub[i]);
		}
		
		//目标
		model.addMinimize(model.scalProd(c, x));
		
		//约束
		model.addGe(model.scalProd(A, x), b);
		
		return new ModelAndVars(model,x);
	}
	//将模型与变量封装
	static class ModelAndVars
	{
		public IloCplex cplex;
		public IloNumVar[] vars;
		
		public ModelAndVars(IloCplex cplex,IloNumVar[] vars)
		{
			this.cplex=cplex;
			this.vars=vars;
		}
	}
	//修改变量的边界
	public void setVarBound(ModelAndVars modelandvars,double[] lb,double[] ub) throws IloException
	{
		for(int i=0;i<nCol;i++)
		{
			 modelandvars.vars[i].setLB(lb[i]);
			 modelandvars.vars[i].setUB(ub[i]);
		}
	}
	
}
