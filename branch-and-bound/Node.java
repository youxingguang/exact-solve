package branchandboundexam;

import java.util.Arrays;

import branchandboundexam.IPProblem.*;

import ilog.concert.*;


/*
 * (1)完整的问题 IPProblem problem
 * (2)分支节点：固定部分变量
 * (3)重新建立松弛问题
 * 
 * 
 */

public class Node {
	
	IPProblem problem;
	double[] currLB;//当前节点的变量的下界
	double[] currUB;//当前节点的变量的上界
	int depth;//节点深度
	
	//构造根节点
	public Node(IPProblem problem)
	{
		this.problem=problem;
		this.currLB=Arrays.copyOf(IPProblem.initial_lb,IPProblem.nCol);
		this.currUB=Arrays.copyOf(IPProblem.initial_ub,IPProblem.nCol);
		this.depth=0;
	}
	
	public Node(IPProblem problem,double[] currLB,double[] currUB,int depth)
	{
		this.problem=problem;
		this.currLB=Arrays.copyOf(currLB,IPProblem.nCol);
		this.currUB=Arrays.copyOf(currUB,IPProblem.nCol);
		this.depth=depth;
	}
	
	
	
	//依据节点 新建松弛模型
	public ModelAndVars buildLPR() throws IloException
	{
		ModelAndVars newModelAndVars=problem.initialModel();
		
		//更改变量边界
		problem.setVarBound(newModelAndVars,currLB,currUB);
		return newModelAndVars;
	}
	
	//对节点分支
	/**
	 * 
	 * @param varIndex 要分支的变量索引
	 * @param value  分支变量的值
	 * @return
	 */
	public Node[] branch(int varIndex,double value)
	{
		Node[] children=new Node[2];
		
		//节点1 向下取整 如 0.4--0
		double[] lb1=Arrays.copyOf(currLB, IPProblem.nCol);
		double[] ub1=Arrays.copyOf(currUB, IPProblem.nCol);
		ub1[varIndex]=Math.floor(value);
		children[0]=new Node(problem,lb1,ub1,depth+1);
		
		//节点2 向上取整 如 0.4--1
		double[] lb2=Arrays.copyOf(currLB, IPProblem.nCol);
		double[] ub2=Arrays.copyOf(currUB, IPProblem.nCol);
		lb2[varIndex]=Math.ceil(value);
		children[1]=new Node(problem,lb2,ub2,depth+1);
		
		return children;
	}
}
