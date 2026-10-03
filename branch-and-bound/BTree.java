package branchandboundexam;

import java.util.Arrays;
import java.util.Comparator;
import java.util.PriorityQueue;

import ilog.concert.*;
import ilog.cplex.*;

//分支定界树
public class BTree {
	
	IPProblem problem;
	double bestObjValue;
	double[] bestSolution;
	boolean foundIntegerSolution;
	
	
	/*
	 * 搜索分支定界树：
	 * (1)节点搜索策略: 最优界搜索 ——LP值小的优先 
	 * (2)剪枝策略
	 */
	
	//将节点和对应求解的目标值 封装
	static class NodeInfo
	{
		Node node;
		double lpObjValue;
		double[] solution;//节点的解
		public NodeInfo(Node node,double lpObjValue)
		{
			this.node=node;
			this.lpObjValue=lpObjValue;
		}
		public NodeInfo(Node node,double lpObjValue,double[] solution)
		{
			this.node=node;
			this.lpObjValue=lpObjValue;
			this.solution=Arrays.copyOf(solution, IPProblem.nCol);
		}
	}
	PriorityQueue<NodeInfo> nodeQueue;//优先队列 目标值排序
	
	//相关属性初始化
	public BTree(IPProblem problem)
	{
		this.problem=problem;
		this.foundIntegerSolution=false;//默认开始非整数解
		
		//min 初始目标 设置正无穷  小值排在前面
		bestObjValue=Double.POSITIVE_INFINITY;
		nodeQueue=new PriorityQueue<>(Comparator.comparingDouble(node->node.lpObjValue));
	}
	
	//主算法
	public double[] solve()
	{
		Node root=new Node(problem);//初始根节点
		IloCplex cplexRoot=null;
		try
		{
			//由根节点建立松弛问题
			IPProblem.ModelAndVars rootNodelAndVars=root.buildLPR();
			cplexRoot=rootNodelAndVars.cplex;
			if(cplexRoot.solve())
			{
				double[] rootSolution=cplexRoot.getValues(rootNodelAndVars.vars);
				nodeQueue.add(new NodeInfo(root,cplexRoot.getObjValue(),rootSolution));
			}else
			{
				//表明根节点松弛问题不可行
				System.out.println(" 根节点松弛问题不可行 ");
				return null;
			}
			cplexRoot.end();
		
		}catch(IloException e)
		{
		  System.err.println("Error "+e.getMessage());
  		  return null;
		}finally
		{
			if(cplexRoot!=null)
			{
				cplexRoot.end();
			}
		}
		
		while(!nodeQueue.isEmpty())
		{
			//从节点队列取出
			NodeInfo currNodeInfo=nodeQueue.poll();
			Node currNode=currNodeInfo.node;
			double currLPObjValue=currNodeInfo.lpObjValue;
			
			//剪枝判断:如果当前节点松弛目标值不如已知最佳整数解,则剪枝
			
			//先判断找到整数解
			if(foundIntegerSolution)
			{
				//对于最小化 线性松弛要好于整数解， 当松弛解大于整数解剪枝
				if(currLPObjValue>=bestObjValue)
				{
					continue;
				}
			}
			
			//对当前节点判断: 
			double[] currSolution=currNodeInfo.solution;
					
			//检查解的类型
			boolean isIntegerSolution=true;//默认要求整数解
			
			int branchVarIndex=-1;//要分支的变量索引
			double branchVal=0.0;
			
			//判断 并 遍历找到第一个分数解 即作为要分支的变量
			for(int i=0;i<IPProblem.nCol;i++)
			{
				//更严谨 需要先判断该变量是否要求为整数解 省略，默认都要求整数解
				
				double val=currSolution[i];
				if(Math.abs(val-Math.round(val))>1e-6)
				{
					//找到分数解
					isIntegerSolution=false;//标记
					branchVarIndex=i;
					branchVal=val;
					break;
				}
			}
					
					
			if(isIntegerSolution)
			{
				//如果当前是整数解,更新全局最优解
				System.out.println("找到一个整数解在节点深度:"+currNode.depth+": "+Arrays.toString(currSolution)+" obj:"+currLPObjValue);
				
				if(currLPObjValue<bestObjValue)
				{
					bestObjValue=currLPObjValue;
					bestSolution=Arrays.copyOf(currSolution,IPProblem.nCol);
					foundIntegerSolution=true;
				}
		
			}else
			{
				//非整数解,分支
				Node[] children=currNode.branch(branchVarIndex, branchVal);
				for(Node child:children)
				{
					//求解子节点添加队列
					IloCplex cplexChild=null;
					try {
						
						IPProblem.ModelAndVars childModelAndVars=child.buildLPR();
						cplexChild=childModelAndVars.cplex;
						cplexChild.setParam(IloCplex.Param.MIP.Limits.Nodes, 0);
						if(cplexChild.solve())
						{
							double childLPObjValue=cplexChild.getObjValue();
							
							//依据新目标值重新剪枝判断
							boolean canPruneChild=false;
							
							if(foundIntegerSolution)
							{
								if(childLPObjValue>=bestObjValue)
								{
									canPruneChild=true;//能剪去
								}
							}
							
							//若不能剪去
							if(!canPruneChild)
							{
								double[] values=cplexChild.getValues(childModelAndVars.vars);
								nodeQueue.add(new NodeInfo(child,childLPObjValue,values));
							}
							
							
						}else
						{
							System.out.println(" 子节点不可行 ");
						}
						
					}catch(IloException e)
					{
						
						System.err.println("Concert exception caught: " + e.getMessage());
					}finally
					{
						if(cplexChild!=null)
						{
							cplexChild.end();
						}
					}
					
				}
				
			}

			
		}//end while
			
		if(foundIntegerSolution)
		{
			return bestSolution;
		}else
		{
			return null;
		}
		
	}//end solve()
}
