using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic mass-spring model component which can be dropped onto
/// a game object and configured so that the set of nodes and
/// edges behave as a mass-spring model.
/// </summary>
public class SimulableNodesMS : MonoBehaviour, ISimulable
{
	/// <summary>
	/// Default constructor. All zero. 
	/// </summary>
	public SimulableNodesMS()
	{
		this.Manager = null;
		this.SceneNodes = new List<SceneNode> ();
		this.SceneEdges = new List<SceneEdge> ();
		this.Ke = 1.0f;
		this.Kd = 0.0f;
	}

	#region EditorVariables

	public PhysicsManager Manager;
	public List<SceneNode> SceneNodes;
	public List<SceneEdge> SceneEdges;

	/// <summary>
	/// Elastic stiffness.
	/// </summary>
	public float Ke;

	/// <summary>
	/// Damping stiffness.
	/// </summary>
	public float Kd;

	#endregion

    #region OtherVariables
	
	protected Node[] m_vnodes;
	protected Edge[] m_vedges;

    protected VectorXD m_vf;
    protected MatrixXD m_mJ;
	protected MatrixXD m_mD;
	protected MatrixXD m_mM;

    protected List<MSSpring> m_vsprings;

    #endregion

	#region MonoBehaviour

	// Nothing to do here

	#endregion

    #region ISimulable

	public void initialize ()
	{
		int numNodes = this.SceneNodes.Count;
		int numEdges = this.SceneEdges.Count;
		this.m_vnodes = new Node[numNodes];
		this.m_vedges = new Edge[numEdges];
		
		// Start scene nodes/edges

		for (int i = 0; i < this.SceneEdges.Count; ++i)
			this.SceneEdges [i].initialize (); // Prepare
		
		for (int i = 0; i < this.SceneNodes.Count; ++i)
			this.SceneNodes [i].initialize (); // Prepare
		
		// Cache internal nodes
		for (int i = 0; i < numNodes; ++i) 
		{
			this.m_vnodes [i] = this.SceneNodes [i].Node;
			
			// Configure nodes indices
			this.m_vnodes [i].SimIdx = i;
		}
		
		// Cache internal edges
		for (int i = 0; i < numEdges; ++i)
			this.m_vedges [i] = this.SceneEdges [i].Edge;
		
		int N = this.getNumDOF ();
		this.m_vf = new DenseVectorXD (N);
		this.m_mJ = new DenseMatrixXD (N, N);
		this.m_mD = new DenseMatrixXD (N, N);
		this.m_mM = new DenseMatrixXD (N, N);
		
		// Create energy elements vector and initialize rest-state
		this.m_vsprings = new List<MSSpring> (this.m_vedges.Length);
		
		for (int i = 0; i < this.m_vedges.Length; ++i) 
		{
			MSSpring spring = new MSSpring();
			spring.Edge = this.m_vedges[i];
			spring.Ke = this.Ke;
			spring.L0 = 0.0f;
			
			// Restart rest and add
			spring.computeRestState();
			this.m_vsprings.Add(spring);
		}
		
		// Initialize forces and matrices
		this.clearForcesAndMatrices ();
	}

	public int getNumDOF ()
	{
		return 3 * this.m_vnodes.Length;
	}

	public int getNumNodes ()
	{
		return this.m_vnodes.Length;
	}
	
	public Node getNode (int i)
	{
		return this.m_vnodes [i];
	}
	
	public VectorXD getForceVector ()
	{
		return this.m_vf.Clone();
	}

	public VectorXD getMassVector ()
	{
		VectorXD vmout = new DenseVectorXD (this.getNumDOF ());
		for (int i = 0; i < this.getNumNodes(); ++i)
			for (int j = 0; j < 3; ++j) // Set X,Y,Z values
				vmout [3 * i + j] = this.m_vnodes[i].Mass;

		return vmout;
	}

	public VectorXD getVelocityVector ()
	{
		VectorXD vvout = new DenseVectorXD (this.getNumDOF ());

		for (int i = 0; i < this.getNumNodes(); ++i) {
			vvout [3 * i + 0] = this.m_vnodes [i].Vt [0];
			vvout [3 * i + 1] = this.m_vnodes [i].Vt [1];
			vvout [3 * i + 2] = this.m_vnodes [i].Vt [2];
		}
            
		return vvout;
	}

	public VectorXD getPositionVector ()
	{
		VectorXD vxout = new DenseVectorXD (this.getNumDOF ());

		for (int i = 0; i < this.getNumNodes(); ++i) {
			vxout [3 * i + 0] = this.m_vnodes [i].Xt [0];
			vxout [3 * i + 1] = this.m_vnodes [i].Xt [1];
			vxout [3 * i + 2] = this.m_vnodes [i].Xt [2];
		}

		return vxout;
	}

	public void setVelocityVector (VectorXD vvin)
	{
		for (int i = 0; i < this.getNumNodes(); ++i) {
			this.m_vnodes [i].Vt = new Vector3(
				(float)vvin [3 * i + 0],
				(float)vvin [3 * i + 1],
				(float)vvin [3 * i + 2]);
		}
	}
	
	public void setPositionVector (VectorXD vxin)
	{
		for (int i = 0; i < this.getNumNodes(); ++i) {
			this.m_vnodes [i].Xt = new Vector3(
				(float)vxin [3 * i + 0],
				(float)vxin [3 * i + 1],
				(float)vxin [3 * i + 2]);
		}
	}

	public MatrixXD getMassMatrix ()
	{
		return this.m_mM;
	}

	public MatrixXD getDampingMatrix ()
	{
		return this.m_mD;
	}

	public MatrixXD getSpringsJacobian ()
	{
		return this.m_mJ;
	}

	public void updateForcesAndMatrices ()
	{
		this.clearForcesAndMatrices ();

		// TO COMPLETE: Exercise 2
	}

	public void clearForcesAndMatrices ()
	{
		this.m_vf.Clear ();
		this.m_mJ.Clear ();
		this.m_mD.Clear ();
		this.m_mM.Clear ();
	}

	public void updateScene ()
	{
		// Update nodes
		for (int i = 0; i < this.SceneNodes.Count; ++i)
			this.SceneNodes [i].updateScene (); // Async.

		// Update edges
		for (int i = 0; i < this.SceneEdges.Count; ++i)
			this.SceneEdges [i].updateScene (); // Async.
	}

	public void fixSimulationVector(VectorXD v)
	{
		for (int i = 0; i < this.getNumNodes(); ++i) 
		{
			if (this.m_vnodes [i].IsFixed) 
			{
				int nodeOffset = 3*this.m_vnodes[i].SimIdx;
				for (int j = 0; j < 3; ++j)
					v[nodeOffset + j] = 0.0;
			}
		}
	}

	public void fixSimulationMatrix(MatrixXD m)
	{
		for (int i = 0; i < this.getNumNodes(); ++i)
		{
			if (this.m_vnodes[i].IsFixed)
			{			
				int nodeOffset = 3*this.m_vnodes[i].SimIdx;
				for (int j = 0; j < 3; ++j)
				{
					for (int k = 0; k < this.getNumDOF(); ++k)
					{
						m[nodeOffset + j,k] = 0.0;
						m[k,nodeOffset + j] = 0.0;
					}
					m[nodeOffset + j,nodeOffset + j] = 1.0;
				}
			}
		}
	}

    #endregion

	#region OtherMethods

	// TO COMPLETE: Exercise 2

	#endregion

}
