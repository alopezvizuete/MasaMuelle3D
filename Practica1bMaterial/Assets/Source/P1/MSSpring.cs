using UnityEngine;
using System.Collections;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;


/// <summary>
/// Elastic energy component corresponding to a simple spring.
/// </summary>
public class MSSpring 
{
	/// <summary>
	/// Default constructor. All zero.
	/// </summary>
	public MSSpring()
	{
		this.Edge = null;
		this.Ke = -1.0f;
		this.L0 = -1.0f;
	}

    public Edge Edge { get; set; }

    public float Ke { get; set; }

    public float L0 { get; set; }

    /// <summary>
    /// Computes the data dependent on the rest state 
    /// of the spring. In this case, only rest-length.
    /// </summary>
    public void computeRestState()
    {
        Node node0 = Edge.Node0;
        Node node1 = Edge.Node1;
        L0 = (node1.X0 - node0.X0).magnitude;
    }

    /// <summary>
    /// Returns the energy of the spring at current state.
    /// </summary>
    public float getEnergy()
    {
        Node node0 = Edge.Node0;
        Node node1 = Edge.Node1;
        float L = (node1.Xt - node0.Xt).magnitude;
        return 0.5f * this.Ke * (L - L0)*(L - L0);
    }

    /// <summary>
    /// Adds the elastic forces corresponding to the string
    /// to the specified global forces vector at the index
	/// associated with edge nodes. Note |vfsum| = nDOF.
    /// </summary>
    public void addForce(VectorXD vfsum)
    {
		// TO COMPLETE: Exercise 2
    }

	/// <summary>
	/// Adds the elastic Jacobian corresponding to the string
	/// to the specified global Jacobian matrix at the index
	/// associated with edge nodes. Note |mJSum| = (nDOF,nDOF).
	/// </summary>
	public void addJacobian(MatrixXD mJsum)
	{
		// TO COMPLETE: Exercise 2
	}

}
