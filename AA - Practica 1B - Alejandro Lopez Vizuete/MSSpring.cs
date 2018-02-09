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

        //Fuerza elastica de Nodo 0 a Nodo 1
        Vector3 Felastica1 = -Ke * ((Edge.Node0.Xt - Edge.Node1.Xt).magnitude - L0) * ((Edge.Node0.Xt - Edge.Node1.Xt) / (Edge.Node0.Xt - Edge.Node1.Xt).magnitude);
        //float Felastica1X = -Ke * ((Edge.Node0.Xt.x - Edge.Node1.Xt.x) - L0);
        //float Felastica1Y = -Ke * ((Edge.Node0.Xt.y - Edge.Node1.Xt.y) - L0);
        //float Felastica1Z = -Ke * ((Edge.Node0.Xt.z - Edge.Node1.Xt.z) - L0);

        //Fuerza Elastica de Nodo 1 a Nodo 0
        Vector3 Felastica0 = -Ke * ((Edge.Node1.Xt - Edge.Node0.Xt).magnitude - L0) * ((Edge.Node1.Xt - Edge.Node0.Xt) / (Edge.Node1.Xt - Edge.Node0.Xt).magnitude);
        //float Felastica0X = -Ke * ((Edge.Node1.Xt.x - Edge.Node0.Xt.x) - L0);
        //float Felastica0Y = -Ke * ((Edge.Node1.Xt.y - Edge.Node0.Xt.y) - L0);
        //float Felastica0Z = -Ke * ((Edge.Node1.Xt.z - Edge.Node0.Xt.z) - L0);

        //Sumamos las fuerzas en el nodo correspondiente
        vfsum[3*Edge.Node1.SimIdx + 0] += Felastica0.x;
        vfsum[3*Edge.Node1.SimIdx + 1] += Felastica0.y;
        vfsum[3*Edge.Node1.SimIdx + 2] += Felastica0.z;
        vfsum[3*Edge.Node0.SimIdx + 0] += Felastica1.x;
        vfsum[3*Edge.Node0.SimIdx + 1] += Felastica1.y;
        vfsum[3*Edge.Node0.SimIdx + 2] += Felastica1.z;
    }

	/// <summary>
	/// Adds the elastic Jacobian corresponding to the string
	/// to the specified global Jacobian matrix at the index
	/// associated with edge nodes. Note |mJSum| = (nDOF,nDOF).
	/// </summary>
	public void addJacobian(MatrixXD mJsum)
	{
        // TO COMPLETE: Exercise 2
        //Calculos necesarios
        float L = (Edge.Node1.Xt - Edge.Node0.Xt).magnitude;
        Vector3 Felastica0 = -Ke * ((Edge.Node1.Xt - Edge.Node0.Xt).magnitude - L0) * ((Edge.Node1.Xt - Edge.Node0.Xt) / (Edge.Node1.Xt - Edge.Node0.Xt).magnitude);
        Vector3 Felastica1 = -Ke * ((Edge.Node0.Xt - Edge.Node1.Xt).magnitude - L0) * ((Edge.Node0.Xt - Edge.Node1.Xt) / (Edge.Node0.Xt - Edge.Node1.Xt).magnitude);

        //Calculamos la Jacobiana de la Felastica para cada dirección de cada nodo
        Vector3 J1X = (1 / L) * (-Ke * (L - L0) - Ke * (Edge.Node0.Xt.x - Edge.Node1.Xt.x) + Felastica0.x) * (Ke * (L - L0) * ((Edge.Node1.Xt - Edge.Node0.Xt) / L));
        Vector3 J1Y = (1 / L) * (-Ke * (L - L0) - Ke * (Edge.Node0.Xt.y - Edge.Node1.Xt.y) + Felastica0.y) * (Ke * (L - L0) * ((Edge.Node1.Xt - Edge.Node0.Xt) / L));
        Vector3 J1Z = (1 / L) * (-Ke * (L - L0) - Ke * (Edge.Node0.Xt.z - Edge.Node1.Xt.z) + Felastica0.z) * (Ke * (L - L0) * ((Edge.Node1.Xt - Edge.Node0.Xt) / L));
        Vector3 J0X = (1 / L) * (-Ke * (L - L0) - Ke * (Edge.Node1.Xt.x - Edge.Node0.Xt.x) + Felastica1.x) * (Ke * (L - L0) * ((Edge.Node0.Xt - Edge.Node1.Xt) / L));
        Vector3 J0Y = (1 / L) * (-Ke * (L - L0) - Ke * (Edge.Node1.Xt.y - Edge.Node0.Xt.y) + Felastica1.y) * (Ke * (L - L0) * ((Edge.Node0.Xt - Edge.Node1.Xt) / L));
        Vector3 J0Z = (1 / L) * (-Ke * (L - L0) - Ke * (Edge.Node1.Xt.z - Edge.Node0.Xt.z) + Felastica1.z) * (Ke * (L - L0) * ((Edge.Node0.Xt - Edge.Node1.Xt) / L));

        //Asignamos los valores de la Jacobiana en su lugar corredpondiente en la matriz
        mJsum[3 * Edge.Node1.SimIdx + 0, 3 * Edge.Node1.SimIdx + 0] += J1X.x;
        mJsum[3 * Edge.Node1.SimIdx + 1, 3 * Edge.Node1.SimIdx + 1] += J1Y.y;
        mJsum[3 * Edge.Node1.SimIdx + 2, 3 * Edge.Node1.SimIdx + 2] += J1Z.z;
        mJsum[3 * Edge.Node0.SimIdx + 0, 3 * Edge.Node0.SimIdx + 0] += J0X.x;
        mJsum[3 * Edge.Node0.SimIdx + 1, 3 * Edge.Node0.SimIdx + 1] += J0Y.y;
        mJsum[3 * Edge.Node0.SimIdx + 2, 3 * Edge.Node0.SimIdx + 2] += J0Z.z;
    }

}
