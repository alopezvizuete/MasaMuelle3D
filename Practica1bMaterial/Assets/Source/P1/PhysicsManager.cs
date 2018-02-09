using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		this.m_simulable = null;
		this.m_isSimulable = false;

		this.Paused = true;
		this.OneStep = false;
		this.TimeStep = 0.01f;
		this.Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		this.IntegrationMethod = Integration.Explicit;
		this.SimulableObject = null;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Explicit = 0,
		Symplectic = 1,
		SemiImplicit = 2,
        MidPoint = 3,
        Verlet = 4
	};

	#region InEditorVariables
	
	public bool Paused;
	public bool OneStep;
	public float TimeStep;
	public Vector3 Gravity;
	public GameObject SimulableObject;
	public Integration IntegrationMethod;

	#endregion

	#region OtherVariables

	private ISimulable m_simulable;

	private bool m_isSimulable;
	
	private VectorXD m_vxPre;
	private VectorXD m_vvPre;
		
	#endregion

	#region MonoBehaviour

	public void Start () 
	{
		// Default initialization
		this.m_simulable = null;

		// Try to get simulable component from the object
		this.m_simulable = this.SimulableObject.GetComponent<ISimulable> ();

		// Check if component was found
		if (this.m_simulable == null) 
		{
			System.Console.WriteLine ("[ERROR] Couldn't find any ISimulable component");
			this.m_isSimulable = false;
		} 
		else 
		{
			System.Console.WriteLine ("[TRACE] Succesfully found ISimulable component");
			this.m_isSimulable = true;
		}

		if (this.m_isSimulable) 
		{
			// Initialize simulable model
			this.m_simulable.initialize ();

			// Update forces/matrices after the change
			this.m_simulable.updateForcesAndMatrices ();

			// Store current position and velocity vectors
			this.m_vxPre = this.m_simulable.getPositionVector();
			this.m_vvPre = this.m_simulable.getVelocityVector();
		}
	}

	public void Update()
	{
		if (Input.GetKeyUp (KeyCode.O))
			this.OneStep = !this.OneStep;
	
		if (Input.GetKeyUp (KeyCode.P))
			this.Paused = !this.Paused;
	}

	public void FixedUpdate () 
	{
		if (this.Paused && !this.OneStep)
			return; // Not simulating

		if (this.OneStep) // One!
			this.OneStep = false;

		// Simulate if possible
		if (this.m_isSimulable) 
		{
			// Update forces/matrices after the change
			this.m_simulable.updateForcesAndMatrices ();

			// Select integration method
			switch (this.IntegrationMethod)
			{
			case Integration.Explicit: this.stepExplicit(this.m_simulable); break;
			case Integration.Symplectic: this.stepSymplectic(this.m_simulable); break;
			case Integration.SemiImplicit: this.stepSemiImplicit(this.m_simulable); break;
			case Integration.MidPoint: this.stepMidpoint(this.m_simulable); break;
			case Integration.Verlet: this.stepVerlet(this.m_simulable); break;
			default: 
				throw new System.Exception("[ERROR] WTF? Should never happen!");
			}
			// Update simulated elements
			this.m_simulable.updateScene();
		}
	}

	#endregion

	/// <summary>
	/// Performs a simulation step using Explicit integration.
	/// </summary>
	private void stepExplicit(ISimulable o)
	{
		// TO COMPLETE: Exercise 2
    }

	/// <summary>
	/// Performs a simulation step using Symplectic integration.
	/// </summary>
	private void stepSymplectic(ISimulable o)
	{
		// TO COMPLETE: Exercise 2
    }

	/// <summary>
	/// Performs a simulation step using MidPoint integration.
	/// </summary>
	private void stepMidpoint(ISimulable o)
	{
		// TO COMPLETE: Exercise 2
	}

	/// <summary>
	/// Performs a simulation step using Verlet integration.
	/// </summary>

	private void stepVerlet(ISimulable o)
	{
		// TO COMPLETE: Exercise 2
	}

	/// <summary>
	/// Performs a simulation step using Semi-Implicit integration.
	/// </summary>
	private void stepSemiImplicit(ISimulable o)
	{
		// TO COMPLETE: Exercise 2
	}

}
