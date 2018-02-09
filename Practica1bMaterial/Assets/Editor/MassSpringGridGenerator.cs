using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using UnityEditor;

public class MassSpringGridGenerator : ScriptableWizard
{
    #region GeneratorOptions

    public Color NodeColor = new Color(0, 0.71f, 1, 1); //Orange
    public Color EdgeColor = new Color(1, 0.44f, 0, 1); //Light Blue

    public uint DimensionX = 2;
    public uint DimensionY = 2;
    public uint DimensionZ = 2;
    public float SizeX = 1.0f;
    public float SizeY = 1.0f;
    public float SizeZ = 1.0f;

    public float NodeRadius = 0.2f;
    public float EdgeRadius = 0.1f;

	public float NodeMass = 1.0f;
	public float Kd = 1.0f;
	public float Ke = 1.0f;

	public bool FixedNode = false;

    public bool AddLatticeSprings = true;
    public bool AddDiagonalSprings = false;

    public bool DeletePrevious = false;


    public string Name = "New Mass-Spring System";

    #endregion

    #region OtherMembers

	private SceneNode[] m_vnodesVector;
    private SceneEdge[] m_vedgesVector;

    private SceneNode[, ,] m_mnodesMatrix;
    private SceneNode[, ,] m_minnernodesMatrix;

    [MenuItem("GameObject/Create Mass-Spring Grid %g")]
    static void CreateWizard()
    {
        ScriptableWizard.DisplayWizard<MassSpringGridGenerator>("Create Grid", "Create");
    }

    void OnWizardCreate()
    {
        m_mnodesMatrix = new SceneNode[this.DimensionX, this.DimensionY, this.DimensionZ];

        // Delete all previous mass-spring grids
        if(DeletePrevious)
        {
            GameObject[] massSpringGOList;
            massSpringGOList = GameObject.FindGameObjectsWithTag("Mass-Spring");

            // Backwards iteration to avoid memory leaks
            for (int i = massSpringGOList.Length - 1; i >= 0; --i)
            {
                DestroyImmediate(massSpringGOList[i]);
            }
        }

        bool areDiagSprings_doable = AddDiagonalSprings;
        if (this.AddDiagonalSprings)
        {
            if (this.DimensionX > 1 && this.DimensionY > 1 && this.DimensionZ > 1)
            {
                // Initialize matrix for diagonal springs
                m_minnernodesMatrix = new SceneNode[this.DimensionX - 1, this.DimensionY - 1, this.DimensionZ - 1];
            }
            else
            {
                //Not Diagonal Springs for 2D or 1D!!!
                //AddDiagonalSprings = false;
                areDiagSprings_doable = false;
            }
        }

        GameObject root = new GameObject(Name);
        GameObject nodesGroup = new GameObject("Nodes");
        GameObject edgesGroup = new GameObject("Edges");

        root.tag = "Mass-Spring";

        nodesGroup.transform.SetParent(root.transform);
        edgesGroup.transform.SetParent(root.transform);

        Vector3 start = new Vector3(-this.SizeX / 2.0f, -this.SizeY / 2.0f, -this.SizeZ / 2.0f);

        Vector3 delta = new Vector3(
            (DimensionX == 1)? 0 : this.SizeX / (float)(this.DimensionX - 1),
            (DimensionY == 1)? 0 : this.SizeY / (float)(this.DimensionY - 1),
            (DimensionZ == 1)? 0 : this.SizeZ / (float)(this.DimensionZ - 1));

        // Create nodes

		List<SceneNode> vtempNodes= new List<SceneNode>(); ;

        int count = 0;
        int diagCount = 0;

        for (int i = 0; i < this.DimensionX; ++i)
        {
            for (int j = 0; j < this.DimensionY; ++j)
            {
                for (int k = 0; k < this.DimensionZ; ++k)
                {
                    // Create Node object
                    GameObject node = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    node.transform.localScale = new Vector3(1.0f, 1.0f, 1.0f) * NodeRadius;
                    node.transform.localPosition = start + new Vector3(delta[0] * i, delta[1] * j, delta[2] * k);
                    node.name = "Node" + count;

                    // Set color
                    Renderer renderer = node.GetComponent<Renderer>();
                    renderer.sharedMaterial = new Material(Shader.Find("Standard"));
                    renderer.sharedMaterial.SetColor("_Color", NodeColor);

                    // Add SceneNode component
                    node.AddComponent<SceneNode>();
                    m_mnodesMatrix[i, j, k] = node.GetComponent<SceneNode>();
                    m_mnodesMatrix[i, j, k].IsFixed = this.FixedNode;
                    m_mnodesMatrix[i, j, k].Mass = this.NodeMass;

                    // Add node object to the nodes objects group
                    node.transform.SetParent(nodesGroup.transform, true);

                    vtempNodes.Add(m_mnodesMatrix[i, j, k]);

                    count++;

                    //'Diagonal' nodes (for diagonal Springs)
                    if (this.AddDiagonalSprings && areDiagSprings_doable)
                    {
                        if ((i - 1 >= 0) && (j - 1 >= 0) && (k - 1 >= 0))
                        {
                            // Create Node object
                            GameObject innerNode = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                            innerNode.transform.localScale = new Vector3(1.0f, 1.0f, 1.0f) * NodeRadius;
                            innerNode.transform.localPosition = start + new Vector3(delta[0] * (i-1) +(delta[0] / 2.0f), 
                                                                                    delta[1] * (j-1) +(delta[1] / 2.0f), 
                                                                                    delta[2] * (k-1) +(delta[2] / 2.0f));
                            innerNode.name = "Diagonal Node" + diagCount;

                            // Set color
                            Renderer innerRenderer = innerNode.GetComponent<Renderer>();
                            innerRenderer.sharedMaterial = new Material(Shader.Find("Standard"));
                            innerRenderer.sharedMaterial.SetColor("_Color", NodeColor);

                            // Add SceneNode component
                            innerNode.AddComponent<SceneNode>();
                            m_minnernodesMatrix[i-1, j-1, k-1] = innerNode.GetComponent<SceneNode>();
                            m_minnernodesMatrix[i-1, j-1, k-1].IsFixed = this.FixedNode;
                            m_minnernodesMatrix[i-1, j-1, k-1].Mass = this.NodeMass;

                            // Add node object to the nodes objects group
                            innerNode.transform.SetParent(nodesGroup.transform, true);

                            vtempNodes.Add(m_minnernodesMatrix[i-1, j-1, k-1]);

                            diagCount++;
                        }
                    }
                }
            }
        }

        // Store the nodes in the class variable
        this.m_vnodesVector = new SceneNode[count+diagCount];
        for (int i = 0; i < count+diagCount; ++i) // Store
            this.m_vnodesVector[i] = vtempNodes[i];

        // Create edges

        List<SceneEdge> vtempEdges = new List<SceneEdge>(); ;

        if (this.AddLatticeSprings)
        {
            count = 0;
            for (int i = 0; i < this.DimensionX; ++i)
            {
                for (int j = 0; j < this.DimensionY; ++j)
                {
                    for (int k = 0; k < this.DimensionZ; ++k)
                    {
                        for (int e = 0; e < 3; ++e)
                        {
                            SceneEdge sceneEdge = null;

                            switch (e)
                            {
                                case 0: 
                                    if (i < this.DimensionX - 1)
                                        sceneEdge = addEdge(this.m_mnodesMatrix[i, j, k], this.m_mnodesMatrix[i+1, j, k], count, false);
                                    break;

                                case 1:
                                    if (j < this.DimensionY - 1)
                                        sceneEdge = addEdge(this.m_mnodesMatrix[i, j, k], this.m_mnodesMatrix[i, j + 1, k], count, false);
                                    break;

                                case 2:
                                    if (k < this.DimensionZ - 1)
                                        sceneEdge = addEdge(this.m_mnodesMatrix[i, j, k], this.m_mnodesMatrix[i, j, k + 1], count, false);
                                    break;
                            }

                            if (sceneEdge == null)
                                continue; // Jump

                            // Store the create edge
                            vtempEdges.Add(sceneEdge);

                            // Add the edge object to the edges objects group
                            sceneEdge.transform.SetParent(edgesGroup.transform);

                            count++;
                        }
                    }
                }
            }
        }

        if (this.AddDiagonalSprings && areDiagSprings_doable)
        {
            diagCount = 0;

            for (int i = 0; i < this.DimensionX-1; ++i)
            {
                for (int j = 0; j < this.DimensionY-1; ++j)
                {
                    for (int k = 0; k < this.DimensionZ-1; ++k)
                    {
                        for (int e = 0; e < 8; ++e)
                        {
                            SceneEdge sceneEdge = null;

                            switch (e)
                            {
                                case 0:
                                    sceneEdge = addEdge(this.m_minnernodesMatrix[i, j, k], this.m_mnodesMatrix[i, j, k], diagCount, true);
                                    break;

                                case 1:
									sceneEdge = addEdge(this.m_minnernodesMatrix[i, j, k], this.m_mnodesMatrix[i, j, k+1], diagCount, true);
                                    break;

                                case 2:
									sceneEdge = addEdge(this.m_minnernodesMatrix[i, j, k], this.m_mnodesMatrix[i, j+1, k], diagCount, true);
                                    break;

                                case 3:
									sceneEdge = addEdge(this.m_minnernodesMatrix[i, j, k], this.m_mnodesMatrix[i, j+1, k+1], diagCount, true);
                                    break;

                                case 4:
									sceneEdge = addEdge(this.m_minnernodesMatrix[i, j, k], this.m_mnodesMatrix[i+1, j, k], diagCount, true);
                                    break;

                                case 5:
									sceneEdge = addEdge(this.m_minnernodesMatrix[i, j, k], this.m_mnodesMatrix[i+1, j, k+1], diagCount, true);
                                    break;

                                case 6:
									sceneEdge = addEdge(this.m_minnernodesMatrix[i, j, k], this.m_mnodesMatrix[i+1, j+1, k], diagCount, true);
                                    break;

                                case 7:
									sceneEdge = addEdge(this.m_minnernodesMatrix[i, j, k], this.m_mnodesMatrix[i+1, j+1, k+1], diagCount, true);
                                    break;
                            }

                            if (sceneEdge == null)
                                continue; // Jump

                            // Store the create edge
                            vtempEdges.Add(sceneEdge);

                            // Add the edge object to the edges objects group
                            sceneEdge.transform.SetParent(edgesGroup.transform);

                            diagCount++;
                        }
                    }
                }
            }

        }

    

		// Store the edges in the class variable
		this.m_vedgesVector = new SceneEdge[count+diagCount];
        for (int i = 0; i < count + diagCount; ++i) // Store
			this.m_vedgesVector[i] = vtempEdges[i];

		// Create the mass-spring component
		root.AddComponent<SimulableNodesMS> ();
		SimulableNodesMS MSComponent = root.GetComponent<SimulableNodesMS> ();

		for (int i = 0; i < this.m_vnodesVector.Length; ++i)
			MSComponent.SceneNodes.Add (this.m_vnodesVector [i]);

		for (int i = 0; i < this.m_vedgesVector.Length; ++i)
			MSComponent.SceneEdges.Add (this.m_vedgesVector [i]);

		MSComponent.Kd = this.Kd;
		MSComponent.Ke = this.Ke;
    }

    private SceneEdge addEdge(SceneNode node0, SceneNode node1, int count, bool diag)
    {
        // Create Edge object
        GameObject edge = GameObject.CreatePrimitive(PrimitiveType.Cylinder);

        if (!diag) 
            edge.name = "Edge" + count;
        else
            edge.name = "Diag Edge" + count;

        // Set color
        Renderer renderer = edge.GetComponent<Renderer>();
        renderer.sharedMaterial = new Material(Shader.Find("Standard"));
        renderer.sharedMaterial.SetColor("_Color", EdgeColor);

        // Add SceneEdge component
        edge.AddComponent<SceneEdge>();
        SceneEdge sceneEdge = edge.GetComponent<SceneEdge>();
        sceneEdge.SceneNode0 = node0;
        sceneEdge.SceneNode1 = node1;
       
        // Orient and scale properly 
        Vector3 createRot = new Vector3(0.0f, 1.0f, 0.0f);
        Vector3 springRot = sceneEdge.SceneNode0.transform.position -
							sceneEdge.SceneNode1.transform.position;

        float springScale = springRot.magnitude;
        springRot = springRot * (1.0f / springScale);
        // Cylinders in Unity dimensions by default are height = 2 and radius = 0.5: scale
		edge.transform.localPosition = (node0.transform.position + node1.transform.position) * 0.5f;
        edge.transform.localScale = new Vector3(EdgeRadius, springScale*0.5f, EdgeRadius);
        edge.transform.rotation = Quaternion.FromToRotation(createRot, springRot);

        return sceneEdge;
    }

    #endregion

}
