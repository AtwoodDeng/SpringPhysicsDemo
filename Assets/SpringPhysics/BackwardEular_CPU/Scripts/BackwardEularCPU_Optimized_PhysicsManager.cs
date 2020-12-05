using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Sirenix.OdinInspector;
using System;

public class BackwardEularCPU_Optimized_PhysicsManager: PhysicsManager
{
    public GameObject floor;
    [Header("Simulation")]
    public float K = 1.0f;
    public float G = -9.8f;
    public int simIteration = 5;
    public int solveIteration = 10;
    public float backBeta = 1.0f;
    public float damp = 0;

    [ReadOnly]
    public float floorY = 0;

    public bool is3D = false;

    /// <summary>
    /// Position list
    /// Length : N
    /// </summary>
    V3Buffer[] P;
    /// <summary>
    /// Velocity list 
    /// Length : N 
    /// </summary>
    V3Buffer[] V;

    /// <summary>
    /// temperary buffer Velocity list
    /// Length : N 
    /// </summary>
    V3Buffer[] temV;

    /// <summary>
    /// Jacob matrix ( in array )
    /// Length : N * N 
    /// </summary>
    Matrix3x3Buffer[] J;

    /// <summary>
    /// A matrix ( in array )
    /// Length : N * N 
    /// </summary>
    Matrix3x3Buffer[] A;
    /// <summary>
    /// B matrix ( in array )
    /// Length : N
    /// </summary>
    V3Buffer[] B;

    /// <summary>
    /// temporary buffer
    /// </summary>
    V3Buffer temX, temR, temVV , temF ;

    Matrix3x3Buffer temDiff, temXij;
    Matrix3x3Buffer[] temInvA;

    /// <summary>
    /// rest length of matrix
    /// length : N * N 
    /// </summary>
    float[] restLM;
    /// <summary>
    /// k matrix
    /// length : N * N 
    /// </summary>
    float[] KM;

    bool inited = false;
    int temN = 0;

    public void InitSim()
    {
        temN = N;

        P = new V3Buffer[N];
        V = new V3Buffer[N];
        temV = new V3Buffer[N];

        for (int i = 0; i < N; ++i)
        {
            var body = bodyList[i];
            P[i] = new V3Buffer(body.transform.position);
            V[i] = new V3Buffer(body.velocity);
            temV[i] = new V3Buffer(body.velocity);
        }

        InitConnectionData();

        J = new Matrix3x3Buffer[N * N];
        // init J
        for (int i = 0; i < J.Length; ++i)
            J[i] = new Matrix3x3Buffer();

        A = new Matrix3x3Buffer[N * N];
        // init A
        for (int i = 0; i < A.Length; ++i)
            A[i] = new Matrix3x3Buffer();

        B = new V3Buffer[N];
        // init B
        for (int i = 0; i < B.Length; ++i)
            B[i] = new V3Buffer();

        temInvA = new Matrix3x3Buffer[N];
        for (int i = 0; i < temInvA.Length; ++i)
            temInvA[i] = new Matrix3x3Buffer();

        temX = new V3Buffer();
        temR = new V3Buffer();
        temVV = new V3Buffer();
        temF = new V3Buffer();

        temDiff = new Matrix3x3Buffer();
        temXij = new Matrix3x3Buffer();
    }

    public void InitConnectionData()
    {
        restLM = new float[N * N];
        KM = new float[N * N];

        for (int i = 0; i < connectionList.Count; ++i)
        {
            var conInfo = connectionList[i];
            int ii = conInfo.body0.Index;
            int jj = conInfo.body1.Index;

            restLM[ii * N + jj] = conInfo.restLength;
            KM[ii * N + jj] = conInfo.k;
            restLM[jj * N + ii] = conInfo.restLength;
            KM[jj * N + ii] = conInfo.k;
        }
    }

    public void DampVelocity( float dt )
    {
        float d = Mathf.Exp(-dt * damp );
        for ( int i = 0; i < N; ++ i )
        {
            V[i].Multiply(d);
        }
    }

    public void UpdateJacob()
    {
        // init J
        for (int i = 0; i < J.Length; ++i)
            J[i].Rest();
        //for (int i = 0; i < N ; ++i)
        //    J[i * N + i ].Rest();

        for ( int i = 0; i < N; ++ i )
            for ( int j = 0; j < N; ++ j )
            {
                float restL_ij = restLM[i * N + j];
                float k_ij = KM[i * N + j] * K ;
                if ( restL_ij > 0 )
                {
                    temX.Copy(P[i]);
                    temX.Reduce(P[j]);
                    
                    float x_ij_norm = temX.GetLength();
                    temX.Multiply(1f / x_ij_norm);

                    temXij.InitFromOuterProductOfV3(temX);
                    temDiff.InitI();
                    temDiff.Multiply(1f - restL_ij / x_ij_norm);
                    temDiff.Add(temXij);
                    temDiff.Multiply(-k_ij);

                    J[i * N + i].Add(temDiff);
                    J[i * N + j].Copy(temDiff);
                    J[i * N + j].Multiply(-1f);
                }
            }
    }

    public void UpdateA( float dt )
    {
        // init A

        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
            {
                A[i * N + j].Copy(J[i * N + j]);
                A[i * N + j].Multiply( - backBeta * dt * dt * bodyList[i].invMass );
                if (i == j)
                    A[i * N + j].AddI();
            }
    }

    public void UpdateB( float dt )
    {
        for ( int i = 0; i < N; ++ i )
        {
            temF.Reset();
            temF.data[1] = bodyList[i].mass * G;

            for (int j = 0; j < N; ++j)
            {
                float restL_ij = restLM[i * N + j];
                float k_ij = KM[i * N + j] * K;

                if (restL_ij > 0)
                {
                    temX.Copy(P[i]);
                    temX.Reduce(P[j]);
                    float length = temX.GetLength();
                    temX.Multiply(-k_ij * (length - restL_ij) / length );
                    temF.Add(temX);
                }

            }
            B[i].Copy(temF);
            B[i].Multiply(dt * bodyList[i].invMass);
            B[i].Add(V[i]);
        }
    }
    

    public void Solve()
    {
        for (int i = 0; i < N; ++i)
            temInvA[i].GetInverse(A[i * N + i]);

        for (int k = 0; k < solveIteration; ++k)
        {
            for (int i = 0; i < N; ++i)
                temV[i].Copy(V[i]);

            for ( int i = 0; i < N; ++i )
            {
                temR.Copy(B[i]);
                for ( int j = 0; j < N;  ++ j)
                {
                    if ( i != j )
                    {
                        temR.Reduce(A[i * N + j] * temV[j]);
                    }
                }

                // A[i, i] *v[i] = r
                // temVV.Copy(temInvA[i].Multiply(temR));

                // Use jacob again to solve 
                temVV.Copy(temV[i]);
                for (int t = 0; t < 5; ++t)
                {
                    V[i].Copy(temR);
                    for (int ii = 0; ii < 3; ++ii)
                    {
                        // finalV[i].data[ii] = temR.data[ii];
                        for (int jj = 0; jj < 3; ++jj)
                            if (ii != jj)
                                V[i].data[ii] -= A[i * N + i].Get(ii, jj) * temVV.data[jj];
                        V[i].data[ii] /= A[i * N + i].Get(ii, ii);
                    }

                    temVV.Copy(V[i]);
                }

                V[i].Copy(temVV);
            }

        }

    }


    public void UpdateCollision(float dt)
    {
        floorY = floor.transform.position.y;

        for (int i = 0; i < N; ++i)
        {
            if (P[i].y < floorY)
            {
                P[i].data[1] = floorY;
                V[i].data[1] = V[i].data[1] < 0 ? -V[i].data[1] : V[i].data[1];
            }
        }
    }

    public void UpdatePosition(float dt)
    {

        for (int i = 0; i < N; ++i)
        {
            temVV.Copy(V[i]);
            temVV.Multiply(dt);

            if (temVV.IsNAN())
            {
                Debug.Log("Vel NAN" + i);
                V[i].Reset();
            }
            else
            {
                P[i].Add(temVV);
            }
        }

    }

    public override void UpdateSimulation()
    {
        base.UpdateSimulation();

        if (N != temN)
        {
            InitSim();
        }

        if (N < 3)
            return;


        for (int k = 0; k < simIteration; ++k)
        {
            float dt = Time.deltaTime / simIteration;

            DampVelocity(dt);

            UpdateJacob();

            UpdateA(dt);

            UpdateB(dt);

            Solve();

            UpdateCollision(dt);

            UpdatePosition(dt);
        }
    }
    public override void UpdateVisual()
    {
        base.UpdateVisual();

        for( int i = 0; i < N; ++ i )
        {
            if ( !P[i].IsNAN() )
                bodyList[i].transform.position = is3D ? P[i].ToV3() : P[i].ToV2();
            if (!V[i].IsNAN())
                bodyList[i].velocity = V[i].ToV3();

            //bodyList[i].transform.position = new Vector3(Px[i], Py[i], is3D? Pz[i] : 0 );
            //bodyList[i].velocity = new Vector3(Vx[i], Vy[i], Vz[i]);
        }
    }
}
