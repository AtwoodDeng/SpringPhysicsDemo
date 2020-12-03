using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Sirenix.OdinInspector;
using System;

public class BackwardEularPhysicsManager : PhysicsManager
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

    float[] Px;
    float[] Py;
    float[] Pz;

    float[] Vx;
    float[] Vy;
    float[] Vz;


    float[] Ax;
    float[] Ay;
    float[] Az;

    float[] Bx;
    float[] By;
    float[] Bz;


    V3Buffer[] P;
    V3Buffer[] V;
    V3Buffer[] temV;

    Matrix3x3Buffer[] J;
    Matrix3x3Buffer[] A;
    V3Buffer[] B;

    V3Buffer temR, temVV;

    float[] restLM;
    float[] KM;

    bool inited = false;
    int temBodyCount = 0;

    public void InitSim()
    {
        //Px = new float[BodyCount];
        //Py = new float[BodyCount];
        //Pz = new float[BodyCount];

        //Vx = new float[BodyCount];
        //Vy = new float[BodyCount];
        //Vz = new float[BodyCount];

        //for (int i = 0; i < BodyCount; ++i)
        //{
        //    var body = bodyList[i];
        //    Px[i] = body.transform.position.x;
        //    Py[i] = body.transform.position.y;
        //    Pz[i] = body.transform.position.z;

        //    Vx[i] = body.velocity.x;
        //    Vy[i] = body.velocity.y;
        //    Vz[i] = body.velocity.z;
        //}

        //Ax = new float[BodyCount];
        //Ay = new float[BodyCount];
        //Az = new float[BodyCount];

        //Bx = new float[BodyCount];
        //By = new float[BodyCount];
        //Bz = new float[BodyCount];

        P = new V3Buffer[BodyCount];
        V = new V3Buffer[BodyCount];
        temV = new V3Buffer[BodyCount];

        for (int i = 0; i < BodyCount; ++i)
        {
            var body = bodyList[i];
            P[i] = new V3Buffer(body.transform.position);
            V[i] = new V3Buffer(body.velocity);
            temV[i] = new V3Buffer(body.velocity);
        }

        temBodyCount = BodyCount;

        InitConnectionData();

        J = new Matrix3x3Buffer[BodyCount * BodyCount];

        // init J
        for (int i = 0; i < J.Length; ++i)
            J[i] = new Matrix3x3Buffer();

        A = new Matrix3x3Buffer[BodyCount * BodyCount];

        // init A
        for (int i = 0; i < A.Length; ++i)
            A[i] = new Matrix3x3Buffer();

        B = new V3Buffer[BodyCount];

        // init B
        for (int i = 0; i < B.Length; ++i)
            B[i] = new V3Buffer();

        temR = new V3Buffer();
        temVV = new V3Buffer();
    }

    public void InitConnectionData()
    {
        restLM = new float[BodyCount * BodyCount];
        KM = new float[BodyCount * BodyCount];

        for (int i = 0; i < connectionList.Count; ++i)
        {
            int ii = connectionList[i].body0.Index;
            int jj = connectionList[i].body1.Index;

            restLM[ii * BodyCount + jj] = connectionList[i].restLength;
            KM[ii * BodyCount + jj] = connectionList[i].k;
        }
    }

    public void DampVelocity( float dt )
    {
        float d = Mathf.Exp(-dt * damp );
        for ( int i = 0; i < BodyCount; ++ i )
        {
            V[i].Multiply(d);
        }
    }


    public void UpdateJacob()
    {
        // init J
        for (int i = 0; i < J.Length; ++i)
            J[i].Rest();

        for ( int i = 0; i < BodyCount; ++ i )
            for ( int j = 0; j < BodyCount; ++ j )
            {
                float restL_ij = restLM[i * BodyCount + j];
                float k_ij = KM[i * BodyCount + j] * K ;
                if ( restL_ij > 0 )
                {
                    //float[] x_ij = new float[3] { Px[i] - Px[j] , Py[i] - Py[j] , Pz[i] - Pz[j] };
                    //float x_ij_norm = GetLengthV3(x_ij);
                    //float[] x_ij_bar = GetNormalizeV3(x_ij);
                    // Matrix3x3Buffer diff = -k_ij * ((1 - restL_ij / x_ij_norm) * (Matrix3x3Buffer.CreateI() - x_ij_mat) + x_ij_mat);
                    V3Buffer x_ij = P[i] - P[j];
                    float x_ij_norm = x_ij.GetLength();
                    V3Buffer x_ij_bar = x_ij * (1f / x_ij_norm);
                    Matrix3x3Buffer x_ij_mat = Matrix3x3Buffer.CreateFromOuterProductOfV3(x_ij_bar);
                    Matrix3x3Buffer diff = Matrix3x3Buffer.CreateI();
                    diff.Reduce ( x_ij_mat);
                    diff.Multiply(1f - restL_ij / x_ij_norm);
                    diff.Add(x_ij_mat);
                    diff.Multiply(-k_ij);

                    J[i * BodyCount + i].Add( diff );
                    J[i * BodyCount + j] = diff * (-1f);
                }
            }
    }

    public void UpdateA( float dt )
    {
        // init A
        for (int i = 0; i < A.Length; ++i)
            A[i].Rest();

        for (int i = 0; i < BodyCount; ++i)
            for (int j = 0; j < BodyCount; ++j)
            {
                A[i * BodyCount + j] = - backBeta * dt * dt * bodyList[i].invMass * J[i * BodyCount + j];
                if (i == j)
                    A[i * BodyCount + j].AddI();
            }
    }

    public void UpdateB( float dt )
    {
        for ( int i = 0; i < BodyCount; ++ i )
        {
            V3Buffer F = new V3Buffer();
            for (int j = 0; j < BodyCount; ++j)
            {
                float restL_ij = restLM[i * BodyCount + j];
                float k_ij = KM[i * BodyCount + j] * K;

                if (k_ij > 0)
                {
                    V3Buffer x_ij = P[i] - P[j];
                    float length = x_ij.GetLength();
                    x_ij.Multiply(-k_ij * (length - restL_ij) / length );
                    
                    F.Add(x_ij);
                }

                F.data[1] += bodyList[i].mass * G;
            }
            B[i] = V[i] + dt * bodyList[i].invMass * F;
        }
    }
    //public void DampVelocity(float dt)
    //{
    //    for (int i = 0; i < BodyCount; ++i)
    //    {

    //        //Vx[i] = Mathf.Lerp(Vx[i], 0, damp * dt);
    //        //Vy[i] = Mathf.Lerp(Vy[i], 0, damp * dt);
    //        //if ( is3D )
    //        //    Vz[i] = Mathf.Lerp(Vz[i], 0, damp * dt);
    //    }
    //}
    

    public void Solve( Matrix3x3Buffer[] inA , V3Buffer[] inB , int count , int iteration, ref V3Buffer[] finalV )
    {
        for (int k = 0; k < iteration; ++k)
        {
            for (int i = 0; i < count; ++i)
                temV[i].Copy(finalV[i]);

            for ( int i = 0; i < count; ++i )
            {
                temR.Copy(inB[i]);
                for ( int j = 0; j < count;  ++ j)
                {
                    if ( i != j )
                    {
                        temR.Reduce(inA[i * count + j] * temV[j]);
                    }
                }

                // Use jacob again to solve A[i, i] *v[i] = r
                temVV.Copy(temV[i]);
                for (int t = 0; t < 5; ++t)
                {
                    finalV[i].Copy(temR);
                    for (int ii = 0; ii < 3; ++ii)
                    {
                        // finalV[i].data[ii] = temR.data[ii];
                        for (int jj = 0; jj < 3; ++jj)
                            if (ii != jj)
                                finalV[i].data[ii] -= inA[i * count + i].Get(ii, jj) * temVV.data[jj];
                        finalV[i].data[ii] /= inA[i * count + i].Get(ii, ii);
                    }

                    temVV.Copy(finalV[i]);
                }

                finalV[i].Copy(temVV);
            }

        }
    }

    public void Solve( float[] A , float[] B , int count , ref float[] output )
    {
        float[] temOut = new float[count];

        for ( int k = 0; k < solveIteration; ++ k )
        {
            for (int i = 0; i < count; ++i) temOut[i] = output[i];

            for (int i = 0; i < count; ++i)
            {
                float r = B[i];
                for (int j = 0; j < count; ++j)
                {
                    if (i != j)
                    {
                        r -= A[j] * temOut[j];
                    }
                }
                output[i] = r / (1 + A[i]);
            }
        }
    }


    public void UpdateCollision(float dt)
    {
        floorY = floor.transform.position.y;

        for (int i = 0; i < BodyCount; ++i)
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
        for (int i = 0; i < BodyCount; ++i)
        {
            P[i].Add(V[i] * dt);
        }

    }

    public override void UpdateSimulation()
    {
        base.UpdateSimulation();

        if (BodyCount != temBodyCount)
        {
            InitSim();
        }

        if (BodyCount < 1)
            return;


        for (int k = 0; k < simIteration; ++k)
        {
            float dt = Time.deltaTime / simIteration;

            DampVelocity(dt);

            UpdateJacob();

            UpdateA(dt);

            UpdateB(dt);

            Solve(A, B, BodyCount, solveIteration, ref V);

            //for( int i = 0; i < BodyCount; ++ i )
            //{
            //    Bx[i] = Vx[i];
            //    By[i] = Vy[i] + G * dt ;
            //    Bz[i] = Vz[i];
            //}

            //for (int i = 0; i < connectionList.Count; ++i)
            //{
            //    var t0 = connectionList[i].body0.Index;
            //    var m0 = connectionList[i].body0.invMass;
            //    var t1 = connectionList[i].body1.Index;
            //    var m1 = connectionList[i].body1.invMass;
            //    var Kc = K * connectionList[i].k;
            //    var restL = connectionList[i].restLength;

            //    Ax[t0] += Kc * dt * dt * m0 * backBeta;
            //    Ay[t0] += Kc * dt * dt * m0 * backBeta;
            //    if (is3D)
            //        Az[t0] += Kc * dt * dt * m0 * backBeta;

            //    Ax[t1] += Kc * dt * dt * m1 * backBeta;
            //    Ay[t1] += Kc * dt * dt * m1 * backBeta;
            //    if (is3D)
            //        Az[t1] += Kc * dt * dt * m1 * backBeta;

            //    Bx[t0] += (Px[t0] < Px[t1] ? (Px[t1] - Px[t0] - restL) : -(Px[t0] - Px[t1] - restL) ) * Kc * dt * m0;
            //    Bx[t1] += (Px[t1] < Px[t0] ? (Px[t0] - Px[t1] - restL) : -(Px[t1] - Px[t0] - restL) ) * Kc * dt * m1;

            //    By[t0] += (Py[t0] < Py[t1] ? (Py[t1] - Py[t0] - restL) : -(Py[t0] - Py[t1] - restL)) * Kc * dt * m0;
            //    By[t1] += (Py[t1] < Py[t0] ? (Py[t0] - Py[t1] - restL) : -(Py[t1] - Py[t0] - restL)) * Kc * dt * m1;

            //    if (is3D)
            //    {
            //        Bz[t0] += (Pz[t0] < Pz[t1] ? (Pz[t1] - Pz[t0] - restL) : -(Pz[t0] - Pz[t1] - restL)) * Kc * dt * m0;
            //        Bz[t1] += (Pz[t1] < Pz[t0] ? (Pz[t0] - Pz[t1] - restL) : -(Pz[t1] - Pz[t0] - restL)) * Kc * dt * m1;
            //    }
            //}

            //Solve(Ax, Bx, BodyCount, ref Vx);
            //Solve(Ay, By, BodyCount, ref Vy);
            //if (is3D)
            //    Solve(Az, Bz, BodyCount, ref Vz);

            UpdateCollision(dt);


            UpdatePosition(dt);
        }
    }
    public override void UpdateVisual()
    {
        base.UpdateVisual();

        for( int i = 0; i < BodyCount; ++ i )
        {
            bodyList[i].transform.position = is3D ? P[i].ToV3() : P[i].ToV2();
            bodyList[i].velocity = P[i].ToV3();

            //bodyList[i].transform.position = new Vector3(Px[i], Py[i], is3D? Pz[i] : 0 );
            //bodyList[i].velocity = new Vector3(Vx[i], Vy[i], Vz[i]);
        }
    }
}
