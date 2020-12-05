using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Sirenix.OdinInspector;
using System.Numerics;
using Sirenix.Utilities;
using System;

public class ConnectionBarRT
{
    public BodyCenter body0;
    public BodyCenter body1;
    public float k = 1.0f;
    public float restLength = 0;

    public ConnectionBarRT(BodyCenter _body0, BodyCenter _body1 , float _k = 1.0f )
    {
        this.body0 = _body0;
        this.body1 = _body1;
        this.k = _k;
        restLength = (body0.transform.position - body1.transform.position).magnitude;
    }
}

public class V3Buffer
{

    public float[] data;
    public float x { get { return data[0];  } set { data[0] = value; } }
    public float y { get { return data[1]; } set { data[1] = value; } }
    public float z { get { return data[2]; } set { data[2] = value; } }

    public void Reset()
    {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
    }

    public void Copy( V3Buffer other )
    {
        data[0] = other.data[0];
        data[1] = other.data[1];
        data[2] = other.data[2];
    }

    public V3Buffer()
    {
        data = new float[3];
        Reset();
    }
    public V3Buffer(V3Buffer other )
    {
        data = new float[3];
        data[0] = other.data[0];
        data[1] = other.data[1];
        data[2] = other.data[2];
    }


    public V3Buffer(float x , float y , float z )
    {
        data = new float[3];
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }

    public V3Buffer(UnityEngine.Vector3 input )
    {
        data = new float[3];
        data[0] = input.x;
        data[1] = input.y;
        data[2] = input.z;
    }

    public void Add( V3Buffer other )
    {
        data[0] += other.data[0];
        data[1] += other.data[1];
        data[2] += other.data[2];
    }


    public void Reduce(V3Buffer other)
    {
        data[0] -= other.data[0];
        data[1] -= other.data[1];
        data[2] -= other.data[2];
    }
    public float GetLength()
    {
        return Mathf.Sqrt(x * x + y * y + z * z);
    }

    public V3Buffer GetNormalize()
    {
        float length = GetLength();
        return new V3Buffer(x / length, y / length, z / length);
    }

    public UnityEngine.Vector3 ToV3( )
    {
        return new UnityEngine.Vector3(x, y, z);
    }

    public UnityEngine.Vector3 ToV2()
    {
        return new UnityEngine.Vector3(x, y, 0);
    }

    public static V3Buffer operator +(V3Buffer a, V3Buffer b)
    {
        V3Buffer result = new V3Buffer();
        result.data[0] = a.data[0] + b.data[0];
        result.data[1] = a.data[1] + b.data[1];
        result.data[2] = a.data[2] + b.data[2];

        return result;
    }

    public static V3Buffer operator -(V3Buffer a, V3Buffer b)
    {
        V3Buffer result = new V3Buffer();
        result.data[0] = a.data[0] - b.data[0];
        result.data[1] = a.data[1] - b.data[1];
        result.data[2] = a.data[2] - b.data[2];

        return result;
    }
    public static V3Buffer operator *(V3Buffer a, float k)
    {
        V3Buffer result = new V3Buffer();
        result.data[0] = a.data[0] * k;
        result.data[1] = a.data[1] * k;
        result.data[2] = a.data[2] * k;

        return result;
    }
    public static V3Buffer operator *(float k , V3Buffer a)
    {
        V3Buffer result = new V3Buffer();
        result.data[0] = a.data[0] * k;
        result.data[1] = a.data[1] * k;
        result.data[2] = a.data[2] * k;

        return result;
    }


    public void Multiply( float k )
    {
        data[0] *= k;
        data[1] *= k;
        data[2] *= k;
    }

    public override string ToString()
    {
        return string.Format("({0:0.00},{1:0.00},{2:0.00})", x, y, z);
    }

    public bool IsNAN()
    {
        return float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z);
    }
}


public class Matrix3x3Buffer
{
    public float[] data;

    public void Rest()
    {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
    }

    public Matrix3x3Buffer()
    {
        data = new float[9];
        Rest();
    }

    public void Add( Matrix3x3Buffer other )
    {
        for (int i = 0; i < 9; ++i)
            data[i] += other.data[i];
    }

    public void Reduce(Matrix3x3Buffer other)
    {
        for (int i = 0; i < 9; ++i)
            data[i] -= other.data[i];
    }

    public void Copy( Matrix3x3Buffer other )
    {
        data[0] = other.data[0];
        data[1] = other.data[1];
        data[2] = other.data[2];
        data[3] = other.data[3];
        data[4] = other.data[4];
        data[5] = other.data[5];
        data[6] = other.data[6];
        data[7] = other.data[7];
        data[8] = other.data[8];
    }
    public void AddI( )
    {
        data[0] += 1;
        data[4] += 1;
        data[8] += 1;

    }

    static public Matrix3x3Buffer CreateFromOuterProductOfV3 ( float[] input )
    {
        if (input.Length != 3)
            return new Matrix3x3Buffer();

        var result = new Matrix3x3Buffer();

        for( int i = 0; i < 3; ++ i)
            for (int j = 0; j < 3; ++j)
            {
                result.data[i * 3 + j] = input[i] * input[j];
            }

        return result;
    }

    public void InitFromOuterProductOfV3(V3Buffer x)
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                data[i * 3 + j] = x.data[i] * x.data[j];
            }
    }
    static public Matrix3x3Buffer CreateFromOuterProductOfV3(V3Buffer x)
    {
        var result = new Matrix3x3Buffer();

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                result.data[i * 3 + j] = x.data[i] * x.data[j];
            }

        return result;
    }
    public float[] Multiply( float[] input )
    {
        if (input.Length != 3)
            return new float[3];

        float[] result = new float[3];

        for (int i = 0; i < 3; ++i)
        {
            result[i] = 0;
            for (int j = 0; j < 3; ++j)
            {
                result[i] += data[i * 3 + j] * input[j];
            }
        }
        return result;
    }

    public V3Buffer Multiply(V3Buffer input)
    {
        V3Buffer result = new V3Buffer();

        for (int i = 0; i < 3; ++i)
        {
            result.data[i] = 0;
            for (int j = 0; j < 3; ++j)
            {
                result.data[i] += data[i * 3 + j] * input.data[j];
            }
        }
        return result;
    }
    public void Multiply( float value )
    {
        for (int i = 0; i < 9; ++i)
            data[i] *= value;
    }

    public void InitI()
    {
        data[0] = 1;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 1;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        data[8] = 1;
    }

    static public Matrix3x3Buffer CreateI()
    {
        var result = new Matrix3x3Buffer();

        result.data[0] = 1;
        result.data[4] = 1;
        result.data[8] = 1;

        return result;
    }

    public static V3Buffer operator *(Matrix3x3Buffer a, V3Buffer b)
    {
        return a.Multiply(b);
    }
    public static Matrix3x3Buffer operator +(Matrix3x3Buffer a , Matrix3x3Buffer b)
    {
        Matrix3x3Buffer result = new Matrix3x3Buffer();
        for (int i = 0; i < 9; ++i)
            result.data[i] = a.data[i] + b.data[i];

        return result;
    }

    public static Matrix3x3Buffer operator -(Matrix3x3Buffer a, Matrix3x3Buffer b)
    {
        Matrix3x3Buffer result = new Matrix3x3Buffer();
        for (int i = 0; i < 9; ++i)
            result.data[i] = a.data[i] - b.data[i];

        return result;
    }

    public static Matrix3x3Buffer operator *(Matrix3x3Buffer a, float b )
    {
        Matrix3x3Buffer result = new Matrix3x3Buffer();
        for (int i = 0; i < 9; ++i)
            result.data[i] = a.data[i] * b ;

        return result;
    }
    public static Matrix3x3Buffer operator *( float b , Matrix3x3Buffer a)
    {
        Matrix3x3Buffer result = new Matrix3x3Buffer();
        for (int i = 0; i < 9; ++i)
            result.data[i] = a.data[i] * b;

        return result;
    }
    public static Matrix3x3Buffer operator / (Matrix3x3Buffer a, float b)
    {
        Matrix3x3Buffer result = new Matrix3x3Buffer();
        for (int i = 0; i < 9; ++i)
            result.data[i] = a.data[i] / b;

        return result;
    }

    public void GetInverse( Matrix3x3Buffer other )
    {
        var m = new UnityEngine.Matrix4x4();
        m.m00 = other.data[0];
        m.m01 = other.data[1];
        m.m02 = other.data[2];
        m.m10 = other.data[3];
        m.m11 = other.data[4];
        m.m12 = other.data[5];
        m.m20 = other.data[6];
        m.m21 = other.data[7];
        m.m22 = other.data[8];
        m.m33 = 1f;

        m = m.inverse;

        data[0] = m.m00;
        data[1] = m.m01;
        data[2] = m.m02;
        data[3] = m.m10;
        data[4] = m.m11;
        data[5] = m.m12;
        data[6] = m.m20;
        data[7] = m.m21;
        data[8] = m.m22;
    }

    public float Get(int i, int j) { return data[i * 3 + j]; }

}


public class PhysicsManager : MonoBehaviour
{
    private static PhysicsManager m_Instance;

    public static PhysicsManager Instance
    {
        get
        {
            if (m_Instance == null)
                m_Instance = FindObjectOfType<PhysicsManager>();

            return m_Instance;
        }
    }

    public List<BodyCenter> bodyList = new List<BodyCenter>();
    public List<ConnectionBarRT> connectionList = new List<ConnectionBarRT>();

    /// <summary>
    /// Number of the body in the system
    /// </summary>
    public int N {  get { return bodyList.Count;  } }


    virtual public void Register(BodyCenter body)
    {

        if (bodyList == null)
            bodyList = new List<BodyCenter>();
        if (connectionList == null)
            connectionList = new List<ConnectionBarRT>();
        bodyList.Add(body);
        body.Index = bodyList.Count - 1;

        foreach (var other in bodyList)
        {
            if (other != body)
            {
                foreach (var neighbour in body.neighbours)
                {
                    if (other == neighbour)
                    {
                        var connection = new ConnectionBarRT( other , body );
                        connectionList.Add(connection);
                    }
                }
            }
        }
    }


    virtual public void Unregister(BodyCenter body)
    {
        // TODO : need to deal with the index 
        // bodyList.Remove(body);
    }
    public void Update()
    {
        UpdateSimulation();

        UpdateVisual();
    }

    virtual public void UpdateSimulation()
    {
        
    }

    virtual public void UpdateVisual()
    {

    }

    public static float GetLengthV3( float[] x )
    {
        if (x.Length != 3) return 0;
        return Mathf.Sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    }

    public static float[] GetNormalizeV3(float[] x)
    {
        if (x.Length != 3) return new float[3];

        var length = GetLengthV3(x);
        var result = new float[3] { x[0] / length , x[1] / length , x[2] / length } ;
        return result;
    }

    public static float[] MultiplyV3( float[] x , float k )
    {
        if (x.Length != 3) return new float[3];

        return new float[3] { x[0] * k , x[1] * k , x[2] * k };

    }
}
