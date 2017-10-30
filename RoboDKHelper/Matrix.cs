using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net.Sockets;


/********************************************************************************************
 *
 *
 *
 *
 *
 *******************************************************************************************/



namespace RoboDKHelper
{
    #region MatException Class

    /// <summary>
    /// 矩阵异常类
    /// </summary>
    public class MatException : Exception
    {
        /// <summary>
        /// 实例化一个矩阵的异常类
        /// </summary>
        /// <param name="Message"></param>
        public MatException(string Message)
            : base(Message)
        {

        }
    }
    
    #endregion

    /// <summary>
    /// Matrix class for robotics. 
    /// 简单矩阵类用于均匀运算
    /// </summary>
    public class Matrix
    {
        #region 基础属性

        /// <summary>
        /// 矩阵的行数
        /// </summary>
        public int rows;
        /// <summary>
        /// 矩阵的列数
        /// </summary>
        public int cols;
        /// <summary>
        /// 矩阵的值
        /// </summary>
        public double[,] mat;

        #endregion

        #region 构造方法

        /// <summary>
        /// 任何大小矩阵的矩阵类构造函数，初始为 0
        /// </summary>
        /// <param name="Rows">dimension 1 size (rows)</param>
        /// <param name="Cols">dimension 2 size (columns)</param>
        public Matrix(int Rows, int Cols)         // Matrix Class constructor
        {
            rows = Rows;
            cols = Cols;
            mat = new double[rows, cols];
        }

        /// <summary>
        /// 4x4均匀矩阵的矩阵类构造函数
        /// </summary>
        /// <param name="nx">Position [0,0]</param>
        /// <param name="ox">Position [0,1]</param>
        /// <param name="ax">Position [0,2]</param>
        /// <param name="tx">Position [0,3]</param>
        /// <param name="ny">Position [1,0]</param>
        /// <param name="oy">Position [1,1]</param>
        /// <param name="ay">Position [1,2]</param>
        /// <param name="ty">Position [1,3]</param>
        /// <param name="nz">Position [2,0]</param>
        /// <param name="oz">Position [2,1]</param>
        /// <param name="az">Position [2,2]</param>
        /// <param name="tz">Position [2,3]</param>
        public Matrix(double nx, double ox, double ax, double tx, double ny, double oy, double ay, double ty, double nz, double oz, double az, double tz)         // Matrix Class constructor
        {
            rows = 4;
            cols = 4;
            mat = new double[rows, cols];
            mat[0, 0] = nx; mat[1, 0] = ny; mat[2, 0] = nz; mat[3, 0] = 0.0;
            mat[0, 1] = ox; mat[1, 1] = oy; mat[2, 1] = oz; mat[3, 1] = 0.0;
            mat[0, 2] = ax; mat[1, 2] = ay; mat[2, 2] = az; mat[3, 2] = 0.0;
            mat[0, 3] = tx; mat[1, 3] = ty; mat[2, 3] = tz; mat[3, 3] = 1.0;

        }

        /// <summary>
        /// 从另一个矩阵返回一个新的矩阵
        /// </summary>
        /// <param name="pose">矩阵</param>
        /// <exception cref="ArgumentNullException"></exception>
        public Matrix(Matrix pose)
        {
            rows = pose.rows;
            cols = pose.cols;

            mat = new double[rows, cols];

            // 复制
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    mat[i, j] = pose[i, j];
        }

        /// <summary>
        /// Matrix class constructor for a 4x1 vector [x,y,z,1]
        /// 构造一个4*1的一维矩阵
        /// </summary>
        /// <param name="x">x coordinate</param>
        /// <param name="y">y coordinate</param>
        /// <param name="z">z coordinate</param>
        public Matrix(double x, double y, double z)
        {
            rows = 4;
            cols = 1;
            mat = new double[rows, cols];
            mat[0, 0] = x;
            mat[1, 0] = y;
            mat[2, 0] = z;
            mat[3, 0] = 1.0;
        }


        #endregion
        
        #region 运算支持

        private static void SafeAplusBintoC(Matrix A, int xa, int ya, Matrix B, int xb, int yb, Matrix C, int size)
        {
            for (int i = 0; i < size; i++)          // rows
                for (int j = 0; j < size; j++)     // cols
                {
                    C[i, j] = 0;
                    if (xa + j < A.cols && ya + i < A.rows) C[i, j] += A[ya + i, xa + j];
                    if (xb + j < B.cols && yb + i < B.rows) C[i, j] += B[yb + i, xb + j];
                }
        }

        private static void SafeAminusBintoC(Matrix A, int xa, int ya, Matrix B, int xb, int yb, Matrix C, int size)
        {
            for (int i = 0; i < size; i++)          // rows
                for (int j = 0; j < size; j++)     // cols
                {
                    C[i, j] = 0;
                    if (xa + j < A.cols && ya + i < A.rows) C[i, j] += A[ya + i, xa + j];
                    if (xb + j < B.cols && yb + i < B.rows) C[i, j] -= B[yb + i, xb + j];
                }
        }

        private static void SafeACopytoC(Matrix A, int xa, int ya, Matrix C, int size)
        {
            for (int i = 0; i < size; i++)          // rows
                for (int j = 0; j < size; j++)     // cols
                {
                    C[i, j] = 0;
                    if (xa + j < A.cols && ya + i < A.rows) C[i, j] += A[ya + i, xa + j];
                }
        }

        private static void AplusBintoC(Matrix A, int xa, int ya, Matrix B, int xb, int yb, Matrix C, int size)
        {
            for (int i = 0; i < size; i++)          // rows
                for (int j = 0; j < size; j++) C[i, j] = A[ya + i, xa + j] + B[yb + i, xb + j];
        }

        private static void AminusBintoC(Matrix A, int xa, int ya, Matrix B, int xb, int yb, Matrix C, int size)
        {
            for (int i = 0; i < size; i++)          // rows
                for (int j = 0; j < size; j++) C[i, j] = A[ya + i, xa + j] - B[yb + i, xb + j];
        }

        private static void ACopytoC(Matrix A, int xa, int ya, Matrix C, int size)
        {
            for (int i = 0; i < size; i++)          // rows
                for (int j = 0; j < size; j++) C[i, j] = A[ya + i, xa + j];
        }

        private static Matrix StrassenMultiply(Matrix A, Matrix B)                // Smart matrix multiplication
        {
            if (A.cols != B.rows) throw new MatException("Wrong dimension of matrix!");

            Matrix R;

            int msize = Math.Max(Math.Max(A.rows, A.cols), Math.Max(B.rows, B.cols));

            if (msize < 32)
            {
                R = ZeroMatrix(A.rows, B.cols);
                for (int i = 0; i < R.rows; i++)
                    for (int j = 0; j < R.cols; j++)
                        for (int k = 0; k < A.cols; k++)
                            R[i, j] += A[i, k] * B[k, j];
                return R;
            }

            int size = 1; int n = 0;
            while (msize > size) { size *= 2; n++; };
            int h = size / 2;


            Matrix[,] mField = new Matrix[n, 9];

            /*
             *  8x8, 8x8, 8x8, ...
             *  4x4, 4x4, 4x4, ...
             *  2x2, 2x2, 2x2, ...
             *  . . .
             */

            int z;
            for (int i = 0; i < n - 4; i++)          // rows
            {
                z = (int)Math.Pow(2, n - i - 1);
                for (int j = 0; j < 9; j++) mField[i, j] = new Matrix(z, z);
            }

            SafeAplusBintoC(A, 0, 0, A, h, h, mField[0, 0], h);
            SafeAplusBintoC(B, 0, 0, B, h, h, mField[0, 1], h);
            StrassenMultiplyRun(mField[0, 0], mField[0, 1], mField[0, 1 + 1], 1, mField); // (A11 + A22) * (B11 + B22);

            SafeAplusBintoC(A, 0, h, A, h, h, mField[0, 0], h);
            SafeACopytoC(B, 0, 0, mField[0, 1], h);
            StrassenMultiplyRun(mField[0, 0], mField[0, 1], mField[0, 1 + 2], 1, mField); // (A21 + A22) * B11;

            SafeACopytoC(A, 0, 0, mField[0, 0], h);
            SafeAminusBintoC(B, h, 0, B, h, h, mField[0, 1], h);
            StrassenMultiplyRun(mField[0, 0], mField[0, 1], mField[0, 1 + 3], 1, mField); //A11 * (B12 - B22);

            SafeACopytoC(A, h, h, mField[0, 0], h);
            SafeAminusBintoC(B, 0, h, B, 0, 0, mField[0, 1], h);
            StrassenMultiplyRun(mField[0, 0], mField[0, 1], mField[0, 1 + 4], 1, mField); //A22 * (B21 - B11);

            SafeAplusBintoC(A, 0, 0, A, h, 0, mField[0, 0], h);
            SafeACopytoC(B, h, h, mField[0, 1], h);
            StrassenMultiplyRun(mField[0, 0], mField[0, 1], mField[0, 1 + 5], 1, mField); //(A11 + A12) * B22;

            SafeAminusBintoC(A, 0, h, A, 0, 0, mField[0, 0], h);
            SafeAplusBintoC(B, 0, 0, B, h, 0, mField[0, 1], h);
            StrassenMultiplyRun(mField[0, 0], mField[0, 1], mField[0, 1 + 6], 1, mField); //(A21 - A11) * (B11 + B12);

            SafeAminusBintoC(A, h, 0, A, h, h, mField[0, 0], h);
            SafeAplusBintoC(B, 0, h, B, h, h, mField[0, 1], h);
            StrassenMultiplyRun(mField[0, 0], mField[0, 1], mField[0, 1 + 7], 1, mField); // (A12 - A22) * (B21 + B22);

            R = new Matrix(A.rows, B.cols);                  // result

            // C11
            for (int i = 0; i < Math.Min(h, R.rows); i++)          // rows
                for (int j = 0; j < Math.Min(h, R.cols); j++)     // cols
                    R[i, j] = mField[0, 1 + 1][i, j] + mField[0, 1 + 4][i, j] - mField[0, 1 + 5][i, j] + mField[0, 1 + 7][i, j];

            // C12
            for (int i = 0; i < Math.Min(h, R.rows); i++)          // rows
                for (int j = h; j < Math.Min(2 * h, R.cols); j++)     // cols
                    R[i, j] = mField[0, 1 + 3][i, j - h] + mField[0, 1 + 5][i, j - h];

            // C21
            for (int i = h; i < Math.Min(2 * h, R.rows); i++)          // rows
                for (int j = 0; j < Math.Min(h, R.cols); j++)     // cols
                    R[i, j] = mField[0, 1 + 2][i - h, j] + mField[0, 1 + 4][i - h, j];

            // C22
            for (int i = h; i < Math.Min(2 * h, R.rows); i++)          // rows
                for (int j = h; j < Math.Min(2 * h, R.cols); j++)     // cols
                    R[i, j] = mField[0, 1 + 1][i - h, j - h] - mField[0, 1 + 2][i - h, j - h] + mField[0, 1 + 3][i - h, j - h] + mField[0, 1 + 6][i - h, j - h];

            return R;
        }

        // function for square matrix 2^N x 2^N

        private static void StrassenMultiplyRun(Matrix A, Matrix B, Matrix C, int l, Matrix[,] f)    // A * B into C, level of recursion, matrix field
        {
            int size = A.rows;
            int h = size / 2;

            if (size < 32)
            {
                for (int i = 0; i < C.rows; i++)
                    for (int j = 0; j < C.cols; j++)
                    {
                        C[i, j] = 0;
                        for (int k = 0; k < A.cols; k++) C[i, j] += A[i, k] * B[k, j];
                    }
                return;
            }

            AplusBintoC(A, 0, 0, A, h, h, f[l, 0], h);
            AplusBintoC(B, 0, 0, B, h, h, f[l, 1], h);
            StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 1], l + 1, f); // (A11 + A22) * (B11 + B22);

            AplusBintoC(A, 0, h, A, h, h, f[l, 0], h);
            ACopytoC(B, 0, 0, f[l, 1], h);
            StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 2], l + 1, f); // (A21 + A22) * B11;

            ACopytoC(A, 0, 0, f[l, 0], h);
            AminusBintoC(B, h, 0, B, h, h, f[l, 1], h);
            StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 3], l + 1, f); //A11 * (B12 - B22);

            ACopytoC(A, h, h, f[l, 0], h);
            AminusBintoC(B, 0, h, B, 0, 0, f[l, 1], h);
            StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 4], l + 1, f); //A22 * (B21 - B11);

            AplusBintoC(A, 0, 0, A, h, 0, f[l, 0], h);
            ACopytoC(B, h, h, f[l, 1], h);
            StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 5], l + 1, f); //(A11 + A12) * B22;

            AminusBintoC(A, 0, h, A, 0, 0, f[l, 0], h);
            AplusBintoC(B, 0, 0, B, h, 0, f[l, 1], h);
            StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 6], l + 1, f); //(A21 - A11) * (B11 + B12);

            AminusBintoC(A, h, 0, A, h, h, f[l, 0], h);
            AplusBintoC(B, 0, h, B, h, h, f[l, 1], h);
            StrassenMultiplyRun(f[l, 0], f[l, 1], f[l, 1 + 7], l + 1, f); // (A12 - A22) * (B21 + B22);

            // C11
            for (int i = 0; i < h; i++)          // rows
                for (int j = 0; j < h; j++)     // cols
                    C[i, j] = f[l, 1 + 1][i, j] + f[l, 1 + 4][i, j] - f[l, 1 + 5][i, j] + f[l, 1 + 7][i, j];

            // C12
            for (int i = 0; i < h; i++)          // rows
                for (int j = h; j < size; j++)     // cols
                    C[i, j] = f[l, 1 + 3][i, j - h] + f[l, 1 + 5][i, j - h];

            // C21
            for (int i = h; i < size; i++)          // rows
                for (int j = 0; j < h; j++)     // cols
                    C[i, j] = f[l, 1 + 2][i - h, j] + f[l, 1 + 4][i - h, j];

            // C22
            for (int i = h; i < size; i++)          // rows
                for (int j = h; j < size; j++)     // cols
                    C[i, j] = f[l, 1 + 1][i - h, j - h] - f[l, 1 + 2][i - h, j - h] + f[l, 1 + 3][i - h, j - h] + f[l, 1 + 6][i - h, j - h];
        }

        private static Matrix Multiply(double n, Matrix m)                          // Multiplication by constant n
        {
            Matrix r = new Matrix(m.rows, m.cols);
            for (int i = 0; i < m.rows; i++)
                for (int j = 0; j < m.cols; j++)
                    r[i, j] = m[i, j] * n;
            return r;
        }
        private static Matrix Add(Matrix m1, Matrix m2)         // Add matrix
        {
            if (m1.rows != m2.rows || m1.cols != m2.cols) throw new MatException("Matrices must have the same dimensions!");
            Matrix r = new Matrix(m1.rows, m1.cols);
            for (int i = 0; i < r.rows; i++)
                for (int j = 0; j < r.cols; j++)
                    r[i, j] = m1[i, j] + m2[i, j];
            return r;
        }

        public static Matrix operator -(Matrix m)
        {
            return Multiply(-1, m);
        }

        /// <summary>
        /// 两个矩阵相加
        /// </summary>
        /// <param name="m1"></param>
        /// <param name="m2"></param>
        /// <returns></returns>
        public static Matrix operator +(Matrix m1, Matrix m2)
        {
            return Add(m1, m2);
        }

        /// <summary>
        /// 两个矩阵相减
        /// </summary>
        /// <param name="m1"></param>
        /// <param name="m2"></param>
        /// <returns></returns>
        public static Matrix operator -(Matrix m1, Matrix m2)
        {
            return Add(m1, -m2);
        }

        /// <summary>
        /// 两个矩阵相乘
        /// </summary>
        /// <param name="m1"></param>
        /// <param name="m2"></param>
        /// <returns></returns>
        public static Matrix operator *(Matrix m1, Matrix m2)
        {
            return StrassenMultiply(m1, m2);
        }

        /// <summary>
        /// 矩阵乘以一个常量
        /// </summary>
        /// <param name="n"></param>
        /// <param name="m"></param>
        /// <returns></returns>
        public static Matrix operator *(double n, Matrix m)
        {
            return Multiply(n, m);
        }

        #endregion
        
        #region 静态方法
        
        /// <summary>
        /// 获得一个平移矩阵
        /// </summary>
        /// <param name="x">translation along X (mm)</param>
        /// <param name="y">translation along Y (mm)</param>
        /// <param name="z">translation along Z (mm)</param>
        /// <returns></returns>
        public static Matrix Transl(double x, double y, double z)
        {
            Matrix mat = IdentityMatrix(4, 4);
            mat.SetPosition(x, y, z);
            return mat;
        }

        /// <summary>
        /// 获取一个指定行列但数据都是0的矩阵
        /// </summary>
        /// <param name="iRows"></param>
        /// <param name="iCols"></param>
        /// <returns></returns>
        public static Matrix ZeroMatrix(int iRows, int iCols)       // Function generates the zero matrix
        {
            Matrix matrix = new Matrix(iRows, iCols);
            for (int i = 0; i < iRows; i++)
                for (int j = 0; j < iCols; j++)
                    matrix[i, j] = 0;
            return matrix;
        }

        /// <summary>
        /// 获取一个指定行列的单位矩阵
        /// </summary>
        /// <param name="iRows"></param>
        /// <param name="iCols"></param>
        /// <returns></returns>
        public static Matrix IdentityMatrix(int iRows, int iCols)   // Function generates the identity matrix
        {
            Matrix matrix = ZeroMatrix(iRows, iCols);
            for (int i = 0; i < Math.Min(iRows, iCols); i++)
                matrix[i, i] = 1;
            return matrix;
        }

        /// <summary>
        /// Returns an identity 4x4 matrix (homogeneous matrix)
        /// 返回一个身份4x4矩阵（均匀矩阵）
        /// </summary>
        /// <returns></returns>
        public static Matrix Identity4x4()
        {
            return IdentityMatrix(4, 4);
        }

        /// <summary>
        /// 对指定的矩阵进行转置
        /// </summary>
        /// <param name="m"></param>
        /// <returns></returns>
        public static Matrix Transpose(Matrix m)
        {
            Matrix t = new Matrix(m.cols, m.rows);
            for (int i = 0; i < m.rows; i++)
                for (int j = 0; j < m.cols; j++)
                    t[j, i] = m[i, j];
            return t;
        }

        /// <summary>
        /// 获得一个沿X轴方向旋转的矩阵
        /// </summary>
        /// <param name="rx">rotation around X axis (in radians)</param>
        /// <returns></returns>
        public static Matrix Rotx(double rx)
        {
            double cx = Math.Cos(rx);
            double sx = Math.Sin(rx);
            return new Matrix(1, 0, 0, 0, 0, cx, -sx, 0, 0, sx, cx, 0);
        }

        /// <summary>
        /// 获得一个沿Y轴方向旋转的矩阵
        /// </summary>
        /// <param name="ry">rotation around Y axis (in radians)</param>
        /// <returns></returns>
        public static Matrix Roty(double ry)
        {
            double cy = Math.Cos(ry);
            double sy = Math.Sin(ry);
            return new Matrix(cy, 0, sy, 0, 0, 1, 0, 0, -sy, 0, cy, 0);
        }

        /// <summary>
        /// 获得一个沿Z轴方向旋转的矩阵
        /// </summary>
        /// <param name="rz">rotation around Z axis (in radians)</param>
        /// <returns></returns>
        public static Matrix Rotz(double rz)
        {
            double cz = Math.Cos(rz);
            double sz = Math.Sin(rz);
            return new Matrix(cz, -sz, 0, 0, sz, cz, 0, 0, 0, 0, 1, 0);
        }



        /// <summary>
        /// 从位置及欧拉角获取一个等效矩阵，结果等效于: H = transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="w"></param>
        /// <param name="p"></param>
        /// <param name="r"></param>
        /// <returns>Homogeneous matrix (4x4)</returns>
        public static Matrix FromXYZRPW(double x, double y, double z, double w, double p, double r)
        {
            double a = r * Math.PI / 180.0;
            double b = p * Math.PI / 180.0;
            double c = w * Math.PI / 180.0;
            double ca = Math.Cos(a);
            double sa = Math.Sin(a);
            double cb = Math.Cos(b);
            double sb = Math.Sin(b);
            double cc = Math.Cos(c);
            double sc = Math.Sin(c);
            return new Matrix(cb * cc, cc * sa * sb - ca * sc, sa * sc + ca * cc * sb, x, cb * sc, ca * cc + sa * sb * sc, ca * sb * sc - cc * sa, y, -sb, cb * sa, ca * cb, z);
        }

        /// <summary>
        /// Calculates the pose from the position and euler angles ([x,y,z,r,p,w] vector)
        ///  The result is the same as calling: H = transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)
        /// </summary>
        /// <param name="xyzwpr"></param>
        /// <returns>Homogeneous matrix (4x4)</returns>
        public static Matrix FromXYZRPW(double[] xyzwpr)
        {
            if (xyzwpr.Length < 6)
            {
                return null;
            }
            return FromXYZRPW(xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]);
        }

        /// <summary>
        /// Calculates the pose from the position and euler angles ([x,y,z,rx,ry,rz] array)
        /// The result is the same as calling: H = transl(x,y,z)*rotx(rx*pi/180)*roty(ry*pi/180)*rotz(rz*pi/180)
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="rx"></param>
        /// <param name="ry"></param>
        /// <param name="rz"></param>
        /// <returns>Homogeneous matrix (4x4)</returns>
        public static Matrix FromTxyzRxyz(double x, double y, double z, double rx, double ry, double rz)
        {
            double a = rx * Math.PI / 180.0;
            double b = ry * Math.PI / 180.0;
            double c = rz * Math.PI / 180.0;
            double crx = Math.Cos(a);
            double srx = Math.Sin(a);
            double cry = Math.Cos(b);
            double sry = Math.Sin(b);
            double crz = Math.Cos(c);
            double srz = Math.Sin(c);
            return new Matrix(cry * crz, -cry * srz, sry, x, crx * srz + crz * srx * sry, crx * crz - srx * sry * srz, -cry * srx, y, srx * srz - crx * crz * sry, crz * srx + crx * sry * srz, crx * cry, z);
        }

        /// <summary>
        /// Calculates the pose from the position and euler angles ([x,y,z,rx,ry,rz] array)
        /// The result is the same as calling: H = transl(x,y,z)*rotx(rx*pi/180)*roty(ry*pi/180)*rotz(rz*pi/180)
        /// </summary>
        /// <returns>Homogeneous matrix (4x4)</returns>
        public static Matrix FromTxyzRxyz(double[] xyzwpr)
        {
            if (xyzwpr.Length < 6)
            {
                return null;
            }
            return FromTxyzRxyz(xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]);
        }




        /// <summary>
        /// 返回姿势的四元数（4x4矩阵）
        /// </summary>
        /// <param name="Ti"></param>
        /// <returns></returns>
        public static double[] ToQuaternion(Matrix Ti)
        {
            double[] q = new double[4];
            double a = (Ti[0, 0]);
            double b = (Ti[1, 1]);
            double c = (Ti[2, 2]);
            double sign2 = 1.0;
            double sign3 = 1.0;
            double sign4 = 1.0;
            if ((Ti[2, 1] - Ti[1, 2]) < 0)
            {
                sign2 = -1;
            }
            if ((Ti[0, 2] - Ti[2, 0]) < 0)
            {
                sign3 = -1;
            }
            if ((Ti[1, 0] - Ti[0, 1]) < 0)
            {
                sign4 = -1;
            }
            q[0] = 0.5 * Math.Sqrt(Math.Max(a + b + c + 1, 0));
            q[1] = 0.5 * sign2 * Math.Sqrt(Math.Max(a - b - c + 1, 0));
            q[2] = 0.5 * sign3 * Math.Sqrt(Math.Max(-a + b - c + 1, 0));
            q[3] = 0.5 * sign4 * Math.Sqrt(Math.Max(-a - b + c + 1, 0));
            return q;
        }

        /// <summary>
        /// 从四元数据返回姿势（4x4矩阵）
        /// </summary>
        /// <param name="qin"></param>
        /// <returns></returns>
        public static Matrix FromQuaternion(double[] qin)
        {
            double qnorm = Math.Sqrt(qin[0] * qin[0] + qin[1] * qin[1] + qin[2] * qin[2] + qin[3] * qin[3]);
            double[] q = new double[4];
            q[0] = qin[0] / qnorm;
            q[1] = qin[1] / qnorm;
            q[2] = qin[2] / qnorm;
            q[3] = qin[3] / qnorm;
            Matrix pose = new Matrix(1 - 2 * q[2] * q[2] - 2 * q[3] * q[3], 2 * q[1] * q[2] - 2 * q[3] * q[0], 2 * q[1] * q[3] + 2 * q[2] * q[0], 0, 2 * q[1] * q[2] + 2 * q[3] * q[0], 1 - 2 * q[1] * q[1] - 2 * q[3] * q[3], 2 * q[2] * q[3] - 2 * q[1] * q[0], 0, 2 * q[1] * q[3] - 2 * q[2] * q[0], 2 * q[2] * q[3] + 2 * q[1] * q[0], 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2], 0);
            return pose;
        }

        /// <summary>
        /// Converts a pose to an ABB target
        /// 将姿势转换为ABB目标
        /// </summary>
        /// <param name="H"></param>
        /// <returns></returns>
        public static double[] ToABB(Matrix H)
        {
            double[] q = ToQuaternion(H);
            double[] xyzq1234 = { H[0, 3], H[1, 3], H[2, 3], q[0], q[1], q[2], q[3] };
            return xyzq1234;
        }

        /// <summary>
        /// 从位置和欧拉角（[x，y，z，r，p，w]矢量）计算姿态
        /// Note: The difference between FromUR and FromXYZWPR is that the first one uses radians for the orientation and the second one uses degres
        /// The result is the same as calling: H = transl(x,y,z)*rotx(rx)*roty(ry)*rotz(rz)
        /// </summary>
        /// <param name="xyzwpr">The position and euler angles array</param>
        /// <returns>Homogeneous matrix (4x4)</returns>
        static public Matrix FromUR(double[] xyzwpr)
        {
            double x = xyzwpr[0];
            double y = xyzwpr[1];
            double z = xyzwpr[2];
            double w = xyzwpr[3];
            double p = xyzwpr[4];
            double r = xyzwpr[5];
            double angle = Math.Sqrt(w * w + p * p + r * r);
            if (angle < 1e-6)
            {
                return Identity4x4();
            }
            double c = Math.Cos(angle);
            double s = Math.Sin(angle);
            double ux = w / angle;
            double uy = p / angle;
            double uz = r / angle;
            return new Matrix(ux * ux + c * (1 - ux * ux), ux * uy * (1 - c) - uz * s, ux * uz * (1 - c) + uy * s, x, ux * uy * (1 - c) + uz * s, uy * uy + (1 - uy * uy) * c, uy * uz * (1 - c) - ux * s, y, ux * uz * (1 - c) - uy * s, uy * uz * (1 - c) + ux * s, uz * uz + (1 - uz * uz) * c, z);
        }



        #endregion

        #region 公共方法



        /// <summary>
        /// 将矩阵进行平移并返回一个新的结果
        /// </summary>
        /// <param name="x">translation along X (mm)</param>
        /// <param name="y">translation along Y (mm)</param>
        /// <param name="z">translation along Z (mm)</param>
        /// <returns></returns>
        public Matrix Translate(double x, double y, double z)
        {
            return this * Transl(x, y, z);
        }

        /// <summary>
        /// 将矩阵进行沿X轴旋转并返回一个结果
        /// </summary>
        /// <param name="rx"></param>
        /// <returns></returns>
        public Matrix RotX(double rx)
        {
            return this * Rotx(rx);
        }

        /// <summary>
        /// 将矩阵进行沿Y轴旋转并返回一个结果
        /// </summary>
        /// <param name="ry"></param>
        /// <returns></returns>
        public Matrix RotY(double ry)
        {
            return this * Roty(ry);
        }
        /// <summary>
        /// 将矩阵进行沿Z轴旋转并返回一个结果
        /// </summary>
        /// <param name="rz"></param>
        /// <returns></returns>
        public Matrix RotZ(double rz)
        {
            return this * Rotz(rz);
        }





        /// <summary>
        /// 计算给定姿势的等效位置和欧拉角（[x，y，z，r，p，w]矢量
        /// Note: transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)
        /// See also: FromXYZRPW()
        /// </summary>
        /// <returns>XYZWPR translation and rotation in mm and degrees</returns>
        public double[] ToXYZRPW()
        {
            double[] xyzwpr = new double[6];
            double x = mat[0, 3];
            double y = mat[1, 3];
            double z = mat[2, 3];
            double w, p, r;
            if (mat[2, 0] > (1.0 - 1e-6))
            {
                p = -Math.PI * 0.5;
                r = 0;
                w = Math.Atan2(-mat[1, 2], mat[1, 1]);
            }
            else if (mat[2, 0] < -1.0 + 1e-6)
            {
                p = 0.5 * Math.PI;
                r = 0;
                w = Math.Atan2(mat[1, 2], mat[1, 1]);
            }
            else
            {
                p = Math.Atan2(-mat[2, 0], Math.Sqrt(mat[0, 0] * mat[0, 0] + mat[1, 0] * mat[1, 0]));
                w = Math.Atan2(mat[1, 0], mat[0, 0]);
                r = Math.Atan2(mat[2, 1], mat[2, 2]);
            }
            xyzwpr[0] = x;
            xyzwpr[1] = y;
            xyzwpr[2] = z;
            xyzwpr[3] = r * 180.0 / Math.PI;
            xyzwpr[4] = p * 180.0 / Math.PI;
            xyzwpr[5] = w * 180.0 / Math.PI;
            return xyzwpr;
        }


        /// <summary>
        /// Calculates the equivalent position and euler angles ([x,y,z,rx,ry,rz] array) of a pose 
        /// 计算姿势的等效位置和欧拉角（[x，y，z，rx，ry，rz]数组
        /// Note: Pose = transl(x,y,z)*rotx(rx*pi/180)*roty(ry*pi/180)*rotz(rz*pi/180)
        /// See also: FromTxyzRxyz()
        /// </summary>
        /// <returns>XYZWPR translation and rotation in mm and degrees</returns>
        public double[] ToTxyzRxyz()
        {
            double[] xyzwpr = new double[6];
            double x = mat[0, 3];
            double y = mat[1, 3];
            double z = mat[2, 3];
            double rx1 = 0;
            double ry1 = 0;
            double rz1 = 0;


            double a = mat[0, 0];
            double b = mat[0, 1];
            double c = mat[0, 2];
            double d = mat[1, 2];
            double e = mat[2, 2];

            if (c == 1)
            {
                ry1 = 0.5 * Math.PI;
                rx1 = 0;
                rz1 = Math.Atan2(mat[1, 0], mat[1, 1]);
            }
            else if (c == -1)
            {
                ry1 = -Math.PI / 2;
                rx1 = 0;
                rz1 = Math.Atan2(mat[1, 0], mat[1, 1]);
            }
            else
            {
                double sy = c;
                double cy1 = +Math.Sqrt(1 - sy * sy);

                double sx1 = -d / cy1;
                double cx1 = e / cy1;

                double sz1 = -b / cy1;
                double cz1 = a / cy1;

                rx1 = Math.Atan2(sx1, cx1);
                ry1 = Math.Atan2(sy, cy1);
                rz1 = Math.Atan2(sz1, cz1);
            }

            xyzwpr[0] = x;
            xyzwpr[1] = y;
            xyzwpr[2] = z;
            xyzwpr[3] = rx1 * 180.0 / Math.PI;
            xyzwpr[4] = ry1 * 180.0 / Math.PI;
            xyzwpr[5] = rz1 * 180.0 / Math.PI;
            return xyzwpr;
        }


        /// <summary>
        /// 计算通用机器人格式中给定姿势的等效位置和欧拉角([x，y，z，r，p，w]矢量)
        /// Note: The difference between ToUR and ToXYZWPR is that the first one uses radians for the orientation and the second one uses degres
        /// Note: transl(x,y,z)*rotx(rx*pi/180)*roty(ry*pi/180)*rotz(rz*pi/180)
        /// See also: FromXYZRPW()
        /// </summary>
        /// <returns>XYZWPR translation and rotation in mm and radians</returns>
        public double[] ToUR()
        {
            double[] xyzwpr = new double[6];
            double x = mat[0, 3];
            double y = mat[1, 3];
            double z = mat[2, 3];
            double angle = Math.Acos(Math.Min(Math.Max((mat[0, 0] + mat[1, 1] + mat[2, 2] - 1) / 2, -1), 1));
            double rx = mat[2, 1] - mat[1, 2];
            double ry = mat[0, 2] - mat[2, 0];
            double rz = mat[1, 0] - mat[0, 1];
            if (angle == 0)
            {
                rx = 0;
                ry = 0;
                rz = 0;
            }
            else
            {
                rx = rx * angle / (2 * Math.Sin(angle));
                ry = ry * angle / (2 * Math.Sin(angle));
                rz = rz * angle / (2 * Math.Sin(angle));
            }
            xyzwpr[0] = x;
            xyzwpr[1] = y;
            xyzwpr[2] = z;
            xyzwpr[3] = rx;
            xyzwpr[4] = ry;
            xyzwpr[5] = rz;
            return xyzwpr;
        }



        /// <summary>
        /// 将矩阵进行转置后，并转化为double数组
        /// </summary>
        /// <returns>one-dimensional array</returns>
        public double[] ToDoubles()
        {
            int cnt = 0;
            double[] array = new double[rows * cols];
            for (int j = 0; j < cols; j++)
            {
                for (int i = 0; i < rows; i++)
                {
                    array[cnt] = mat[i, j];
                    cnt = cnt + 1;
                }
            }
            return array;
        }

        /// <summary>
        /// 检查横列是否相等
        /// </summary>
        public Boolean IsSquare()
        {
            return (rows == cols);
        }

        /// <summary>
        /// 检查是否横列4*4
        /// </summary>
        /// <returns></returns>
        public Boolean Is4x4()
        {
            if (cols != 4 || rows != 4)
            {
                return false;
            }
            return true;
        }

        /// <summary>
        /// 检查矩阵是否为4*4的齐次矩阵
        /// </summary>
        public Boolean IsHomogeneous()
        {
            if (!Is4x4())
            {
                return false;
            }
            return true;
        }

        /// <summary>
        /// 获取齐次矩阵的逆矩阵
        /// </summary>
        /// <returns>Homogeneous matrix (4x4)</returns>
        /// <exception cref="MatException"></exception>
        public Matrix Inverse()
        {
            if (!IsHomogeneous())
            {
                throw new MatException("Can't invert a non-homogeneous matrix");
            }
            double[] xyz = this.Position();
            Matrix mat_xyz = new Matrix(xyz[0], xyz[1], xyz[2]);
            Matrix hinv = this.Duplicate();
            hinv.SetPosition(0, 0, 0);
            hinv = hinv.Transpose();
            Matrix new_pos = Rotate(hinv, mat_xyz);
            hinv[0, 3] = -new_pos[0, 0];
            hinv[1, 3] = -new_pos[1, 0];
            hinv[2, 3] = -new_pos[2, 0];
            return hinv;
        }

        /// <summary>
        /// Rotate a vector given a matrix (rotation matrix or homogeneous matrix)
        /// </summary>
        /// <param name="pose">4x4 homogeneous matrix or 3x3 rotation matrix</param>
        /// <param name="vector">4x1 or 3x1 vector</param>
        /// <returns></returns>
        /// <exception cref="MatException"></exception>
        public static Matrix Rotate(Matrix pose, Matrix vector)
        {
            if (pose.cols < 3 || pose.rows < 3 || vector.rows < 3)
            {
                throw new MatException("Invalid matrix size");
            }
            Matrix pose3x3 = pose.Duplicate();
            Matrix vector3 = vector.Duplicate();
            pose3x3.rows = 3;
            pose3x3.cols = 3;
            vector3.rows = 3;
            return pose3x3 * vector3;
        }

        /// <summary>
        /// 返回齐次矩阵的位置
        /// </summary>
        /// <returns>XYZ position</returns>
        public double[] Position()
        {
            if (!Is4x4())
            {
                return null;
            }
            double[] xyz = new double[3];
            xyz[0] = mat[0, 3]; xyz[1] = mat[1, 3]; xyz[2] = mat[2, 3];
            return xyz;
        }

        /// <summary>
        /// 重新设置其次矩阵的位置
        /// </summary>
        /// <param name="xyz">XYZ position</param>
        public void SetPosition(double[] xyz)
        {
            if (!Is4x4() || xyz.Length < 3)
            {
                return;
            }
            mat[0, 3] = xyz[0]; mat[1, 3] = xyz[1]; mat[2, 3] = xyz[2];
        }

        /// <summary>
        /// 重新设置其次矩阵的位置
        /// </summary>
        /// <param name="x">X position</param>
        /// <param name="y">Y position</param>
        /// <param name="z">Z position</param>
        public void SetPosition(double x, double y, double z)
        {
            if (!Is4x4())
            {
                return;
            }
            mat[0, 3] = x; mat[1, 3] = y; mat[2, 3] = z;
        }

        /// <summary>
        /// 获取或设置矩阵的数据
        /// </summary>
        /// <param name="iRow"></param>
        /// <param name="iCol"></param>
        /// <returns></returns>
        /// <exception cref="IndexOutOfRangeException"></exception>
        public double this[int iRow, int iCol]      // Access this matrix as a 2D array
        {
            get { return mat[iRow, iCol]; }
            set { mat[iRow, iCol] = value; }
        }

        /// <summary>
        /// 获取矩阵的某一列
        /// </summary>
        /// <param name="k"></param>
        /// <returns></returns>
        public Matrix GetCol(int k)
        {
            Matrix m = new Matrix(rows, 1);
            for (int i = 0; i < rows; i++) m[i, 0] = mat[i, k];
            return m;
        }

        /// <summary>
        /// 设置矩阵的某一列
        /// </summary>
        /// <param name="v"></param>
        /// <param name="k"></param>
        public void SetCol(Matrix v, int k)
        {
            for (int i = 0; i < rows; i++) mat[i, k] = v[i, 0];
        }


        /// <summary>
        /// 复制一个矩阵
        /// </summary>
        /// <returns></returns>
        public Matrix Duplicate()
        {
            Matrix matrix = new Matrix(rows, cols);
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    matrix[i, j] = mat[i, j];
            return matrix;
        }

        /// <summary>
        /// 返回矩阵的字符串表示形式
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            string s = "";
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++) s += String.Format("{0,5:0.00}", mat[i, j]) + " ";
                s += "\r\n";
            }
            return s;
        }

        /// <summary>
        /// 将矩阵进行转置，并返回转置后结果
        /// </summary>
        /// <returns></returns>
        public Matrix Transpose()
        {
            return Transpose(this);
        }


        #endregion

    }
}
