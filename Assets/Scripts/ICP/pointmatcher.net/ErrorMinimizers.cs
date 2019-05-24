using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using MathNet.Numerics.LinearAlgebra.Storage;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace pointmatcher.net
{
    public class PointToPlaneErrorMinimizer : IErrorMinimizer
    {
        public EuclideanTransform SolveForTransform(ErrorElements mPts)
        {
            if (!mPts.reference.contiansNormals)
            {
                throw new ArgumentException("Reference points must have computed normals. Use appropriate input filter.");
            }

            var readingPts = mPts.reading.points;
            var refPts = mPts.reference.points;

            // Compute cross product of cross = cross(reading X normalRef)
            // wF = [ weights*cross   ]
            //      [ weights*normals ]
            //
            // F  = [ cross   ]
            //      [ normals ]
            var wF = new DenseMatrix(6, readingPts.Length);
            var F = new DenseMatrix(6, readingPts.Length);
            for (int i = 0; i < readingPts.Length; i++)
            {
                var cross = System.Numerics.Vector3.Cross(readingPts[i].point, refPts[i].normal);
                var wCross = mPts.weights[i] * cross;
                var wNormal = mPts.weights[i] * refPts[i].normal;
                wF.At(0, i, wCross.X);
                wF.At(1, i, wCross.Y);
                wF.At(2, i, wCross.Z);
                wF.At(3, i, wNormal.X);
                wF.At(4, i, wNormal.Y);
                wF.At(5, i, wNormal.Z);
                F.At(0, i, cross.X);
                F.At(1, i, cross.Y);
                F.At(2, i, cross.Z);
                F.At(3, i, refPts[i].normal.X);
                F.At(4, i, refPts[i].normal.Y);
                F.At(5, i, refPts[i].normal.Z);
            }

            // Unadjust covariance A = wF * F'
            var A = wF.TransposeAndMultiply(F);

            // dot product of dot = dot(deltas, normals)
            var dotProd = new DenseMatrix(1, mPts.reading.points.Length);
            for (int i = 0; i < readingPts.Length; i++)
            {
                var delta = readingPts[i].point - refPts[i].point;
                dotProd.At(0, i, System.Numerics.Vector3.Dot(delta, refPts[i].normal));
            }

            // b = -(wF' * dot)
            var b = -(wF.TransposeAndMultiply(dotProd));

            // Cholesky decomposition
            UnityEngine.Debug.Log("A: " + A.ToMatrixString());
            //UnityEngine.Debug.Log("b: " + b.ToString());
            var x = A.Cholesky().Solve(b);

            EuclideanTransform transform;
            System.Numerics.Vector3 axis = new System.Numerics.Vector3(x.At(0, 0), x.At(1, 0), x.At(2, 0));
            float len = axis.Length();
            transform.rotation = System.Numerics.Quaternion.Normalize(System.Numerics.Quaternion.CreateFromAxisAngle(axis / len, len));
            transform.translation = new System.Numerics.Vector3(x.At(3, 0), x.At(4, 0), x.At(5, 0));

            return transform;
        }
    }

    public class ErrorElements
    {
        public DataPoints reading;
        public DataPoints reference;
        public float[] weights;
    }

    public static class ErrorMinimizerHelper
    {
        public static EuclideanTransform Compute(
            DataPoints filteredReading,
            DataPoints filteredReference,
            Matrix<float> outlierWeights,
            Matches matches,
            IErrorMinimizer minimizer)
        {
            System.Diagnostics.Debug.Assert(matches.Ids.RowCount > 0);
            ErrorElements mPts = ErrorMinimizerHelper.GetMatchedPoints(filteredReading, filteredReference, matches, outlierWeights);
            return minimizer.SolveForTransform(mPts);
        }

        private static ErrorElements GetMatchedPoints(
		    DataPoints requestedPts,
		    DataPoints sourcePts,
		    Matches matches, 
		    Matrix<float> outlierWeights)
        {
            System.Diagnostics.Debug.Assert(matches.Ids.RowCount > 0);
            System.Diagnostics.Debug.Assert(matches.Ids.ColumnCount > 0);
            System.Diagnostics.Debug.Assert(matches.Ids.ColumnCount == requestedPts.points.Length); //nbpts
            System.Diagnostics.Debug.Assert(outlierWeights.RowCount == matches.Ids.RowCount);  // knn
	
	        int knn = outlierWeights.RowCount;
	        
            int maxPointsCount = matches.Ids.RowCount * matches.Ids.ColumnCount;

            var keptPoints = new List<DataPoint>(maxPointsCount);
            var matchedPoints = new List<DataPoint>(maxPointsCount);
	        List<float> keptWeights = new List<float>(maxPointsCount);

	        //float weightedPointUsedRatio = 0;
	        for(int k = 0; k < knn; k++) // knn
	        {
		        for (int i = 0; i < requestedPts.points.Length; ++i) //nb pts
		        {
                    float weight = outlierWeights.At(k,i);
			        if (weight != 0.0f)
			        {
				        keptPoints.Add(requestedPts.points[i]);

                        float matchIdx = matches.Ids.At(k, i); //int
                        matchedPoints.Add(sourcePts.points[(int)matchIdx]); //-(int)
				        keptWeights.Add(weight);
				        //weightedPointUsedRatio += weight;
			        }
		        }
	        }

            var result = new ErrorElements
            {
                reading = new DataPoints
                {
                    points = keptPoints.ToArray(),
                    contiansNormals = requestedPts.contiansNormals,
                },
                reference = new DataPoints
                {
                    points = matchedPoints.ToArray(),
                    contiansNormals = sourcePts.contiansNormals,
                },
                weights = keptWeights.ToArray(),
            };
            return result;
        }
    }
}
