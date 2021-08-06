using UnityEngine;
using System.Collections.Generic;

class CurveFitter {
  /// <summary>
  /// Fit curve to a given list of sample points.
  /// </summary>
  /// <param name="samplePts">List of sample points to fit curve to.</param>
  /// <param name="error">The error threshold.</param>
  /// <returns>A list of bezier curves fitting the sample points.</returns>
  public static List<BezierCurve> FitCurve(List<Vector3> samplePts, float error) {
    Vector3 th1 = ComputeLeftTangent(samplePts);
    Vector3 th2 = ComputeRightTangent(samplePts);
    return FitCubic(samplePts, 0, samplePts.Count - 1, th1, th2, error);
  }

  private static Vector3 ComputeLeftTangent(List<Vector3> samplePts) {
    Vector3 th = samplePts[1] - samplePts[0];
    return th.normalized;
  }

  private static Vector3 ComputeRightTangent(List<Vector3> samplePts) {
    int end = samplePts.Count - 1;
    Vector3 th = samplePts[end - 1] - samplePts[end];
    return th.normalized;
  }

  private static Vector3 ComputeCenterTangent(List<Vector3> samplePts, int center) {
    Vector3 v1 = samplePts[center - 1] - samplePts[center];
    Vector3 v2 = samplePts[center] - samplePts[center + 1];
    return ((v1 + v2) / 2f).normalized;
  }

  /// <summary>
  /// Fit curve to a subset of sample points.
  /// </summary>
  /// <param name="samplePts"></param>
  /// <param name="begin"></param>
  /// <param name="end"></param>
  /// <param name="th1"></param>
  /// <param name="th2"></param>
  /// <param name="error"></param>
  /// <return>Bezier control points</return>
  private static List<BezierCurve> FitCubic(List<Vector3> samplePts, int begin, int end, Vector3 th1, Vector3 th2, float error) {
    int nPts = end - begin + 1;

    List<BezierCurve> curves = new List<BezierCurve>();
    BezierCurve bezierCurve = new BezierCurve(BezierCurve.Degree.Cubic);

    // Use heuristic if region only has two points in it.
    if (nPts == 2) {
      float dist = Vector3.Distance(samplePts[begin], samplePts[end]) / 3f;
      bezierCurve.SetControlPoint(0, samplePts[begin]);
      bezierCurve.SetControlPoint(1, samplePts[begin] + th1 * dist);
      bezierCurve.SetControlPoint(2, samplePts[end] + th2 * dist);
      bezierCurve.SetControlPoint(3, samplePts[end]);
      curves.Add(bezierCurve);
      return curves;
    }

    // Parameterize points and attempt to fit curve.
    List<float> u = ChordLengthParameterize(samplePts, begin, end);
    bezierCurve = GenerateBezierControlPts(samplePts, begin, end, u, th1, th2);

    float maxError;
    int splitPoint; // Point to split set at

    (maxError, splitPoint) = ComputeMaxError(samplePts, begin, end, bezierCurve, u);
    if (maxError < error) {
      curves.Add(bezierCurve);

      return curves;
    }

    List<float> uPrime; // Improved parameter values
    float iterationError = error * 4f;
    int maxIterations = 4;

    // If error not too large try reparameterization and iteration.
    if (maxError < iterationError) {
      for (int i = 0; i < maxIterations; i++) {
        uPrime = Reparameterize(samplePts, begin, end, u, bezierCurve);
        bezierCurve = GenerateBezierControlPts(samplePts, begin, end, uPrime, th1, th2);

        (maxError, splitPoint) = ComputeMaxError(samplePts, begin, end, bezierCurve, uPrime);

        if (maxError < error) {
          curves.Add(bezierCurve);
          return curves;
        }

        u = uPrime;
      }
    }

    // Fitting failed, so split at max error point and fit recursively.
    Vector3 thCenter = ComputeCenterTangent(samplePts, splitPoint);
    curves.AddRange(FitCubic(samplePts, begin, splitPoint, th1, thCenter, error));
    thCenter = -thCenter;
    curves.AddRange(FitCubic(samplePts, splitPoint, end,thCenter, th2, error));

    return curves;
  }

  private static List<float> ChordLengthParameterize(List<Vector3> samplePts, int begin, int end) {
    List<float> u = new List<float>(new float[end - begin + 1]);
    u[0] = 0f;

    for (int i = begin + 1; i <= end; i++) {
      u[i - begin] = u[i - begin - 1] + Vector3.Distance(samplePts[i], samplePts[i - 1]);
    }

    for (int i = begin + 1; i <= end; i++) {
      u[i - begin] = u[i - begin] / u[end - begin];
    }

    return u;
  }

  /// <summary>
  /// Generate control points fitting list of sample points.
  /// </summary>
  /// <param name="samplePts"></param>
  /// <param name="begin"></param>
  /// <param name="end"></param>
  /// <param name="uPrime"></param>
  /// <param name="th1"></param>
  /// <param name="th2"></param>
  /// <returns></returns>
  private static BezierCurve GenerateBezierControlPts(
    List<Vector3> samplePts,
    int begin,
    int end,
    List<float> uPrime,
    Vector3 th1,
    Vector3 th2
  ) {
    int numPoints = end - begin + 1;
    Vector2[,] A = new Vector2[numPoints, 2];

    for (int i = 0; i < numPoints; i++) {
      A[i,0] = th1 * BezierCurve.CubicB1(uPrime[i]);
      A[i,1] = th2 * BezierCurve.CubicB2(uPrime[i]);
    }

    float[,] C = new float[2, 2];
    Vector2 X = Vector2.zero;
    Vector3 beginPt = samplePts[begin];
    Vector3 endPt = samplePts[end];

    for (int i = 0; i < numPoints; i++) {
      C[0,0] += Vector2.Dot(A[i,0], A[i,0]);
      C[0,1] += Vector2.Dot(A[i,0], A[i,1]);
      C[1,0] = C[0,1];
      C[1,1] += Vector2.Dot(A[i,1], A[i,1]);

      Vector3 tmp = samplePts[begin + i]
        - (beginPt * BezierCurve.CubicB0(uPrime[i])
          + beginPt * BezierCurve.CubicB1(uPrime[i])
          + endPt * BezierCurve.CubicB2(uPrime[i])
          + endPt * BezierCurve.CubicB3(uPrime[i]));

      X[0] += Vector2.Dot(A[i,0], tmp);
      X[1] += Vector2.Dot(A[i,1], tmp);
    }

    float det_C0_C1 = C[0,0] * C[1,1] - C[1,0] * C[0,1];
    float det_C0_X = C[0,0] * X[1] - C[1,0] * X[0];
    float det_X_C1 = X[0] * C[1,1] - X[1] * C[0,1];

    float alphaL = (det_C0_C1 == 0) ? 0f : det_X_C1 / det_C0_C1;
    float alphaR = (det_C0_C1 == 0) ? 0f : det_C0_X / det_C0_C1;

    float segmentLength = Vector3.Distance(beginPt, endPt);
    float epsilon = 1.0e-6f * segmentLength;

    BezierCurve bezierCurve = new BezierCurve(BezierCurve.Degree.Cubic);
    bezierCurve.SetControlPoint(0, beginPt);
    bezierCurve.SetControlPoint(3, endPt);

    if (alphaL < epsilon || alphaR < epsilon) {
      float dist = segmentLength / 3.0f;
      bezierCurve.SetControlPoint(1, beginPt + th1 * dist);
      bezierCurve.SetControlPoint(2, endPt + th2 * dist);

      return bezierCurve;
    }

    bezierCurve.SetControlPoint(1, beginPt + th1 * alphaL);
    bezierCurve.SetControlPoint(2, endPt + th2 * alphaR);

    return bezierCurve;
  }

  private static List<float> Reparameterize(List<Vector3> samplePts, int begin, int end, List<float> u, BezierCurve bezierCurve) {
    int numPts = end - begin + 1;
    List<float> uPrime = new List<float>(new float[numPts]);
    for (int i = begin; i <= end; i++) {
      uPrime[i - begin] = NewtonRaphsonRootFind(bezierCurve, samplePts[i], u[i - begin]);
    }
    return uPrime;
  }

  /// <summary>
  /// Use Newton-Raphson iteration to find better root.
  /// </summary>
  /// <param name="controlPts"></param>
  /// <param name="p">Sample point</param>
  /// <param name="u">Parameter value for 'p'</param>
  /// <returns></returns>
  private static float NewtonRaphsonRootFind(BezierCurve bezierCurve, Vector3 p, float u) {
    // Q(u)
    List<Vector3> controlPts = bezierCurve.GetControlPoints();

    Vector3 Q_u = EvaluateBezier(3, controlPts, u);

    // Generate control vertices for Q'
    List<Vector3> Q1 = new List<Vector3>(new Vector3[3]);
    for (int i = 0; i < 3; i++) {
      Q1[i] = (controlPts[i + 1] - controlPts[i]) * 3f;
    }

    List<Vector3> Q2 = new List<Vector3>(new Vector3[2]);
    // Generate control vertices for Q''
    for (int i = 0; i < 2; i++) {
      Q2[i] = (Q1[i + 1] - Q1[i]) * 2f;
    }

    // Compute Q'(u) and Q''(u)
    Vector3 Q1_u = EvaluateBezier(2, Q1, u);
    Vector3 Q2_u = EvaluateBezier(1, Q2, u);

    float numerator = (Q_u.x - p.x) * Q1_u.x
      + (Q_u.y - p.y) * Q1_u.y
      + (Q_u.z - p.z) * Q1_u.z;

    float denominator = Q1_u.x * Q1_u.x
      + Q1_u.y * Q1_u.y
      + Q1_u.z * Q1_u.z
      + (Q_u.x - p.x) * Q2_u.x
      + (Q_u.y - p.y) * Q2_u.y
      + (Q_u.z - p.z) * Q2_u.z;

    if (denominator == 0f) {
      return u;
    }
    return u - (numerator / denominator);
  }

  private static Vector3 EvaluateBezier(int degree, List<Vector3> controlPts, float t) {
    for (int i = 1; i <= degree; i++) {
      for (int j = 0; j <= degree - i; j ++) {
        controlPts[j] = (1f - t) * controlPts[j] + t * controlPts[j + 1];
      }
    }
    return controlPts[0];
  }

  private static (float, int) ComputeMaxError(List<Vector3> samplePts, int begin, int end, BezierCurve bezierCurve, List<float> u) {
    int splitPoint = (end - begin + 1) / 2;
    float maxError = 0f;
    Vector3 p; // Point on curve
    Vector3 v; // Vector from point to curve.
    float curError;

    for (int i = begin + 1; i < end; i++) {
      List<Vector3> controlPts = bezierCurve.GetControlPoints();
      p = EvaluateBezier(3, controlPts, u[i - begin]);

      v = p - samplePts[i];
      curError = v.sqrMagnitude;

      if (curError >= maxError) {
        maxError = curError;
        splitPoint = i;
      }
    }

    return (maxError, splitPoint);
  }
}