# Curve Fitter

## Summary

C# implementation of Philip J. Schneider's "Algorithm for Automatically Fitting Digitized Curves" from the book "Graphics Gems" adapted to points in 3D for Unity.

Generates a `List<UnityEngine.Vector3>` of `BezierCurve`s fitted to the given list of sample points.

The original C code is available here: https://github.com/erich666/GraphicsGems

## Requirements

Unity is required for the `UnityEngine.Vector3` class, though for non-Unity use it should be trivial to swap it out for `System.Numerics.Vector3`.


## Usage

```C#
const float ERROR_THRESHOLD = 0.1f;
List<Vector3> samplePoints = new List<Vector3>();

samplePoints.Add(new Vector3(0, 0, 1));
samplePoints.Add(new Vector3(1, 2, 1));
samplePoints.Add(new Vector3(2, 3, 1));
samplePoints.Add(new Vector3(3, 4, 5));
// Add sample points as needed

List<BezierCurve> bezierCurves = CurveFitter.FitCurve(samplePoints, ERROR_THRESHOLD);

// Draw each curve
```

## Examples

![1](examples/1.png)
![2](examples/2.png)