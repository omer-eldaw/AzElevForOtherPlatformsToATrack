using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace Az_ElevForOtherPlatformsToATrack
{
    public class AzElevForOtherPlatformsToATrack
    {

        private const double EarthRadius = 6371000; // meters;
        private const double Pi = Math.PI;

        public Vector3 mainPlatformPositionLatLongAlt;
        public Vector3 mainPlatformToTargetAzElevRange;
        public Vector3 mainPlatformOrientation;
        public Vector3 otherPlatformPositionLatLongAlt;
        public Vector3 otherPlatformOrientation;

        public double[] CalculateAzimuthElevationRangeFromOtherPlatformsToTarget(Vector3 mainPlatformPositionLatLongAlt, Vector3 mainPlatformToTargetAzElevRange, Vector3 mainPlatformOrientation, Vector3 otherPlatformPositionLatLongAlt, Vector3 otherPlatformOrientation)
        {
            double[] azimuthElevationRange = new double[3];

            // Calculate the direction vector from the platform to the target
            Vector3 vectorFromOtherPlatformToMainPlatform = CalculateDirectionVectorFromLatLong(mainPlatformPositionLatLongAlt, otherPlatformPositionLatLongAlt);
            Vector3 vectorFromMainPlatformToTarget = CalculateDirectionFromMainPlatformToTarget(mainPlatformToTargetAzElevRange.X, mainPlatformToTargetAzElevRange.Y, mainPlatformToTargetAzElevRange.Z);
            Vector3 vectorFromOtherPlatformToTarget = Vector3.Add(vectorFromOtherPlatformToMainPlatform, vectorFromMainPlatformToTarget);

            // Calculate the rotated unit vectors            
            Matrix4x4 rotatedAxes = CalculateRotatedAxes(otherPlatformOrientation.X, otherPlatformOrientation.Y, otherPlatformOrientation.Z);

            Vector3 rotatedX = new Vector3(rotatedAxes.M11, rotatedAxes.M12, rotatedAxes.M13);
            Vector3 rotatedY = new Vector3(rotatedAxes.M21, rotatedAxes.M22, rotatedAxes.M23);
            Vector3 rotatedZ = new Vector3(rotatedAxes.M31, rotatedAxes.M32, rotatedAxes.M33);

            // Resolve the target vector onto the XY plane of the OTHER platform
            float numOfXYVectors = Vector3.Dot(vectorFromOtherPlatformToTarget, rotatedY);
            Vector3 multipleOfXYVectors = Vector3.Multiply(numOfXYVectors, rotatedY);
            Vector3 resolvedXYTargetVector = vectorFromOtherPlatformToTarget - multipleOfXYVectors;

            // Calculate the azimuth and elevation
            azimuthElevationRange[0] = CalculateAzimuth(rotatedX, resolvedXYTargetVector, rotatedY, vectorFromOtherPlatformToTarget);
            azimuthElevationRange[1] = CalculateElevation(vectorFromOtherPlatformToTarget, resolvedXYTargetVector, rotatedZ);
            azimuthElevationRange[2] = vectorFromOtherPlatformToTarget.Length();

            return azimuthElevationRange;
        }


        public static void Main(string[] args)
        {
            AzElevForOtherPlatformsToATrack azElevForOtherPlatformsToATrack = new AzElevForOtherPlatformsToATrack();

            Vector3 mainPlatformPositionLatLongAlt = new Vector3(37.7749f, -122.4194f, 0);
            Vector3 mainPlatformToTargetAzElevRange = new Vector3(30, 20, 500);
            Vector3 mainPlatformOrientation = new Vector3(0, 0, 0);
            Vector3 otherPlatformPositionLatLongAlt = new Vector3(37.7749f, -122.4194f, 0);
            Vector3 otherPlatformOrientation = new Vector3(0, 0, 0);

            double[] azimuthElevationRange = azElevForOtherPlatformsToATrack.CalculateAzimuthElevationRangeFromOtherPlatformsToTarget(mainPlatformPositionLatLongAlt, mainPlatformToTargetAzElevRange,
                mainPlatformOrientation, otherPlatformPositionLatLongAlt, otherPlatformOrientation);

            Console.WriteLine("Azimuth From Other Platform: " + azimuthElevationRange[0] + " Elevation: " + azimuthElevationRange[1] + " Range: " + azimuthElevationRange[2]);
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        private static Vector3 CalculateDirectionVectorFromLatLong(Vector3 positionLatLongAlt, Vector3 targetPositionLatLongAlt)
        {
            Vector3 directionVector = new Vector3();

            double lat1 = positionLatLongAlt.X * Pi / 180;
            double lon1 = positionLatLongAlt.Y * Pi / 180;
            double alt1 = positionLatLongAlt.Z;

            double lat2 = targetPositionLatLongAlt.X * Pi / 180;
            double lon2 = targetPositionLatLongAlt.Y * Pi / 180;
            double alt2 = targetPositionLatLongAlt.Z;

            double deltaLon = lon2 - lon1;
            double deltaAlt = alt2 - alt1;

            double distance = Math.Acos((Math.Sin(lat1) * Math.Sin(lat2)) + (Math.Cos(lat1) * Math.Cos(lat2) * Math.Cos(deltaLon))) * EarthRadius;
            double bearing = Math.Atan2(Math.Sin(deltaLon) * Math.Cos(lat2), (Math.Cos(lat1) * Math.Sin(lat2)) - (Math.Sin(lat1) * Math.Cos(lat2) * Math.Cos(deltaLon)));
            double elevation = Math.Atan(deltaAlt / distance);

            directionVector.X = (float)Math.Cos(bearing);
            directionVector.Y = (float)Math.Sin(bearing);
            directionVector.Z = (float)Math.Tan(elevation);

            return directionVector;
        }

        // Calculates the direction vector, given the Azimuth, Elevation and Range of the  Main Platform to the Target
        private static Vector3 CalculateDirectionFromMainPlatformToTarget(float azimuthFromMainPlatformToTarget, float elevationFromMainPlatformToTarget, float rangeFromMainPlatformToTarget)
        {
            Vector3 DirectionVector = new Vector3();

            // Calculates X, Y, Z components of the Direction Vector
            double directionVectorX = rangeFromMainPlatformToTarget * Math.Cos(elevationFromMainPlatformToTarget) * Math.Sin(azimuthFromMainPlatformToTarget);
            double directionVectorY = rangeFromMainPlatformToTarget * Math.Cos(elevationFromMainPlatformToTarget) * Math.Cos(azimuthFromMainPlatformToTarget);
            double directionVectorZ = rangeFromMainPlatformToTarget * Math.Sin(elevationFromMainPlatformToTarget);

            DirectionVector.X = (float)directionVectorX;
            DirectionVector.Y = (float)directionVectorY;
            DirectionVector.Z = (float)directionVectorZ;

            return DirectionVector;
        }


        // Calculates the rotated unit X, Y & Z vectors given Yaw, Pitch and Roll in RADIANS of the platform.
        private static Matrix4x4 CalculateRotatedAxes(float yaw, float pitch, float roll) 
        {
            // Define the original axes
            Vector3 originalX = new Vector3(1, 0, 0);
            Vector3 originalY = new Vector3(0, 1, 0);
            Vector3 originalZ = new Vector3(0, 0, 1);

            // Calculate the rotation matrix
            Matrix4x4 rotationMatrix = new Matrix4x4(
                (float)(Math.Cos(pitch) * Math.Cos(roll)), (float)(Math.Cos(pitch) * Math.Sin(roll)), (float)(-Math.Sin(pitch)), 0,
                (float)(Math.Sin(yaw) * Math.Sin(pitch) * Math.Cos(roll) - Math.Cos(yaw) * Math.Sin(roll)), (float)(Math.Sin(yaw) * Math.Sin(pitch) * Math.Sin(roll) + Math.Cos(yaw) * Math.Cos(roll)), (float)(Math.Sin(yaw) * Math.Cos(pitch)), 0,
                (float)(Math.Cos(yaw) * Math.Sin(pitch) * Math.Cos(roll) + Math.Sin(yaw) * Math.Sin(roll)), (float)(Math.Cos(yaw) * Math.Sin(pitch) * Math.Sin(roll) - Math.Sin(yaw) * Math.Cos(roll)), (float)(Math.Cos(yaw) * Math.Cos(pitch)), 0,
                0, 0, 0, 1
            );

            // Rotate the original axes

            Vector3 rotatedX = Vector3.Transform(originalX, rotationMatrix);
            Vector3 rotatedY = Vector3.Transform(originalY, rotationMatrix);
            Vector3 rotatedZ = Vector3.Transform(originalZ, rotationMatrix);

            Matrix4x4 rotatedAxes = new Matrix4x4(
                rotatedX.X, rotatedX.Y, rotatedX.Z, 0,
                rotatedY.X, rotatedY.Y, rotatedY.Z, 0,
                rotatedZ.X, rotatedZ.Y, rotatedZ.Z, 0,
                0, 0, 0, 1
                );

            return rotatedAxes;
        }

        private static double CalculateAzimuth(Vector3 rotatedX, Vector3 resolvedXYTargetVector, Vector3 rotatedY, Vector3 targetVector)
        {
            // Calculate the azimuth
            double azimuth = Math.Acos(Vector3.Dot(rotatedX, resolvedXYTargetVector) / (rotatedX.Length() * resolvedXYTargetVector.Length()));

            // Check if the azimuth is to the left or right of the XY plane of the platform
            if (Vector3.Dot(rotatedY, targetVector) < 0)
            {
                azimuth = - azimuth;
            }

            return azimuth;
        }

        private static double CalculateElevation(Vector3 targetVector, Vector3 resolvedXYTargetVector, Vector3 rotatedZ)
        {
            // Calculate the elevation
            double elevation = Math.Acos(Vector3.Dot(targetVector, resolvedXYTargetVector) / (targetVector.Length() * resolvedXYTargetVector.Length()));

            // Check if the elevation is above or below the XY plane of the platform
            if (Vector3.Dot(rotatedZ, targetVector) < 0)
            {
                elevation = -elevation;
            }

            return elevation;
        }
    }
}
