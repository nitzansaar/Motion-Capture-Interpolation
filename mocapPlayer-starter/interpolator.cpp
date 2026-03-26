#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  double x = angles[0] * M_PI / 180.0;
  double y = angles[1] * M_PI / 180.0;
  double z = angles[2] * M_PI / 180.0;

  double cx = cos(x), sx = sin(x);
  double cy = cos(y), sy = sin(y);
  double cz = cos(z), sz = sin(z);

  // R = Rz * Ry * Rx
  R[0] = cy*cz;               R[1] = -cx*sz + sx*sy*cz;  R[2] = sx*sz + cx*sy*cz;
  R[3] = cy*sz;               R[4] =  cx*cz + sx*sy*sz;  R[5] = -sx*cz + cx*sy*sz;
  R[6] = -sy;                 R[7] =  cy*sx;              R[8] = cy*cx;
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames();

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe  = startKeyframe + N + 1;
    int prevKeyframe = startKeyframe - (N + 1);
    int nextKeyframe = endKeyframe   + (N + 1);

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture   = pInputMotion->GetPosture(endKeyframe);

    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe,   *endPosture);

    // Compute Bezier control postures p1, p2
    Posture p1, p2;

    // Root position control points
    vector rn  = startPosture->root_pos;
    vector rn1 = endPosture->root_pos;
    if (prevKeyframe >= 0)
      p1.root_pos = rn + (rn1 - pInputMotion->GetPosture(prevKeyframe)->root_pos) / 3.0;
    else
      p1.root_pos = rn + (rn1 - rn) / 3.0;
    if (nextKeyframe < inputLength)
      p2.root_pos = rn1 - (pInputMotion->GetPosture(nextKeyframe)->root_pos - rn) / 3.0;
    else
      p2.root_pos = rn1 - (rn1 - rn) / 3.0;

    // Bone rotation control points
    for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
    {
      vector qn  = startPosture->bone_rotation[bone];
      vector qn1 = endPosture->bone_rotation[bone];
      if (prevKeyframe >= 0)
        p1.bone_rotation[bone] = qn + (qn1 - pInputMotion->GetPosture(prevKeyframe)->bone_rotation[bone]) / 3.0;
      else
        p1.bone_rotation[bone] = qn + (qn1 - qn) / 3.0;
      if (nextKeyframe < inputLength)
        p2.bone_rotation[bone] = qn1 - (pInputMotion->GetPosture(nextKeyframe)->bone_rotation[bone] - qn) / 3.0;
      else
        p2.bone_rotation[bone] = qn1 - (qn1 - qn) / 3.0;
    }

    for (int frame = 1; frame <= N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N + 1);

      interpolatedPosture.root_pos = DeCasteljauEuler(t, startPosture->root_pos, p1.root_pos, p2.root_pos, endPosture->root_pos);

      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t,
          startPosture->bone_rotation[bone], p1.bone_rotation[bone],
          p2.bone_rotation[bone], endPosture->bone_rotation[bone]);

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for (int frame = startKeyframe + 1; frame < inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames();

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture   = pInputMotion->GetPosture(endKeyframe);

    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe,   *endPosture);

    for (int frame = 1; frame <= N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N + 1);

      // interpolate root position linearly
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate each bone rotation via Slerp
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        Quaternion<double> qStart, qEnd;
        Euler2Quaternion(startPosture->bone_rotation[bone].p, qStart);
        Euler2Quaternion(endPosture->bone_rotation[bone].p,   qEnd);

        Quaternion<double> q = Slerp(t, qStart, qEnd);
        Quaternion2Euler(q, interpolatedPosture.bone_rotation[bone].p);
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for (int frame = startKeyframe + 1; frame < inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames();

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe  = startKeyframe + N + 1;
    int prevKeyframe = startKeyframe - (N + 1);
    int nextKeyframe = endKeyframe   + (N + 1);

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture   = pInputMotion->GetPosture(endKeyframe);

    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe,   *endPosture);

    // Precompute control quaternions for all bones
    Quaternion<double> q_n[MAX_BONES_IN_ASF_FILE], q_n1[MAX_BONES_IN_ASF_FILE];
    Quaternion<double> a_n[MAX_BONES_IN_ASF_FILE], b_n1[MAX_BONES_IN_ASF_FILE];

    for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
    {
      Euler2Quaternion(startPosture->bone_rotation[bone].p, q_n[bone]);
      Euler2Quaternion(endPosture->bone_rotation[bone].p,   q_n1[bone]);

      if (prevKeyframe >= 0)
      {
        Quaternion<double> q_nm1;
        Euler2Quaternion(pInputMotion->GetPosture(prevKeyframe)->bone_rotation[bone].p, q_nm1);
        Quaternion<double> d_n = Double(q_nm1, q_n[bone]);
        a_n[bone] = Slerp(1.0/3.0, q_n[bone], d_n);
      }
      else
        a_n[bone] = Slerp(1.0/3.0, q_n[bone], q_n1[bone]);

      if (nextKeyframe < inputLength)
      {
        Quaternion<double> q_n2;
        Euler2Quaternion(pInputMotion->GetPosture(nextKeyframe)->bone_rotation[bone].p, q_n2);
        Quaternion<double> d_n1 = Double(q_n2, q_n1[bone]);
        b_n1[bone] = Slerp(1.0/3.0, q_n1[bone], d_n1);
      }
      else
        b_n1[bone] = Slerp(1.0/3.0, q_n1[bone], q_n[bone]);
    }

    // Root position control points (translational — use linear Bezier)
    vector rn  = startPosture->root_pos;
    vector rn1 = endPosture->root_pos;
    vector rp1, rp2;
    if (prevKeyframe >= 0)
      rp1 = rn + (rn1 - pInputMotion->GetPosture(prevKeyframe)->root_pos) / 3.0;
    else
      rp1 = rn + (rn1 - rn) / 3.0;
    if (nextKeyframe < inputLength)
      rp2 = rn1 - (pInputMotion->GetPosture(nextKeyframe)->root_pos - rn) / 3.0;
    else
      rp2 = rn1 - (rn1 - rn) / 3.0;

    for (int frame = 1; frame <= N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N + 1);

      interpolatedPosture.root_pos = DeCasteljauEuler(t, rn, rp1, rp2, rn1);

      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        Quaternion<double> q = DeCasteljauQuaternion(t, q_n[bone], a_n[bone], b_n1[bone], q_n1[bone]);
        Quaternion2Euler(q, interpolatedPosture.bone_rotation[bone].p);
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for (int frame = startKeyframe + 1; frame < inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q)
{
  double x = angles[0] * M_PI / 180.0;
  double y = angles[1] * M_PI / 180.0;
  double z = angles[2] * M_PI / 180.0;

  Quaternion<double> qx(cos(x/2), sin(x/2), 0, 0);
  Quaternion<double> qy(cos(y/2), 0, sin(y/2), 0);
  Quaternion<double> qz(cos(z/2), 0, 0, sin(z/2));

  q = qz * qy * qx;
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3])
{
  double R[9];
  q.Quaternion2Matrix(R);
  Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  Quaternion<double> qEnd = qEnd_;

  // dot product = cos(theta)
  double dot = qStart.Gets()*qEnd.Gets() + qStart.Getx()*qEnd.Getx()
             + qStart.Gety()*qEnd.Gety() + qStart.Getz()*qEnd.Getz();

  // take the shorter arc
  if (dot < 0)
  {
    qEnd = -1.0 * qEnd;
    dot = -dot;
  }

  // if quaternions are nearly identical, fall back to linear interpolation
  if (dot > 1.0 - 1e-6)
    return (1.0 - t) * qStart + t * qEnd;

  double theta = acos(dot);
  double sinTheta = sin(theta);
  return (sin((1-t)*theta) / sinTheta) * qStart + (sin(t*theta) / sinTheta) * qEnd;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // reflect p over q on the unit sphere: 2*(p·q)*q - p
  double dot = p.Gets()*q.Gets() + p.Getx()*q.Getx() + p.Gety()*q.Gety() + p.Getz()*q.Getz();
  return 2.0*dot * q - p;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  vector p01 = p0 * (1-t) + p1 * t;
  vector p12 = p1 * (1-t) + p2 * t;
  vector p23 = p2 * (1-t) + p3 * t;
  vector p012 = p01 * (1-t) + p12 * t;
  vector p123 = p12 * (1-t) + p23 * t;
  return p012 * (1-t) + p123 * t;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  Quaternion<double> q01  = Slerp(t, p0, p1);
  Quaternion<double> q12  = Slerp(t, p1, p2);
  Quaternion<double> q23  = Slerp(t, p2, p3);
  Quaternion<double> q012 = Slerp(t, q01, q12);
  Quaternion<double> q123 = Slerp(t, q12, q23);
  return Slerp(t, q012, q123);
}

