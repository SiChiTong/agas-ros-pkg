/*******************************************************************************
 *  CoordinateConverter.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  $Id: CoordinateConverter.cpp 44313 2011-04-06 22:46:28Z agas $
 ******************************************************************************/


/**
 * COORDINATE SYSTEMS:
 *
 * nativeLaser
 * laser
 * laser3D
 * simLaser3D
 * robot
 * world
 * map
 */

#include <cmath>
#include <iostream>

#include "CoordinateConverter.h"
//#include "Architecture/Config/Config.h"

#define THIS CoordinateConverter

using namespace std;

THIS* THIS::m_Instance = 0;

THIS* THIS::getInstance() {
  if(m_Instance == 0){
    m_Instance = new THIS();
  }
  return m_Instance;
}


THIS::THIS() {
  float mWidth = 35;//Config::getInt("Map.iSize");
  float mHeight = 35;//Config::getInt("Map.iSize");
  m_CellSize = 0.05;//Config::getInt("Map.iCellSize");


  m_MapWidth = mWidth / m_CellSize + 1;
  m_MapHeight = mHeight / m_CellSize + 1;
  m_MapHalfWidth = m_MapWidth / 2;
  m_MapHalfHeight = m_MapHeight / 2;
}


THIS::~THIS() {
}


///
/// SINGLE CONVERSION STEPS
///


Point2D THIS::robotToWorld(Pose robotPose, Point2D robotPoint) {
  float sinTheta = sin(robotPose.theta());
  float cosTheta = cos(robotPose.theta());
  float x = cosTheta * robotPoint.x() - sinTheta * robotPoint.y() + robotPose.x();
  float y = sinTheta * robotPoint.x() + cosTheta * robotPoint.y() + robotPose.y();
  return Point2D(x, y);
}

Pose THIS::robotToWorld( Pose robotPose, Pose pose) {

  Point2D newPose = robotToWorld ( robotPose, Point2D(pose.x(), pose.y()) );

  double angle = robotPose.theta() + pose.theta();
  while(angle > M_PI){
    angle -= 2*M_PI;
  }
  while(angle < -M_PI){
    angle += 2*M_PI;
  }

  return Pose(newPose.x(), newPose.y(), angle);
}

Transformation2D THIS::robotToWorld(Pose robotPose, Transformation2D transformation) {
  float sinTheta = sin(robotPose.theta());
  float cosTheta = cos(robotPose.theta());
  float x = cosTheta * transformation.x() - sinTheta * transformation.y();
  float y = sinTheta * transformation.x() + cosTheta * transformation.y();
  return Transformation2D(x, y, transformation.theta());
}

Point2D THIS::worldToRobot(Pose robotPose, Point2D worldPoint) {
  float sinTheta = sin(-robotPose.theta());
  float cosTheta = cos(-robotPose.theta());
  float x = cosTheta * (worldPoint.x() - robotPose.x()) - sinTheta * (worldPoint.y() - robotPose.y());
  float y = sinTheta * (worldPoint.x() - robotPose.x()) + cosTheta * (worldPoint.y() - robotPose.y());
  return Point2D(x, y);
}

Transformation2D THIS::worldToRobot(Pose robotPose, Transformation2D transformation) {
  float sinTheta = sin(-robotPose.theta());
  float cosTheta = cos(-robotPose.theta());
  float x = cosTheta * (transformation.x() - robotPose.x()) - sinTheta * (transformation.y() - robotPose.y());
  float y = sinTheta * (transformation.x() - robotPose.x()) + cosTheta * (transformation.y() - robotPose.y());
  return Transformation2D(x, y, transformation.theta());
}

Pixel THIS::worldToMap(Point2D worldPoint) {
  int pixelX = m_MapHalfWidth - (worldPoint.y()/m_CellSize+0.5);
  int pixelY = m_MapHalfHeight - (worldPoint.x()/m_CellSize+0.5);
  if (pixelX < 0 || pixelX >= m_MapWidth || pixelY < 0 || pixelY >= m_MapHeight) {
    if ( pixelX < 0 ) { pixelX=0; }
    if ( pixelX >= m_MapWidth ) { pixelX=m_MapWidth-1; }
    if ( pixelY < 0 ) { pixelY=0; }
    if ( pixelY >= m_MapHeight ) { pixelY=m_MapHeight-1; }
  }
  return Pixel(pixelX, pixelY);
}

Point2D THIS::worldToMapPoint(Point2D worldPoint) {
  double pixelX = double(m_MapHalfWidth) - (worldPoint.y()/double(m_CellSize)+0.5);
  double pixelY = double(m_MapHalfHeight) - (worldPoint.x()/double(m_CellSize)+0.5);
  if (pixelX < 0 || pixelX >= m_MapWidth || pixelY < 0 || pixelY >= m_MapHeight) {
    if ( pixelX < 0 ) { pixelX=0; }
    if ( pixelX >= m_MapWidth ) { pixelX=m_MapWidth-1; }
    if ( pixelY < 0 ) { pixelY=0; }
    if ( pixelY >= m_MapHeight ) { pixelY=m_MapHeight-1; }
  }
  return Point2D(pixelX, pixelY);
}

void THIS::worldToMap(float worldX, float worldY, int& mapX, int& mapY) {
  Pixel mapPixel = worldToMap(Point2D(worldX, worldY));
  mapX = mapPixel.x();
  mapY = mapPixel.y();
}

Pixel THIS::worldToMap(Pose worldPose) {
  return worldToMap(Point2D(worldPose.x(), worldPose.y()));
}

Point2D THIS::mapToWorld(Pixel mapPoint) {
  float worldX = (float(m_MapHalfHeight) - 0.5 - float(mapPoint.y())) * float(m_CellSize);
  float worldY = (float(m_MapHalfWidth) - 0.5 - float(mapPoint.x())) * float(m_CellSize);
  return Point2D(worldX, worldY);
}


// spherical to cartesian
Vector3D THIS::sphericalToCartesian(float range, float phi, float theta){
  float x = range * sin(phi) * cos(theta);
  float y = range * sin(phi) * sin(theta);
  float z = range * cos(phi);
  return Vector3D(x, y, z);
}

// spherical plane to world coordinate
Vector3D THIS::sphericalPlaneToWorldCoordRad(float range, float h, float theta_rad, float phi_rad)
{
    const float cos_phi = cos(phi_rad);
    // alpha = theta - 90 degree (theta = 0, alpha = -90; theta = 90, alpha = 0; theta = 180, alpha = 90)
    const float difference = 90.0*M_PI/180.0;
    const float alpha = theta_rad-difference;

    float x = range * cos(alpha) * cos_phi + h * cos(theta_rad);
    float y = range * sin(phi_rad);
    float z = range * sin(alpha) * cos_phi + h * sin(theta_rad);
  return Vector3D( x, y, z );
}

Vector3D THIS::simLaser3DToLaser3D(float alpha, float beta, float range){
  return sphericalToCartesian(range, (M_PI/2.0 + beta), alpha);
}

// computes the worldpoints itself and does not call robotToWorld(Pose, Point2D)
// to avoid multiple computation of sin(theta) and cos(theta)
vector<Point2D> THIS::robotToWorld(Pose robotPose, const vector<Point2D>& robotPoints) {
  float sinTheta = sin(robotPose.theta());
  float cosTheta = cos(robotPose.theta());
  vector<Point2D> worldPoints( robotPoints.size() );
  for (unsigned int i = 0; i < robotPoints.size(); i++) {
    float x = cosTheta * robotPoints[i].x() - sinTheta * robotPoints[i].y() + robotPose.x();
    float y = sinTheta * robotPoints[i].x() + cosTheta * robotPoints[i].y() + robotPose.y();
    worldPoints[i].setX( x );
    worldPoints[i].setY( y );
  }
  return worldPoints;
}

vector<Pixel> THIS::worldToMap( const vector<Point2D>& worldPoints ) {
  vector<Pixel> ret( worldPoints.size() );
  for (unsigned int i = 0; i < worldPoints.size(); i++) {
    ret[i] = worldToMap(worldPoints[i]);
  }
  return ret;
}

vector<Point2D> THIS::mapToWorld( const vector<Pixel>& mapPoints ) {
  vector<Point2D> ret( mapPoints.size() );
  for (unsigned int i = 0; i < mapPoints.size(); i++) {
    ret[i]= mapToWorld( mapPoints[i] );
  }
  return ret;
}

vector<Vector3D> THIS::simLaser3DToLaser3D(float fovH, float fovV, float resH, float resV, vector<float> *range){
  vector<Vector3D> ret;
  float index = 0;
  float angleH = 0;
  float angleV = 0;
  float deg2rad = 1.0/180.0 * M_PI;
  float stepH = fovH / resH;
  float stepV = fovV / resV;
  fovH = -1.0 * fovH/2.0;
  fovV = -1.0 * fovV/2.0;
  //cout<<"   Resolution: "<<resH<<"x"<<resV<<endl;
  //cout<<"          FOV: "<<fovH<<"x"<<fovV<<endl;
  //cout<<"         Step: "<<stepH<<"x"<<stepV<<endl;
  for (int v = 0; v<resV; v++){
      angleV = (fovV + (v * stepV)) * deg2rad;
      for (int h = 0; h<resH; h++){
        angleH = (fovH + (h * stepH)) * deg2rad;
        index = v*resH+h;
        if(index < range->size()) ret.push_back(simLaser3DToLaser3D(angleH, angleV, range->at(index)));
      }
  }
  return ret;
}




Pixel THIS::robotToMap(Pose robotPose, Point2D robotPoint) {
  return worldToMap(robotToWorld(robotPose, robotPoint));
}

vector<Pixel> THIS::robotToMap(Pose robotPose, const vector<Point2D>& robotPoints) {
  vector<Point2D> worldPoints = robotToWorld(robotPose, robotPoints);
  return worldToMap( worldPoints);
}

void THIS::robotToMap(float robotX, float robotY, float robotTheta, float pointX, float pointY, int& mapX, int& mapY) {
  Pixel pix = robotToMap(Pose(robotX, robotY, robotTheta), Point2D(pointX, pointY));
  mapX = pix.x();
  mapY = pix.y();
}

Point2D THIS::polarToCartesian(float theta, float distance) {
  float x = sin(theta) * distance;
  float y = cos(theta) * distance;
  return Point2D(x, y);
}

#undef THIS
