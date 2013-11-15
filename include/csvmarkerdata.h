#ifndef CSVMARKERDATA_H
#define CSVMARKERDATA_H

#include <QObject>
#include <ode/ode.h>
#include <vector>
using std::vector;

class CapBody;

#define MARKERS 37
//#define BOARDS 2
//#define BPT 5

struct CSVMarkerFrame {
  double time;
  double markers[MARKERS][3]; //50 markers: x,y,z
  //double boards[BOARDS][BPT][4]; //2 boards, 4 corners, x,y,z,v
  CSVMarkerFrame()
  {
    time=0;
    //memset(markers,0,sizeof(double)*MARKERS*4);
    //memset(boards,0,sizeof(double)*BOARDS*BPT*4);
  }
};

class CSVMarkerData : public QObject
{
    Q_OBJECT
public:
    explicit CSVMarkerData(dWorldID world,dSpaceID space,QObject *parent = 0);

  /// We set the markers to be where the data frame
  /// says they should be.  We set their velocity to
  /// match where they will be with the next step.
  void step();

  // Need to check on dimensions

  void setStepSize(int stepSize) {framesPerStep=stepSize;}
  /// This allows us to set the body's velocities correctly.
  void setStepTime(double stepTime) {timePerStep = stepTime;}

  /// For rendering purposes.
  dSpaceID getSpace() { return space; }

  int size();
  bool isPaused() {return paused;}

signals:
  void frameChanged(int);

public slots:

  void setPaused(bool);
  void setSingleStep();
  void setFrame(int frame);

  void changeBodyConnect(int mark,int body);
  void changeBodyLink(int mark,bool link);
  void changeLinkPos(int mark,double xx,double yy,double zz);
protected:
  void loadData();
  void readInData(std::ifstream&);

public:
  int framesPerStep;
  double timePerStep;
  int current_frame;
  /// If the object is paused, the markers have constant velocity.
  bool paused;
  bool singleStep;
  dSpaceID space;
  dWorldID world;

  dGeomID* geom;
  dBodyID* body;
  dJointID* joint;
  dJointGroupID jointGroup;
  bool* try_link;

  CSVMarkerFrame* cFrame;
  CSVMarkerFrame* nFrame;

  int marker_count;
  CapBody* body_pointer;


  // Data Structure
  vector<CSVMarkerFrame> data;
};

#endif // CSVMARKERDATA_H
