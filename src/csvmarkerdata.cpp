#include "csvmarkerdata.h"
#include <Eigen/Dense>
#include "CapBody.h"
#include <fstream>
#include <iostream>
#include <math.h>

using namespace std;

CSVMarkerData::CSVMarkerData(dWorldID world,dSpaceID space,QObject *parent) :
    QObject(parent)
{
  this->world=world;
  this->space=space;

  loadData();

  marker_count = MARKERS;
  cFrame = NULL;
  nFrame = NULL;

  geom = new dGeomID[marker_count];
  body = new dBodyID[marker_count];
  joint = new dJointID[marker_count];
  try_link = new bool[marker_count];
  jointGroup = dJointGroupCreate(0);
  for (int ii=0;ii<marker_count;++ii) {
    body[ii] = dBodyCreate(world);
    dBodySetKinematic(body[ii]);
    geom[ii] = dCreateSphere(space,.01);
    dGeomSetBody(geom[ii],body[ii]);
    try_link[ii]=false;
    joint[ii] = dJointCreateBall(world,jointGroup);
  }

  framesPerStep=1;
  timePerStep = .015;
  current_frame = 0;
  paused=true;
  singleStep=false;
}

//void readInData(std::ifstream&);

void CSVMarkerData::loadData()
{
  //FILE* psFile = fopen("psData.dat","rb");
  //FILE* headerFile = fopen("header.txt","r");

  //double seconds;
  //unsigned int psFrame;
  //int markers;
  //std::ifstream datapts_file("../QtVR/data/ind002_L000_tr15_xyzptsTruncated.csv");
  std::ifstream datapts_file("../QtVR/data/just330by27.csv");
  readInData(datapts_file);

  std::cout << "The CSV data has been loaded." << std::endl;
  //int boards;
  //fscanf(headerFile,"%lf %u %d %d\n",&seconds,&psFrame,&markers,&boards);

  //data.resize(psFrame);

  //fread(&(data[0]),sizeof(BoardMarkerFrame),psFrame,psFile);
  //fclose(psFile);
  //fclose(headerFile);
}

void CSVMarkerData::readInData(std::ifstream& file){

    bool numColDetermined = false;
    int rows, columns;

    Eigen::MatrixXd m;
    string line;
    //vector<double> fileInfo;
    //vector<string> v;

    vector<string> tokens;
    if(file.is_open()){
        cout << "The file is now open" << endl;
        while(file.good()){
            getline(file, line);
            string buf;
            stringstream ss(line);
            if (line.find_first_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ") != 0){
                while(getline(ss, buf, ',')){
                    tokens.push_back(buf);
                    cout << buf << endl;
                    //cout << buf << endl;
                }
                if(!numColDetermined){
                    columns = tokens.size();
                    //cout << "number of columns is " << columns << endl;
                    numColDetermined = true;
                }
                //break;
            }
        }
        file.close();
    }
    else cout << "Unable to open file" << endl;
    file.clear();

    std::cout << "We have tokenized the data!!!" << std::endl;
    rows = tokens.size()/columns;
    cout << "number of rows is " << rows << endl;
    m.resize(rows, columns);

    for (int i = 0; i < m.rows(); i++){
        for (int j = 0; j < m.cols(); j++){
            m(i,j) = atof(tokens[i*columns+j].c_str());
        }
    }

    //m = m*-0.1;
    m = m*100;
    cout << "We have scaled the data by multiplying by -0.1" << endl;

    CSVMarkerFrame Frame;

    std::cout << "About to load the data into Frames!!!" << std::endl;
    cout << "number of columns: " << m.cols() << endl;

#if 1
    for (int i = 0; i < m.rows(); i++){
        for (int j = 0; j < m.cols(); j=j+3){
            Frame.markers[j/3][0]=m(i,j);
            Frame.markers[j/3][1]=m(i,j+2)-500;
            Frame.markers[j/3][2]=m(i,j+1);
            //if (i==0 && j==0){
            cout << "Frame number: " << i << endl;
            cout << "Frame.markers[" << j/3 << "][" << 0 << "]: " << Frame.markers[j/3][0] << endl;
            cout << "Frame.markers[" << j/3 << "][" << 1 << "]: "<< Frame.markers[j/3][1] << endl;
            cout << "Frame.markers[" << j/3 << "][" << 2 << "]: "<< Frame.markers[j/3][2] << endl;
            //}
        }
        data.push_back(Frame);
    }
#endif

    cout << "Done loading data" << endl;
}

int CSVMarkerData::size()
{
  return data.size();
}

/**
  Update the marker positions and velocities.
  If the data model is paused, then we still
  set the position of the markers so that they
  can be displayed.  However, the velocity is
  zero.  If the model is not paused, we set the
  marker location and set its velocity.  Then
  we advance the current marker frame forward.

  If the current marker frame is not visible,
  the marker goes to the origin (or below the
  plane).
  If either frame is not visible, we don't
  attach the joint.

  */
void CSVMarkerData::step()
{
  // Set the markers to hold the
  // current frame
  int nextFrame = current_frame+framesPerStep;
  int maxFrame = data.size()-1;
  if (current_frame>maxFrame) current_frame=maxFrame;
  if (nextFrame>maxFrame) nextFrame=maxFrame;


  // Get the current frame
  cFrame = &(data[current_frame]);
  // Get the next frame
  nFrame = &(data[nextFrame]);

  dJointGroupEmpty(jointGroup);

  if (paused && !singleStep) {
    for (int ii=0;ii<marker_count;++ii) {
      float xx,yy,zz;
      //if (isnan(cFrame->markers[ii][2]) ) xx = 0;
      //else
      xx = cFrame->markers[ii][2]*.001;
      //if (isnan(cFrame->markers[ii][0])) yy = 0;
      //else
      yy = cFrame->markers[ii][0]*.001;
      //if (isnan(cFrame->markers[ii][1])) zz = 0;
      //else
      zz = cFrame->markers[ii][1]*.001;
      if (cFrame->markers[ii][3]<0) zz=-1;

      dBodySetLinearVel(body[ii],0,0,0);
      dBodySetPosition(body[ii],xx,yy,zz);
    }
  } else {
    for (int ii=0;ii<marker_count;++ii) {
      float xx,yy,zz;
      xx = cFrame->markers[ii][2]*.001;
      yy = cFrame->markers[ii][0]*.001;
      zz = cFrame->markers[ii][1]*.001;
      if (cFrame->markers[ii][3]<0) zz=-1;

      float dx,dy,dz;
      dx = nFrame->markers[ii][2]*.001 - xx;
      dy = nFrame->markers[ii][0]*.001 - yy;
      dz = nFrame->markers[ii][1]*.001 - zz;

      dBodySetLinearVel(body[ii],dx/timePerStep,dy/timePerStep,dz/timePerStep);
      dBodySetPosition(body[ii],xx,yy,zz);
    }
    setFrame(nextFrame);
    singleStep = false;
  }
  for (int ii=0;ii<marker_count;++ii) {
    int bID = body_pointer->marker_to_body[ii].id;
    //std::cout << "MARKER_COUNT: " << MARKER_COUNT << endl;
    //std::cout << "body_Pointer->marker_to_body[" << ii << "]: "<< bID << std::endl;
    if (try_link[ii] && (bID>=0) &&
        (cFrame->markers[ii][3]>0) &&
        (nFrame->markers[ii][3]>0))
    {
      joint[ii]=dJointCreateBall(world,jointGroup);

      dJointAttach(joint[ii],body[ii],body_pointer->body_segments[bID]);
      dJointSetBallAnchor1Rel(joint[ii],0,0,0);
      dJointSetBallAnchor2Rel(joint[ii],
                              body_pointer->marker_to_body[ii].position[0],
                              body_pointer->marker_to_body[ii].position[1],
                              body_pointer->marker_to_body[ii].position[2]);
      dJointSetBallParam(joint[ii],dParamCFM,.0001);
      dJointSetBallParam(joint[ii],dParamERP,.2);
    }

  }

}


void CSVMarkerData::setPaused(bool playing)
{
  this->paused = !playing;
}

void CSVMarkerData::setSingleStep()
{
  singleStep=true;
  paused = true;
}

void CSVMarkerData::setFrame(int frame)
{
  if (frame>=0 && frame<size() && current_frame!=frame) {
    current_frame=frame;
    emit frameChanged(frame);
  }
}

void CSVMarkerData::changeBodyConnect(int mark,int body)
{
  body_pointer->marker_to_body[mark].id=body;
}

void CSVMarkerData::changeBodyLink(int mark,bool link)
{
  try_link[mark]=link;
}

void CSVMarkerData::changeLinkPos(int mark,double xx,double yy,double zz)
{
  body_pointer->marker_to_body[mark].position[0]=xx;
  body_pointer->marker_to_body[mark].position[1]=yy;
  body_pointer->marker_to_body[mark].position[2]=zz;
}

