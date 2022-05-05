#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Skin.hpp>
#include <webots/Keyboard.hpp>
#include <webots/bvh_util.h>
#include <math.h>
#include <array>

using namespace webots;
using namespace std;

int main(int argc, char **argv) {

  struct Frame {
    double rootT[3];
    vector<double> boneOrientation;
  };
  struct BvhFile {
    string fileName;
    vector<Frame> frames;
    int startFrame;
    int endFrame;
  };  
  
  // download from http://www.cgspeed.com/
  // http://www.mediafire.com/file/cu0a99w1day8hnr/cmuconvert-mb2-15-19.zip/file
  // todo: 
  //   stop->walk: 16_31.bvh frame 40->108
  //   (frame 108 is similar to frame 277 of 16_15.bvh)
  //   walk->stop: 16_33.bvh
  vector<BvhFile> bvhFiles = {{
    .fileName="data/16_15.bvh",
    .startFrame=161, .endFrame=301
  }};

  Supervisor *robot = new Supervisor();
  Keyboard *keyboard = robot->getKeyboard();
  keyboard->enable(robot->getBasicTimeStep());
  Field* rotationField=robot->getSelf()->getField("rotation");
  double heading=rotationField->getSFRotation()[3];
  Field* translationField=robot->getSelf()->getField("translation");
  const double* trans=translationField->getSFVec3f();
  double x=trans[0];
  double y=trans[1];  
  Skin *skin=robot->getSkin("skin");
  int boneCount = skin->getBoneCount();
  int rootBoneIndex=-1;
  // feet should touch the ground at walking frame, not t-pose frame
  // Robert scale 1.0 => -0.025
  // Robert scale 0.8 => -0.02
  const double zOffset = -0.02;
  for (auto& f : bvhFiles) {
    WbuBvhMotion data = wbu_bvh_read_file(f.fileName.c_str());
    // CharacterSkin model "Robert":
    // scale 1.0 1.0 1.0 => 2.25m height => 13.5
    // scale 0.8 0.8 0.8 => 1.8m height => 16.875
    wbu_bvh_set_scale(data, 16.875);
    int jointCount=wbu_bvh_get_joint_count(data);
    vector<int> boneToJoint(boneCount, -1);
    for(int i=0; i<boneCount; i++) {
      string name = skin->getBoneName(i);
      if(name=="Hips")
        rootBoneIndex = i;
      if(name=="LThumb" || name=="LeftFingerBase" || name=="LeftHandFinger1" ||
        name=="RThumb" || name=="RightFingerBase" || name=="RightHandFinger1")
        continue;
      for(int j=0; j<jointCount; j++)
        if(name==wbu_bvh_get_joint_name(data, j))
          boneToJoint[i] = j;
      if(boneToJoint[i] != -1)
        for(int j=0; j<2; j++)
          wbu_bvh_set_model_t_pose(data, skin->getBoneOrientation(i, j), boneToJoint[i], j);
    }
    int frameCount = wbu_bvh_get_frame_count(data);
    f.frames.resize(frameCount);
    for(int i=0; i<frameCount; i++) {
      wbu_bvh_goto_frame(data, i);
      const double *p = wbu_bvh_get_root_translation(data);
      for(int j=0; j<3; j++)
        f.frames[i].rootT[j] = p[j];
      f.frames[i].boneOrientation.resize(boneCount*4, NAN);
      for (int j=0; j<boneCount; j++) {
        const double* ori;
        if (boneToJoint[j]!=-1)
          ori = wbu_bvh_get_joint_rotation(data, boneToJoint[j]);
        else
          ori = skin->getBoneOrientation(j, false);
        for(int k=0; k<4; k++)
          f.frames[i].boneOrientation[j*4+k] = ori[k];
      }
    }
    wbu_bvh_cleanup(data);
  }
  if(rootBoneIndex==-1) {
    cout << "Hips not found" << endl;
    return -1;
  }
  const double maxRotSpeed = 3.0; // radians per second
  const double maxSpeed = 240.0; // frames per second
  const double maxAcceleration = 300.0; // change of frame speed per second
  double vx=0.0; // frames per second
  BvhFile &f=bvhFiles[0];
  double frame=f.startFrame;
  while (robot->step(robot->getBasicTimeStep()) != -1) {
    int kx=0, ky=0;
    int key;
    while( (key = keyboard->getKey()) != -1) {
      // press both keys at same time for double speed
      if(key==keyboard->UP || key=='W')
        kx++;
      if(key==keyboard->DOWN || key=='S')
        kx--;
      if(key==keyboard->LEFT || key=='A')
        ky++;
      if(key==keyboard->RIGHT || key=='D')
        ky--;
    }
    double t=robot->getBasicTimeStep()/1000.0;
    heading += ky*maxRotSpeed*t;
    double rot[4]={0.0, 0.0, 1.0, heading};
    rotationField->setSFRotation(rot);
    double target_vx=kx*maxSpeed;
    double acc=(target_vx-vx)/t;
    if(acc>maxAcceleration)
      acc=maxAcceleration;
    if(acc<-maxAcceleration)
      acc=-maxAcceleration;
    vx+=acc*t;
    double *startT = f.frames[(int)frame].rootT;
    frame+=vx*t;
    double *endT = f.frames[(int)frame].rootT;
    double posChange=0.0;
    for(int i=0; i<=2; i+=2)
      posChange += (startT[i]-endT[i])*(startT[i]-endT[i]);
    posChange=sqrt(posChange);
    kx=vx>0?1:-1;
    x+=kx*cos(heading)*posChange;
    y+=kx*sin(heading)*posChange;
    const double trans[3]={x, y, endT[1]-f.frames[0].rootT[1]+zOffset};
    translationField->setSFVec3f(trans);
    int loopLength = f.endFrame-f.startFrame;
    while(frame>=f.endFrame)
      frame-=loopLength;
    while(frame<f.startFrame)
      frame+=loopLength;
    vector<double> vec=f.frames[frame].boneOrientation;
    for (int i=0; i<boneCount; i++) {
      const double x[4]={vec[4*i], vec[4*i+1], vec[4*i+2], vec[4*i+3]};
      skin->setBoneOrientation(i, x, false);
    }
  }
  return 0;
}
