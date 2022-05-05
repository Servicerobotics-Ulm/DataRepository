#include <webots/Supervisor.hpp>
#include <webots/Display.hpp>
#include <cmath>
#include <webots/utils/AnsiCodes.hpp>

using namespace webots;
using namespace std;

#define CHECK_FIELD(x) checkField((x), __LINE__)

Field* checkField(Field *f, int lineNumber) {
    if (!f)
        cerr << "missing getField in line " << __LINE__ << endl;
    return f;
}

bool hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int main(int argc, char **argv) {
  Supervisor *robot = new Supervisor();
  Display* display=robot->getDisplay("display");
  Node* self=robot->getSelf();
  while (robot->step(robot->getBasicTimeStep()) != -1) {
    int width = display->getWidth();
    int height = display->getHeight();
    const double *backgroundColor=CHECK_FIELD(self->getField("backgroundColor"))->getSFColor();
    int color=0;
    for(int i=0; i<3; i++)
      color=color*256+lround(255*backgroundColor[i]);
    display->setColor(color);
    display->setAlpha(1.0-CHECK_FIELD(self->getField("backgroundTransparency"))->getSFFloat());
    display->fillRectangle(0, 0, width, height);
    display->setFont(CHECK_FIELD(self->getField("textFont"))->getSFString(), CHECK_FIELD(self->getField("textSize"))->getSFInt32(), CHECK_FIELD(self->getField("antiAliasing"))->getSFBool());
    const double *textColor=CHECK_FIELD(self->getField("textColor"))->getSFColor();
    color=0;
    for(int i=0; i<3; i++)
      color=color*256+lround(255*textColor[i]);
    display->setColor(color);
    display->setAlpha(1.0-CHECK_FIELD(self->getField("textTransparency"))->getSFFloat());
    const double *textXY = CHECK_FIELD(self->getField("textXY"))->getSFVec2f();
    Field *dataField = CHECK_FIELD(self->getField("text"));
    string data = "";
    for (int j = 0; j < dataField->getCount(); j++)
        data += dataField->getMFString(j)+"\n";
    display->drawText(data, textXY[0], textXY[1]);
    bool saveNow=CHECK_FIELD(self->getField("saveNow"))->getSFBool();
    if(saveNow) {
      CHECK_FIELD(self->getField("saveNow"))->setSFBool(false);      
      string fileName=CHECK_FIELD(self->getField("fileName"))->getSFString();
      if(!hasEnding(fileName, ".png") && !hasEnding(fileName, ".PNG"))
        fileName+=".png";
      if(fileName.find('/') == string::npos)
        fileName = "../../worlds/textures/TextToPNG/"+fileName;
      cout << webots::AnsiCodes::GREEN_FOREGROUND << "Write " << fileName << webots::AnsiCodes::RESET << endl;
      ImageRef *imageRef = display->imageCopy(0, 0, width, height);
      display->imageSave(imageRef, fileName);
      display->imageDelete(imageRef);
    }
  }
}
