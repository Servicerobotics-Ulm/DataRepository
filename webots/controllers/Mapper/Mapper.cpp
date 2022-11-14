// now save and reload needed after import
//   => do import without save/reload
//   (display width/height shows warning after changeing width/height; but change is needed)
//   walls_*.proto has to reload too

// how to read .jpg/.png file?

// todo: import .png file
//       import .jpg file
//       export .jpg file ?
// world must be in ENU coordinate system

#include <webots/Supervisor.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>
#include <webots/utils/AnsiCodes.hpp>

#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string.h>
#include <libgen.h> // dirname()

// yaml a is human readable file format for data like json
// libsmartsoft-yaml 0.2.5 is installed with smartsoft
// add in Makefile:
// CFLAGS = -Wno-deprecated-declarations
// INCLUDE = -I"/usr/include/YAML_CPP"
// LIBRARIES = -L/usr/lib -lYAML_CPP
#include <yaml.h>

using namespace std;

// https://stackoverflow.com/questions/8126815/how-to-read-in-data-from-a-pgm-file-in-c
// http://netpbm.sourceforge.net/doc/pgm.html

// works with both ascii or utf8 encoding for path (may not work with linux files with \ inside)
string getFilenameFromPath(string path) {
  int i;
  for(i=path.length(); i--;)
    if(path[i]=='/' || path[i]=='\\')
      break;
  return path.substr(i+1);
}

int main(int argc, char **argv) {
  webots::Supervisor *robot = new webots::Supervisor();
  webots::Display *display = robot->getDisplay("display");
  webots::RangeFinder *rangeFinder = robot->getRangeFinder("range-finder");
  string worldName = getFilenameFromPath(robot->getWorldPath());
  worldName = worldName.substr(0, worldName.length()-4); // remove .wbt
  webots::Node* myNode = robot->getSelf();  
  while (robot->step(robot->getBasicTimeStep()) != -1) {
    double resolution = myNode->getField("resolution")->getSFFloat();
    const double *originValues = myNode->getField("origin")->getSFVec2f();
    double origin[3] = {originValues[0], originValues[1], 0.0};
    int mx = myNode->getField("width")->getSFInt32();
    int my = myNode->getField("height")->getSFInt32();
    double wallHeight = myNode->getField("wallHeight")->getSFFloat();
    bool showMapBorders = myNode->getField("showMapBorders")->getSFBool();
    double depth = resolution * 100000; // depth of RangeFinder [m]
    if(true) { // update hidden fields
      double x1 = origin[0] - resolution/2;
      double y1 = origin[1] - resolution/2;
      double x2 = origin[0] + mx*resolution + resolution/2;
      double y2 = origin[1] + my*resolution + resolution/2;
      struct fieldValue { string name; double values[3]; };
      fieldValue fieldValues[9] = {
        {"border1_translation", {x1, (y1+y2)/2, wallHeight}},
        {"border2_translation", {x2, (y1+y2)/2, wallHeight}},
        {"border3_translation", {(x1+x2)/2, y1, wallHeight}},
        {"border4_translation", {(x1+x2)/2, y2, wallHeight}},
        {"border1_size", {resolution, my*resolution, wallHeight*2}},
        {"border3_size", {mx*resolution, resolution, wallHeight*2}},
        {"cone2_translation", {x2+resolution+wallHeight/2, 0, 0}},
        {"cone4_translation", {0, y2+resolution+wallHeight/2, 0}},
        {"range_translation", {origin[0]+mx*resolution/2, origin[1]+my*resolution/2, -depth}}
      };
      for(int i=0; i<(int)(sizeof(fieldValues)/sizeof(*fieldValues)); i++)
        myNode->getField(fieldValues[i].name)->setSFVec3f(fieldValues[i].values);
      myNode->getField("fieldOfView")->setSFFloat(2.0*atan(mx*resolution/2/depth));
      myNode->getField("cone_radius")->setSFFloat(wallHeight/2);
      myNode->getField("maxRange")->setSFFloat(2*depth);
      myNode->getField("MapBorderTransparency")->setSFFloat(showMapBorders ? 0.5 : 1.0);
    }
    if(rangeFinder->getSamplingPeriod()!=0) {
      const float * rangeImage = rangeFinder->getRangeImage();
      rangeFinder->disable();
      cout << webots::AnsiCodes::GREEN_FOREGROUND << "Get RangeFinder image (depth=" << depth << " width=" << mx << " height=" << my << ")" << webots::AnsiCodes::RESET << endl;
      for(int y=0; y<my; y++) {
        for(int x=0; x<mx; x++) {
          bool b = rangeImage[(my-1-y)*mx+x] < depth + wallHeight;
          display->setColor(b ? 0xFFFFFF : 0);
          display->drawPixel(x,y);
        }
      }
      webots::ImageRef *imageRef = display->imageCopy(0, 0, mx, my);
      cout << webots::AnsiCodes::GREEN_FOREGROUND << "Write worlds/maps/"+worldName+".png" << webots::AnsiCodes::RESET << endl;
      display->imageSave(imageRef, "../../worlds/maps/"+worldName+".png");
      display->imageDelete(imageRef);
      cout << "Export finished." << endl;
    }
    string filename = robot->wwiReceiveText();
    if(filename.length()==0)
      continue;
    if(filename == "exportMap") {
      rangeFinder->enable(robot->getBasicTimeStep());
      continue;
    }
    if(filename.rfind("importMap ", 0)!=0) {
      cerr << "wrong command "+filename << endl;
      continue;
    }
    filename = filename.substr(10);
    std::ifstream fin(filename.c_str());
    if (fin.fail()) {
      cerr << "Could not open " << filename << endl;
      continue;
    }
    int negate=0;
    double occupied_thresh=0.196, free_thresh=0.65;
    if(filename.length()>5 && filename.substr(filename.length()-5)==".yaml") {
      cout << webots::AnsiCodes::GREEN_FOREGROUND << "Read .yaml file: '" << filename << "'" << webots::AnsiCodes::RESET << endl;
      YAML::Parser parser(fin);
      YAML::Node doc;
      parser.GetNextDocument(doc);
      string image = "";
      try {
        doc["image"] >> image;
        doc["resolution"] >> resolution;
        bool boolNegate; // GMapping writes "negate: false" instead of "negate: 0"
        try {
          doc["negate"] >> boolNegate;
          negate = boolNegate;
        } catch (YAML::InvalidScalar &e) {
          doc["negate"] >> negate;
        }
        doc["occupied_thresh"] >> occupied_thresh;
        doc["free_thresh"] >> free_thresh;
        doc["origin"][0] >> origin[0];
        doc["origin"][1] >> origin[1];
        doc["origin"][2] >> origin[2];
      } catch (YAML::InvalidScalar &e) {
        cerr << ".yaml file has missing field:" << e.what() << endl;
        continue;
      }
      if (image.size() == 0) {
        cerr << "empty image name" << endl;
        continue;
      }
      if (image[0] != '/') {
        char* fname_copy = strdup(filename.c_str());
        image = string(dirname(fname_copy)) + '/' + image;
        free(fname_copy);
      }
      cout
        << "image: " << image << endl
        << "resolution: " << resolution << endl
        << "negate: " << negate << endl
        << "occupied_thresh: " << occupied_thresh << endl
        << "free_thresh: " << free_thresh << endl
        << "origin: [" << origin[0] << "," << origin[1] << "," << origin[2] << "]" << endl;
      filename = image;
    } 

// https://gehrcke.de/2011/06/reading-files-in-c-using-ifstream-dealing-correctly-with-badbit-failbit-eofbit-and-perror/
    cout << webots::AnsiCodes::GREEN_FOREGROUND << "Read .pgm file: '" << filename << "'" << webots::AnsiCodes::RESET << endl;
    cout.flush();
    ifstream f (filename.c_str());
    if (!f.is_open()) {
      perror(("error while opening file " + filename).c_str());
      continue;
    }
    string line;
    int programCounter=0, maxval, x, y;
    while(getline(f, line)) {
      if(programCounter==0) {
        if(line.compare("P5") != 0) {
          if(line.compare("P2") != 0)
            cerr << "Can't read 'plain' (ASCII) PGM files, only 'raw' PGM files" << endl;
          else
            cerr << "Wrong file type" << endl;
          return -1;
        }
        programCounter=1;
      } else if(line.empty() || line.at(0)=='#')
        ; // do nothing for empty lines or comment lines
      else {
        stringstream ss(line);
        if(programCounter==1 && (ss>>mx))
          programCounter++;
        if(programCounter==2 && (ss>>my))
          programCounter++;
        if(programCounter==3 && (ss>>maxval))
          break;
      }
    }
    if (f.bad()) {
      perror(("error while reading file " + filename).c_str());
      continue;
    }
    cout << "width " << mx << " height " << my << " maxval " << maxval << endl;
    // more than 1 gb of data => error
    if( (0.0+mx)*my > 1000000000.0 || mx<=0 || my<=0 || maxval!=255) {
      cerr << "too large (more than 1 GB) or maxval is not 255 or negative values" << endl;
      return -1;
    }
// https://stackoverflow.com/questions/23989308/read-a-file-using-c-and-raii
    std::unique_ptr<char[]> buffer(new char[mx*my]);
    if(!f.read(buffer.get(), mx*my)) {
      cerr << "could not read " << mx*my << " characters" << endl;
      return -1;
    }
    f.close();
    
    // add empty pixels at border to make code more easy (guard)
    int mpx = mx+2;
    int mpy = my+2;
    bool* pixel = new bool[mpx*mpy](); // true = black = wall
#define PIXEL(x,y) pixel[(y)*mpx+(x)]
    int *indexOfPoint = new int[mpx*mpy];
#define INDEX_OF_POINT(x,y) indexOfPoint[(y)*mpx+(x)]
    for(y=mpy; y--;)
        for(x=mpx; x--;)
          INDEX_OF_POINT(x,y) = -1;
    int threshold = (int)(occupied_thresh*255);
    for(y=0; y<my; y++)
      for(x=0; x<mx; x++) {
        uint8_t value = (uint8_t)(buffer.get()[y*mx+x]);
        if(!negate)
          value = 255-value;
        PIXEL(x+1, (my-1-y)+1) = value > threshold;
      }

    // .png not used any more, as webots does not correctly handle transparent pixels from below (RangeFinder) or above (click with mouse)
/*
    // save .png
    // width/height must be a power of 2 or webots will rescale it        
    int width, height;
    for(width=1; width<mx; width*=2)
      ;
    for(height=1; height<my; height*=2)
      ;
    cout << webots::AnsiCodes::GREEN_FOREGROUND << "Write protos/walls/"+worldName+".png" << webots::AnsiCodes::RESET <<  " (" << width << "x" << height << ", top)" << endl;      
    unsigned char* abgrImage = new unsigned char[4*width*height](); // initialize with 0
    for(y=my; y--;)
      for(x=mx; x--;) {
        int i=(y*width+x)*4;
        // pixel true => (#ffffff = white, opacity=FF) => white pixel
        // pixel false => #000 = (black, opacity=0) => transparent pixel
        abgrImage[i] = abgrImage[i+1] = abgrImage[i+2] = abgrImage[i+3] = (PIXEL(x+1, y+1) * 255);
      }
    webots::ImageRef* imageRef = display->imageNew(width, height, abgrImage, webots::Display::ABGR);
    display->imageSave(imageRef, "../../protos/walls/"+worldName+".png");
    display->imageDelete(imageRef);
    delete[] abgrImage;
*/
    
//  delete[] buffer;

    // walls around black pixel[0,0] if neighbouring pixel is white:
    //
    //  0/0 --A--> 0/1
    //   ^          |
    //   D   [0,0]  B
    //   |          v
    //  1/0 <--C-- 1/1
    
    struct coordXY { double x; double y; double z; };
    vector<coordXY> point;
    vector<int> index;
    // https://stackoverflow.com/questions/4324763/can-we-have-functions-inside-functions-in-c
    // lambda function to simulate nested function to access local variables
    float pixelSize = resolution; // [m]
    float xOrigin = origin[0]; // [m]
    float yOrigin = origin[1]; // [m]
    auto addWall = [&] (int x1, int y1, int x2, int y2) {
//      cout << "addWall(" << x1 << " " << y1 << " " << x2 << " " << y2 << ")" << endl;
      int index1 = INDEX_OF_POINT(x1,y1);
      if(index1==-1) {
        index1 = point.size();
        INDEX_OF_POINT(x1,y1) = index1;
        point.push_back({(x1-1)*pixelSize, (y1-1)*pixelSize, 0.0});
        point.push_back({(x1-1)*pixelSize, (y1-1)*pixelSize, wallHeight});
      }
      int index2 = INDEX_OF_POINT(x2, y2);
      if(index2==-1) {
        index2 = point.size();
        INDEX_OF_POINT(x2, y2) = index2;
        point.push_back({(x2-1)*pixelSize, (y2-1)*pixelSize, 0.0});
        point.push_back({(x2-1)*pixelSize, (y2-1)*pixelSize, wallHeight});
      }
      index.push_back(index1);
      index.push_back(index2);
      index.push_back(index2+1);
      index.push_back(-1);
      index.push_back(index1);
      index.push_back(index2+1);
      index.push_back(index1+1);
      index.push_back(-1);
    };
    
    int first=0;
    // A
    for(y=0; y<mpy; y++)
      for(x=0; x<mpx; x++) {
        // begin or expand wall
        if(PIXEL(x,y) && !PIXEL(x,y-1)) {
          if(!first) // begin wall
            first=x;
        } else { // end wall
          if(first)
            addWall(first, y, x, y);
          first = 0;
        }
      }
    // B
    for(x=0; x<mpx; x++)
      for(y=0; y<mpy; y++) {
        if(PIXEL(x,y) && !PIXEL(x+1,y)) {
          if(!first)
            first=y;
        } else {
          if(first)
            addWall(x+1, first, x+1, y);
          first = 0;
        }
      }
    // C
    for(y=mpy; y--;)
      for(x=mpx; x--;) {
        if(PIXEL(x,y) && !PIXEL(x,y+1)) {
          if(!first)
            first=x;
        } else {
          if(first) 
            addWall(first+1, y+1, x+1, y+1);
          first = 0;
        }
      }
    // D
    for(x=mpx; x--;)
        for(y=mpy; y--;) {
        if(PIXEL(x,y) && !PIXEL(x-1,y)) {
          if(!first)
            first=y;
        } else {
          if(first)
            addWall(x, first+1, x, y+1);
          first = 0;
        }
      }
    cout << point.size() << " points and " << index.size()/4 << " triangles created (wall side)" << endl;
    ofstream outfile;
    outfile.open("../../protos/walls/walls_"+worldName+".proto");
    outfile <<
"#VRML_SIM R2022a utf8"
"\n# template language: javascript"
"\nEXTERNPROTO \"../MapperWalls.proto\""
"\nPROTO walls_"+worldName+" ["
"\n  field SFFloat    resolution   " << resolution <<
// "\n  field SFFloat    width        " << width <<
// "\n  field SFFloat    height       " << height << 
"\n  field SFVec2f    origin       " << xOrigin << " " << yOrigin << 
"\n  field SFFloat    wallHeight   " << wallHeight <<
// "\n  field SFString   pngFile      \"walls/"+worldName+".png\""
"\n  field SFColor    baseColor    1 1 1"
"\n  field MFVec3f    Coordinate   [\n";
    for(auto &i: point)
      outfile << i.x << " " << i.y << " " << i.z << "\n";
    outfile <<
"  ]\n"
"\n  field MFInt32    coordIndex   [\n";
    for(auto &i: index)
      outfile << i << (i==-1 ? "\n" : " ");
    outfile <<
"  ]\n"
"\n  field MFVec3f    BottomCoordinate   [\n";
    vector<int> rowStart(mpy);
    vector<int> rowEnd(mpy);
    vector<int> bottomPolygonEdges;
    for(y=0; y<mpy; y++)
      for(x=0; x<mpx; x++)
        if(PIXEL(x,y)) {
          rowStart[y] = x;
          while(PIXEL(++x,y))
            ;
          rowEnd[y] = x;
          int x2, y2;
          y2 = y;
          while(true) {
            for(x2=rowStart[y2]; x2<rowEnd[y2]; x2++)
              PIXEL(x2, y2) = false;
            y2++;
            x2 = rowStart[y2-1];
            // next line starts at same position?
            if(PIXEL(x2, y2) && !PIXEL(x2-1, y2)) {
              rowStart[y2] = x2;
              for(; PIXEL(x2, y2); x2++)
                ;
              rowEnd[y2] = x2;
            } else {
              x2 = rowEnd[y2-1];
              // next line ends at same position?
              if(!PIXEL(x2, y2) && PIXEL(x2-1, y2)) {
                rowEnd[y2] = x2;
                for(; PIXEL(x2-1, y2); x2--)
                  ;
                rowStart[y2] = x2;
              } else break;
            }
          }
          int polygonEdges = 0;
          for(int y3=y; y3<y2; y3++) {
            if(y3==y || rowStart[y3]!=rowStart[y3-1]) {
              outfile << rowStart[y3]-1 << " " << y3-1 << " " << 0 << "\n";
              polygonEdges++;
            }
            if(y3==y2-1 || rowStart[y3]!=rowStart[y3+1]) {
              outfile << rowStart[y3]-1 << " " << y3 << " " << 0 << "\n";
              polygonEdges++;
            }
          }
          for(int y3=y2-1; y3>=y; y3--) {
            if(y3==y2-1 || rowEnd[y3]!=rowEnd[y3+1]) {
              outfile << rowEnd[y3]-1 << " " << y3 << " " << 0 << "\n";
              polygonEdges++;
            }
            if(y3==y || rowEnd[y3]!=rowEnd[y3-1]) {
              outfile << rowEnd[y3]-1 << " " << y3-1 << " " << 0 << "\n";
              polygonEdges++;
            }
          }
          bottomPolygonEdges.push_back(polygonEdges);
        }
    outfile <<
"  ]\n"
"\n  field MFInt32    BottomCoordIndex   [\n";
    int totalPoints=0;
    for(auto &i:bottomPolygonEdges) {
      for(int j=0; j<i; j++)
        outfile << totalPoints+j << " ";
      outfile << -1 << "\n";
      totalPoints += i;
    }
    cout << totalPoints << " points in " << bottomPolygonEdges.size() << " polygons created (bottom)" << endl;
    outfile <<
"  ]\n"
"]\n"
"{ MapperWalls {\n"
"    resolution IS resolution\n"
// "    width IS width\n"
// "    height IS height\n"
"    origin IS origin\n"
// "    pngFile IS pngFile\n"
"    wallHeight IS wallHeight\n"
"    baseColor IS baseColor\n"
"    Coordinate IS Coordinate\n"
"    coordIndex IS coordIndex\n"
"    BottomCoordinate IS BottomCoordinate\n"
"    BottomCoordIndex IS BottomCoordIndex\n"
"  }\n"
"}\n";
    outfile.close();
    cout << webots::AnsiCodes::GREEN_FOREGROUND << "Write protos/walls/walls_"+worldName+".proto" << webots::AnsiCodes::RESET<< endl;
    cout << "Update parameters" << endl;
    myNode->getField("resolution")->setSFFloat(resolution);
    double originValues2[3] = {origin[0], origin[1], 0.0};
    myNode->getField("origin")->setSFVec2f(originValues2);
    myNode->getField("width")->setSFInt32(mx);
    myNode->getField("height")->setSFInt32(my);
    cout << "Import finished." << endl;
    delete[] pixel;
    delete[] indexOfPoint;
  }
  cout << "stop mapper" << endl;
  delete robot;
  return 0;
}
