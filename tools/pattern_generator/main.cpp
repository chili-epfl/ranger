#include <QtWidgets>

#include "Window.hpp"
#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    Window window;

    if((argc > 1) && ((strcmp(argv[1],"-h") == 0) || (strcmp(argv[1],"help") == 0))) {
      std::cout << "Start the program without any arguments to open the GUI inteface." << std::endl;
      std::cout << "In order to use the program in comand line mode," << std::endl << "you must use the following synthax:" << std::endl;
      std::cout << "   RangerLed PathToInputVideo PathToOutputFile [framerate reduction factor] [switch each led off at end (yes or no)]" << std::endl;
      return 0;
    }
    else if(argc == 3) {  // use in command line
      window.setVideoPath(QString(argv[1]));
      window.setOutputFile(QString(argv[2]));
      window.Convert();
      return 0;
    }
    else if(argc == 4) {  // use in command line with a frame reduction
      window.setVideoPath(QString(argv[1]));
      window.setOutputFile(QString(argv[2]));
      window.setFrameReduction(QString(argv[3]).toInt());
      window.Convert();
      return 0;
    }
    else if(argc == 5) {  // use in command line with a frame reduction and switchinng each led of at end
      window.setVideoPath(QString(argv[1]));
      window.setOutputFile(QString(argv[2]));
      window.setFrameReduction(QString(argv[3]).toInt());
      if(strcmp(argv[3], "yes") == 0)
          window.setEndOff(true);
      window.Convert();
      return 0;
    }
    else  // use interface
      window.show();

    return app.exec();
}
